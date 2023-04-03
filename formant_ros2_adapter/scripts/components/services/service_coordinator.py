from enum import Enum
import os
import time
from typing import List, Dict, Optional
import rclpy
from rclpy.node import Node
from rclpy.client import Client

from formant.sdk.agent.v1 import Client as FormantClient

from configuration.config_schema import ConfigSchema
from configuration.service_client_config import ServiceClientConfig
from ros2_utils.topic_type_provider import TopicTypeProvider
from ros2_utils.qos import QOS_PROFILES, qos_profile_system_default
from ros2_utils.logger import get_logger
from message_utils.utils import (
    get_ros2_type_from_string,
    get_message_path_value,
)
from .service_call_result import ServiceCallResult, ResultType
from .prepare_service_request import prepare_serivce_request


FORMANT_OVERRIDE_TIMESTAMP = (
    os.getenv("FORMANT_OVERRIDE_TIMESTAMP", "").lower() == "true"
)
SERVICE_CALL_TIMEOUT = 5


class ServiceCoordinator:
    def __init__(
        self,
        fclient: FormantClient,
        node: Node,
        topic_type_provider: TopicTypeProvider,
    ):
        self._fclient = fclient
        self._node = node
        self._topic_type_provider = topic_type_provider
        self._service_clients: Dict[str, List[Client]] = {}
        self._logger = get_logger()

    def setup_with_config(self, config: ConfigSchema):
        self._logger.info("Setting up Service Coordinator")
        self._config = config
        self._cleanup()
        if self._config.service_clients:
            for service_config in self._config.service_clients:
                try:
                    self._setup_services_for_config(service_config)
                except ValueError as value_error:
                    self._logger.warn(value_error)
                    continue
        self._logger.info("Set up Service Coordinator")

    def _setup_services_for_config(self, service_config: ServiceClientConfig):
        service_name = service_config.service
        formant_stream = service_config.formant_stream

        service_type = service_config.service_type
        if service_type is None:
            service_type = self._topic_type_provider.get_service_type_for_name(
                service_name
            )
        if service_type is None:
            raise ValueError("No Service type found for %s" % service_name)

        self._logger.debug(
            "Setting up service %s, %s, %s"
            % (service_name, formant_stream, service_type)
        )
        try:
            new_service_client = self._node.create_client(
                srv_type=service_type,
                srv_name=service_name,
                callback_group=None,
            )

            if formant_stream not in self._service_clients:
                self._service_clients[formant_stream] = []

            self._service_clients[formant_stream].append(new_service_client)
        except Exception as e:
            self._logger.warn("Failed to set up service client for %s" % service_name)

    def call_service(self, formant_stream, parameter: str):
        service_call_results: List[ServiceCallResult] = []

        for service_client in self._service_clients.get(formant_stream, []):
            if service_client.wait_for_service(SERVICE_CALL_TIMEOUT) == False:
                service_call_results.append(
                    ServiceCallResult(
                        ResultType.TIMEOUT,
                        "Timeout waiting for service",
                        service_client.srv_name,
                    )
                )
                continue
            try:
                request = prepare_serivce_request(service_client, parameter)
            except ValueError as val_error:
                service_call_results.append(
                    ServiceCallResult(
                        ResultType.INVALID_ARGUMENTS,
                        str(val_error),
                        service_client.srv_name,
                    )
                )
                continue

            service_result = service_client.call(request)
            self._logger.info("Service call result: %s" % str(service_result))
            service_call_results.append(
                ServiceCallResult(
                    ResultType.SUCCESS,
                    str(service_result),
                    service_client.srv_name,
                )
            )
        return service_call_results

    def _cleanup(self):
        for service_clients in self._service_clients.items():
            for service_client in service_clients[1]:
                self._node.destroy_client(service_client)
        self._subscriptions = {}
