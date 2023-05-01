import json
from rclpy.client import Client, SrvTypeRequest

from .service_call_result import ServiceCallResult, ResultType
from utils.logger import get_logger


logger = get_logger()


def prepare_serivce_request(service_client: Client, parameter: str) -> SrvTypeRequest:

    service_request = service_client.srv_type.Request()
    service_request_types = list(service_request.get_fields_and_field_types().values())

    # TODO: allow for parsing of json into multiple slots
    logger.debug("Service request attribute types: %s" % service_request_types)
    if len(service_request_types) > 1:
        return ServiceCallResult(
            ResultType.INVALID_ARGUMENTS,
            "Too many request slots",
            service_client.srv_name,
        )

    if service_request_types == []:
        return service_request
    attribute_name = list(service_request.get_fields_and_field_types().keys())[0]
    attribute_type = service_request_types[0]
    if attribute_type == "boolean":
        request_value = get_bool_value(parameter)

    elif attribute_type == "string":
        request_value = get_string_value(parameter)

    elif attribute_type == "sequence<string>":
        request_value = get_string_sequence_value(parameter)

    # If the service has a single numeric parameter, call it with the command text
    # Float32, Float64, Int8, Int16, Int32, Int64, UInt8, UInt16, UInt32, UInt64
    elif attribute_type in [
        "float",
        "float32",
        "float64",
        "int",
        "int8",
        "int16",
        "int32",
        "int64",
        "uint8",
        "uint16",
        "uint32",
        "uint64",
    ]:
        request_value = get_numeric_value(parameter, attribute_type)

    else:
        raise ValueError("Unsupported slot type: %s" % attribute_type)

    setattr(
        service_request,
        attribute_name,
        request_value,
    )

    return service_request


def get_bool_value(parameter):
    if parameter == "":
        service_request_value = True
    elif parameter in ["true", "True", "TRUE", "t", "T", "1"]:
        service_request_value = True
    elif parameter in ["false", "False", "FALSE", "f", "F", "0"]:
        service_request_value = False
    else:
        raise ValueError("Invalid parameter for boolean service: %s" % parameter)
    return service_request_value


def get_string_value(parameter):
    return parameter


def get_string_sequence_value(parameter):
    command_text_json = {}
    try:
        command_text_json = json.loads(parameter)
    except json.decoder.JSONDecodeError:
        raise ValueError(
            "Invalid parameter for string sequence service: %s is not JSON" % parameter
        )

    if type(command_text_json) is not list:
        raise ValueError(
            "Invalid parameter for string sequence service: %s is not a list"
            % parameter
        )
    return command_text_json


def get_numeric_value(parameter, attribute_type):

    try:
        float(parameter)
    except ValueError:
        raise ValueError("Invalid parameter for numeric service: %s" % parameter)

    try:
        if "int" in attribute_type:
            return int(float(parameter))
        elif "float" in attribute_type:
            return float(parameter)
    except ValueError:
        raise ValueError(
            "Invalid parameter for numeric service %s : %s"
            % (attribute_type, parameter)
        )
    raise ValueError("Unsupported slot type: %s" % attribute_type)
