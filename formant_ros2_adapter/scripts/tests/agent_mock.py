import grpc
from formant.protos.agent.v1 import agent_pb2, agent_pb2_grpc
from formant.protos.model.v1 import commands_pb2, datapoint_pb2, math_pb2, config_pb2
from concurrent import futures
from models_mock import Command
import time
import threading
import json

MAX_AMOUNT = 1000000000
ADAPTER_NAME = "ros2_adapter_configuration"
DEFAULT_STREAM = "Buttons"


class AgentMockServicer(agent_pb2_grpc.AgentServicer):
    def __init__(self):
        super().__init__()
        self.post_datapoints = []
        self.transform_frames = []
        self.command_requests = []
        self.i = 0

        self.config = self._get_config_from_json()

    def GetAgentConfiguration(self, request, context):
        return agent_pb2.GetAgentConfigurationResponse(
            configuration=config_pb2.AgentConfiguration()
        )

    def GetConfigBlobData(self, request, context):
        return agent_pb2.GetConfigBlobDataResponse(blob_data=config_pb2.BlobData())

    def GetApplicationConfiguration(self, request, context):
        return agent_pb2.GetApplicationConfigurationResponse(
            configuration=config_pb2.ApplicationConfiguration()
        )

    def GetTeleopControlDataStream(self, request, context):
        # Check if the client is interested in the "Buttons" stream
        if DEFAULT_STREAM in request.stream_filter:
            while True:
                yield agent_pb2.GetTeleopControlDataStreamResponse(
                    control_datapoint=datapoint_pb2.ControlDatapoint(
                        stream=DEFAULT_STREAM,
                        timestamp=1234567890,  # Mock timestamp
                        numeric=math_pb2.Numeric(value=42.0),
                    )
                )  # Mock data for the "Buttons" stream
                time.sleep(MAX_AMOUNT)

        else:
            # Handle other stream filters or return an error
            pass

    def GetCommandRequestStream(self, request, context):
        self._mock_buttons(request, context)
        yield from self._mock_command_request()
        yield from self._mock_service_clients(request)

    def SendCommandResponse(self, request, context):
        return agent_pb2.SendCommandResponseResponse()

    def PostDataMulti(self, request: agent_pb2.PostDataMultiRequest, context):
        # Add the datapoints from the request to the list

        self.post_datapoints.append(request)
        context.set_code(grpc.StatusCode.OK)
        context.set_details("Request processed successfully")

        return agent_pb2.PostDataMultiResponse()

    def PostData(self, request, context):
        # Add the datapoint from the request to the list
        self.post_datapoints.append(request)
        context.set_code(grpc.StatusCode.OK)
        context.set_details("Data point processed successfully")

        # Return a simple response to the client
        return agent_pb2.PostDataResponse()

    def SetBaseFrameID(self, request, context):
        # Here we simulate setting the base frame ID by simply storing it
        context.set_code(grpc.StatusCode.OK)
        context.set_details("Base frame ID set successfully")
        # Return an empty response (as per the protobuf definition)
        return agent_pb2.SetBaseFrameIDResponse()

    def PostTransformFrame(self, request, context):
        # Add the transform frame from the request to the list
        self.transform_frames.append(request)
        context.set_code(grpc.StatusCode.OK)
        context.set_details("Transform frame processed successfully")

        # Return a simple response to the client
        return agent_pb2.PostTransformFrameResponse()

    def _get_config_from_json(self):
        # Try to get config from config.json
        print("Trying to get config from config.json file")
        try:
            with open("config.json") as f:
                config = json.loads(f.read())
                if "ros2_adapter_configuration" in config:
                    print("Got config from config.json file")
                    return config
        except Exception as e:
            print("Error getting config from config.json: %s" % str(e))
        return None

    def _mock_buttons(self, request, context):
        if DEFAULT_STREAM not in request.command_filter:
            return
        buttons_thread = threading.Thread(
            target=self._handle_buttons_stream, args=(context,)
        )
        buttons_thread.daemon = True
        buttons_thread.start()

        # Join the thread to ensure it runs as long as the context is alive
        buttons_thread.join()

    def _mock_command_request(self):
        for command in self.command_requests:
            yield agent_pb2.GetCommandRequestStreamResponse(
                request=commands_pb2.CommandRequest(
                    id=command.id, command=command.name, text=command.value
                )
            )

    def _mock_service_clients(self, request):
        ros2_service_types_response = {
            "formant_test_interfaces/srv/NoParameters": "",
            "example_interfaces/srv/SetBool": "True",
            "formant_test_interfaces/srv/SingleInt": "1",
            "formant_test_interfaces/srv/SingleFloat": "1.0",
            "formant_test_interfaces/srv/SingleString": "exampleString",
        }
        service_clients = self.config[ADAPTER_NAME].get("service_clients", None)
        if not service_clients:
            yield None
        
        for service_client in service_clients:
            yield agent_pb2.GetCommandRequestStreamResponse(
                request=commands_pb2.CommandRequest(
                    id="_",
                    command=service_client["formant_stream"],
                    text=ros2_service_types_response[
                        service_client["ros2_service_type"]
                    ],
                )
            )

    def _handle_buttons_stream(self, context):
        while True:
            yield agent_pb2.GetCommandRequestStreamResponse(
                request=commands_pb2.CommandRequest(
                    id="ButtonsId", command="Buttons", text="ButtonsText"
                )
            )
            time.sleep(MAX_AMOUNT)


class RequestInterceptor(grpc.ServerInterceptor):
    def intercept_service(self, continuation, handler_call_details):
        # Print the handler call details
        # print("Handler Call Details:", handler_call_details)

        # Continue with the given service handler
        return continuation(handler_call_details)


def serve(servicer: AgentMockServicer):
    interceptor = RequestInterceptor()
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=10), interceptors=[interceptor]
    )
    agent_pb2_grpc.add_AgentServicer_to_server(servicer, server)
    server.add_insecure_port("[::]:50051")
    print("Server started at [::]:50051")
    server.start()
    return server
