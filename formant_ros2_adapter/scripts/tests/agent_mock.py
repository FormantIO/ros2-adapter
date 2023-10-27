import grpc
from concurrent import futures
from formant.protos.agent.v1 import agent_pb2, agent_pb2_grpc
from formant.protos.model.v1 import (
    commands_pb2,
    datapoint_pb2,
    event_pb2,
    math_pb2,
    media_pb2,
    navigation_pb2,
    health_pb2,
    text_pb2,
    intervention_pb2,
    file_pb2,
)
from models_mock import (
    GetCommandRequestStreamResponse,
    CommandRequest,
    GetTeleopControlDataStreamResponse,
    SendCommandResponseResponse,
    ApplicationConfiguration,
    GetApplicationConfigurationResponse,
    GetAgentConfigurationResponse,
    GetConfigBlobDataResponse,
    PostDataMultiResponse,
)
from util import (
    default_stream_stream_handler,
    default_stream_unary_handler,
    default_unary_stream_handler,
    default_unary_unary_handler,
)
import json


def ok_method(*args, **kwargs):
    context = args[-1]  # context is always the last argument
    context.set_code(grpc.StatusCode.OK)
    return None


class Interceptor(grpc.ServerInterceptor):
    def intercept_service(self, continuation, handler_call_details):
        method_name = handler_call_details.method.split("/")[-1]

        servicer = AgentMockServicer()
        method = getattr(servicer, method_name, None)

        if not method:

            def unimplemented_method(*args, **kwargs):
                context = args[-1]  # context is always the last argument
                context.set_code(grpc.StatusCode.UNIMPLEMENTED)
                context.set_details(f"Method {method_name} not implemented.")
                return None

            return grpc.unary_unary_rpc_method_handler(unimplemented_method)

        def wrapped_handler(request, context):
            # Here, you have access to the actual request and context
            print(request, context)
            return method(request, context)

        # Return the wrapped handler as the RpcMethodHandler
        return grpc.unary_unary_rpc_method_handler(wrapped_handler)


class AgentMockServicer(agent_pb2_grpc.AgentServicer):
    def GetAgentConfiguration(self, request, context):
        print("Received GetAgentConfiguration request:", request)
        return agent_pb2.GetApplicationConfigurationResponse(request)

    def GetConfigBlobData(self, request, context):
        print("Received GetConfigBlobData request:", request)
        return agent_pb2.GetConfigBlobDataResponse(blob_data={"blob_key": "blob_value"})

    def GetApplicationConfiguration(self, request, context):
        mock_config = ApplicationConfiguration(configuration_map={"key": "value"})
        return agent_pb2.GetApplicationConfigurationResponse(configuration=mock_config)

    def GetCommandRequestStream(self, request, context):
        yield agent_pb2.GetCommandRequestStreamResponse(
            request=CommandRequest(command="mock_command")
        )

    def SendCommandResponse(self, request, context):
        print("Received command response:", request.response.command)
        return agent_pb2.SendCommandResponseResponse()

    def GetTeleopControlDataStream(self, request, context):
        yield agent_pb2.GetTeleopControlDataStreamResponse()

    def PostDataMulti(self, request, context):
        print(request, context)
        return agent_pb2.PostDataMultiResponse()

    def StreamData(self, request, context):
        # Mock implementation for StreamData
        # Depending on whether it's unary or streaming, you can return or yield a mock response
        # For this example, I'll assume it's a unary method:
        return agent_pb2.StreamDataResponse()


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    agent_pb2_grpc.add_AgentServicer_to_server(AgentMockServicer(), server)

    server.add_insecure_port("[::]:50051")
    server.start()
    print("AgentMock Server started on port 50051")
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
