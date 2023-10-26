import grpc
from concurrent import futures
from models_mock import (
    GetCommandRequestStreamResponse,
    CommandRequest,
    GetTeleopControlDataStreamResponse,
    SendCommandResponseResponse,
    ApplicationConfiguration,
    GetApplicationConfigurationResponse,
    GetAgentConfigurationResponse,
    GetConfigBlobDataResponse,
)
from util import (
    default_stream_stream_handler,
    default_stream_unary_handler,
    default_unary_stream_handler,
    default_unary_unary_handler,
)


def ok_method(*args, **kwargs):
    context = args[-1]  # context is always the last argument
    context.set_code(grpc.StatusCode.OK)
    return None


class Interceptor(grpc.ServerInterceptor):
    def intercept_service(self, continuation, handler_call_details):
        method_name = handler_call_details.method.split("/")[-1]
        print(f"Received request for method: {handler_call_details.method}")

        servicer = AgentMockServicer()
        method = getattr(servicer, method_name, None)

        if not method:

            def unimplemented_method(*args, **kwargs):
                context = args[-1]  # context is always the last argument
                context.set_code(grpc.StatusCode.UNIMPLEMENTED)
                context.set_details(f"Method {method_name} not implemented.")
                return None

            return grpc.unary_unary_rpc_method_handler(unimplemented_method)

        # Directly return the RpcMethodHandler from the servicer method
        return method(
            None, None
        )  # Passing None for request and context as they are not usede


class AgentMockServicer:
    def GetAgentConfiguration(self, request, context):
        print("Received GetAgentConfiguration request:", request)
        return grpc.unary_unary_rpc_method_handler(ok_method)

    def GetConfigBlobData(self, request, context):
        print("Received GetConfigBlobData request:", request)
        return grpc.unary_unary_rpc_method_handler(ok_method)

    def GetApplicationConfiguration(self, request, context):
        return grpc.unary_unary_rpc_method_handler(ok_method)

    def GetCommandRequestStream(self, request, context):
        return grpc.unary_unary_rpc_method_handler(ok_method)

    def SendCommandResponse(self, request, context):
        return grpc.unary_unary_rpc_method_handler(ok_method)

    def GetTeleopControlDataStream(self, request, context):
        return grpc.unary_unary_rpc_method_handler(ok_method)


def serve():
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=10), interceptors=[Interceptor()]
    )
    server.add_insecure_port("[::]:50051")
    server.start()
    print("AgentMock Server started on port 50051")
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
