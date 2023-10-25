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


class Interceptor(grpc.ServerInterceptor):
    def intercept_service(self, continuation, handler_call_details):
        method_name = handler_call_details.method.split("/")[
            -1
        ]  # Extract the method name
        print(f"Received request for method: {handler_call_details.method}")

        # Get the method from the AgentMockServicer and call it
        servicer = AgentMockServicer()
        method = getattr(servicer, method_name, None)
        print(method, method_name)

        def custom_handler(request, context):
            try:
                if method:
                    return method(request, context)
                return continuation(handler_call_details)(request, context)
            except Exception as e:
                context.abort(grpc.StatusCode.INTERNAL, str(e))

        # Replace the original handler with the custom handler
        new_details = handler_call_details._replace(invocation_metadata=custom_handler)
        return continuation(new_details)


class AgentMockServicer:
    def GetAgentConfiguration(self, request, context):
        print("Received GetAgentConfiguration request:", request)
        return GetAgentConfigurationResponse(configuration={"key": "value"})

    def GetConfigBlobData(self, request, context):
        print("Received GetConfigBlobData request:", request)
        return GetConfigBlobDataResponse(blob_data={"blob_key": "blob_value"})

    def GetApplicationConfiguration(self, request, context):
        print(request, context)
        mock_config = ApplicationConfiguration(configuration_map={"key": "value"})
        return GetApplicationConfigurationResponse(configuration=mock_config)

    def GetCommandRequestStream(self, request, context):
        print(request, context)
        yield GetCommandRequestStreamResponse(
            request=CommandRequest(command="mock_command")
        )

    def SendCommandResponse(self, request, context):
        print(request, context)
        print("Received command response:", request.response.command)
        return SendCommandResponseResponse()

    def GetTeleopControlDataStream(self, request, context):
        print(request, context)
        yield GetTeleopControlDataStreamResponse()


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
