import grpc
from concurrent import futures
from models_mock import (
    GetCommandRequestStreamResponse,
    CommandRequest,
    SendCommandResponseRequest,
    SendCommandResponseResponse,
)


class AgentMockServicer:
    def GetCommandRequestStream(self, request, context):
        yield GetCommandRequestStreamResponse(
            request=CommandRequest(command="mock_command")
        )

    def SendCommandResponse(self, request, context):
        print("Received command response:", request.response.command)
        return SendCommandResponseResponse()


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    # This is a mock step since we're not using actual gRPC generated code
    # Normally, you'd add the servicer using a function generated from the .proto file
    setattr(server, "AgentMockServicer", AgentMockServicer())

    server.add_insecure_port("[::]:50051")
    server.start()
    print("AgentMock Server started on port 50051")
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
