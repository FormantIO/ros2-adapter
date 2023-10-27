import grpc
from concurrent import futures
from formant.protos.agent.v1 import agent_pb2, agent_pb2_grpc
from formant.protos.model.v1 import commands_pb2
from models_mock import (
    CommandRequest,
    ApplicationConfiguration,
)


class AgentMockServicer(agent_pb2_grpc.AgentServicer):
    def __init__(self):
        super().__init__()

        self.post_datapoints = []

    def GetAgentConfiguration(self, request, context):
        return agent_pb2.GetApplicationConfigurationResponse(request)

    def GetConfigBlobData(self, request, context):
        return agent_pb2.GetConfigBlobDataResponse(blob_data={"blob_key": "blob_value"})

    def GetApplicationConfiguration(self, request, context):
        return agent_pb2.GetApplicationConfigurationResponse(configuration=request)

    def GetCommandRequestStream(self, request, context):
        print(type(request))
        command_request = commands_pb2.CommandRequest()
        yield agent_pb2.GetCommandRequestStreamResponse(request=request)

    def SendCommandResponse(self, request, context):
        return agent_pb2.SendCommandResponseResponse()

    def GetTeleopControlDataStream(self, request, context):
        yield agent_pb2.GetTeleopControlDataStreamResponse()

    def PostDataMulti(self, request, context):
        print("hey", request.datapoints)
        self.post_datapoints.append(request.datapoints)
        return agent_pb2.PostDataMultiResponse()

    def StreamData(self, request, context):
        return agent_pb2.StreamDataResponse()


def serve(servicer: AgentMockServicer):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    agent_pb2_grpc.add_AgentServicer_to_server(servicer, server)

    server.add_insecure_port("[::]:50051")
    server.start()
    print("AgentMock Server started on port 50051")

    return server
