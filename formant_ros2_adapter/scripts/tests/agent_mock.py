import grpc
from formant.protos.agent.v1 import agent_pb2, agent_pb2_grpc
from formant.protos.model.v1 import commands_pb2, datapoint_pb2, math_pb2
from concurrent import futures
from google.protobuf import text_format
import time

MAX_AMOUNT = 1000000000


class AgentMockServicer(agent_pb2_grpc.AgentServicer):
    def __init__(self):
        super().__init__()
        self.post_datapoints = []
        self.i = 0

    def GetAgentConfiguration(self, request, context):
        return agent_pb2.GetAgentConfigurationResponse(
            configuration=agent_pb2.AgentConfiguration(request)
        )

    def GetConfigBlobData(self, request, context):
        return agent_pb2.GetConfigBlobDataResponse(
            blob_data=agent_pb2.BlobData(request)
        )

    def GetApplicationConfiguration(self, request, context):
        return agent_pb2.GetApplicationConfigurationResponse(
            configuration=agent_pb2.ApplicationConfiguration(request)
        )

    def GetTeleopControlDataStream(self, request, context):
        # Check if the client is interested in the "Buttons" stream
        if "Buttons" in request.stream_filter:
            while True:
                yield agent_pb2.GetTeleopControlDataStreamResponse(
                    control_datapoint=datapoint_pb2.ControlDatapoint(
                        stream="Buttons",
                        timestamp=1234567890,  # Mock timestamp
                        numeric=math_pb2.Numeric(value=42.0),  # Mock timestamp
                        bitset=datapoint_pb2.Bitset(
                            bits=[datapoint_pb2.Bit(key="SomeButtonKey", value=True)]
                        ),
                    )
                )  # Mock data for the "Buttons" stream
                time.sleep(MAX_AMOUNT)

        else:
            # Handle other stream filters or return an error
            pass

    def GetCommandRequestStream(self, request, context):

        # Check if the client is interested in the "Buttons" stream
        if "Buttons" in request.command_filter:
            while True:
                # Generate and yield appropriate responses based on the "Buttons" filter
                # For example:
                yield agent_pb2.GetCommandRequestStreamResponse(
                    request=commands_pb2.CommandRequest(
                        id="ButtonsId", command="Buttons", text="ButtonsText"
                    )
                )
                time.sleep(MAX_AMOUNT)
        else:
            # Handle other command filters or return an error
            pass

    def SendCommandResponse(self, request, context):
        return agent_pb2.SendCommandResponseResponse(request)

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


"""

if __name__ == "__main__":
    servicer = AgentMockServicer()
    server = serve(servicer)
    server.wait_for_termination()
"""
