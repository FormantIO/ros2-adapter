class ApplicationConfiguration:
    def __init__(self, configuration_map=None):
        self.configuration_map = configuration_map or {}


class GetApplicationConfigurationResponse:
    def __init__(self, configuration):
        self.configuration = configuration


class GetAgentConfigurationResponse:
    def __init__(self, configuration):
        self.configuration = configuration


class GetConfigBlobDataResponse:
    def __init__(self, blob_data):
        self.blob_data = blob_data


class CommandRequest:
    def __init__(self, command):
        self.command = command


class GetCommandRequestStreamResponse:
    def __init__(self, request):
        self.request = request


class SendCommandResponseRequest:
    def __init__(self, response):
        self.response = response


class SendCommandResponseResponse:
    pass


class ControlDatapoint:
    def __init__(self, data="mock_data"):
        self.data = data


class GetTeleopControlDataStreamRequest:
    def __init__(self, stream_filter=None):
        self.stream_filter = stream_filter or []


class GetTeleopControlDataStreamResponse:
    def __init__(self, control_datapoint=None):
        self.control_datapoint = control_datapoint or ControlDatapoint()


class PostDataMultiResponse:
    def __init__(self, errors=None):
        self.errors = errors or []


class PostDataError:
    def __init__(self, index, code, retryable, message):
        self.index = index
        self.code = code
        self.retryable = retryable
        self.message = message
