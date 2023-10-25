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
