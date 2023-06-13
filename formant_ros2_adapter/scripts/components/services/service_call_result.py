from enum import Enum


class ResultType(Enum):
    CONFIG_NOT_FOUND = 0
    SUCCESS = 1
    FAIL = 2
    TIMEOUT = 3
    INVALID_ARGUMENTS = 4


class ServiceCallResult:
    def __init__(self, type: ResultType, message: str, service_name: str):
        self.type = type
        self.message = message
        self.serivce_name = service_name
