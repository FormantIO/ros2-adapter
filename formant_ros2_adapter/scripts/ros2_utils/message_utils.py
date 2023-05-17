import array
import codecs
import importlib
import json
import logging
import numpy as np
import re

try:
    bool_type = np.bool_
except Exception:
    bool_type = np.bool

NUMPY_DTYPE_TO_BUILTIN_MAPPING = {
    np.uint8: int,
    np.uint16: int,
    np.uint32: int,
    np.uint64: int,
    np.int8: int,
    np.int16: int,
    np.int32: int,
    np.int64: int,
    np.float32: float,
    np.float64: float,
    bool_type: bool,
    bool: bool,
}


logger = logging.getLogger("formant_ros2_adapter")
logger.setLevel(logging.DEBUG)


def get_ros2_type_from_string(message_type_string: str):
    """
    Returns a ROS2 message type for the provided ROS2 message type string
    """
    try:
        path = message_type_string.replace("/", ".").split(".")
        module_name = ".".join(path[:-1])
        module = importlib.import_module(module_name)
        return getattr(module, path[-1])
    except Exception as e:
        logger.warn(
            "Couldn't import ROS2 message type from string: %s %s"
            % (message_type_string, str(e)),
        )
        return None


def parse(m):
    if type(m) == float:
        if float(m) != float(m):
            return None
    if type(m) in [bool, str, int, float]:
        return m
    elif type(m) == bytes:
        return int(codecs.encode(m, "hex"), 16)
    elif type(m) in NUMPY_DTYPE_TO_BUILTIN_MAPPING.keys():
        return NUMPY_DTYPE_TO_BUILTIN_MAPPING[type(m)](m)
    elif type(m) in [list, array.array, np.ndarray]:
        return [parse(o) for o in m]
    elif m is None:
        return None
    else:
        return {k: parse(getattr(m, k)) for k in m._fields_and_field_types}


def message_to_json(message) -> str:
    """
    Converts any ROS2 message into a JSON string
    """
    return json.dumps(parse(message))


rosMessagePathRegex = re.compile(
    r"^([a-zA-Z])([0-9a-zA-Z_])*(\[[0-9]+\])?(\.([a-zA-Z])([0-9a-zA-Z_])*(\[[0-9]+\])?)*"
)


def get_message_path_value(message, messagePath: str):
    """
    Returns the message value at the end of the provided message path
    """
    if not rosMessagePathRegex.fullmatch(messagePath):
        raise ValueError("Invalid message path in configuration: " + messagePath)

    # parse the message path
    steps = []
    for substring in messagePath.split("."):
        indexStart = None
        try:
            indexStart = substring.index("[")
        except ValueError:
            pass

        if indexStart:
            if indexStart == 0:
                steps.append(("indexing", substring))
            else:
                steps.append(("attribute", substring[:indexStart]))
                steps.append(("indexing", substring[indexStart:]))
        else:
            steps.append(("attribute", substring))

    # index based on the message path
    try:
        for step in steps:
            if step[0] == "attribute":
                message = getattr(message, step[1])
            elif step[0] == "indexing":
                indices = [int(s[1:]) for s in step[1].split("]")[:-1]]
                for index in indices:
                    message = message[index]
    except (AttributeError, KeyError):
        return None

    return message
