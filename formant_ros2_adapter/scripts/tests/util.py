import grpc


def default_unary_unary_handler(request, context):
    context.set_code(grpc.StatusCode.OK)
    return None


def default_unary_stream_handler(request, context):
    context.set_code(grpc.StatusCode.OK)
    yield None


def default_stream_unary_handler(request_iterator, context):
    context.set_code(grpc.StatusCode.OK)
    return None


def default_stream_stream_handler(request_iterator, context):
    context.set_code(grpc.StatusCode.OK)
    yield None
