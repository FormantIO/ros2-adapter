import functools
import threading


def lock_decorator(lock_var):
    def decorator(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            lock = getattr(self, lock_var)
            with lock:
                return func(self, *args, **kwargs)

        return wrapped

    return decorator


def handle_value_error(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except ValueError as value_error:
            args[0]._logger.warn(value_error)
            return None

    return wrapper
