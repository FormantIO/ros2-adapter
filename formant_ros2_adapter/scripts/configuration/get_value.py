def get_value(config, key, cls=None, required=False, is_array=False):
    if required:
        data = config.get(key, None)
        if data is None:
            raise ValueError(
                "Could not get required field: %s from: %s. Config is invalid"
                % (key, config)
            )
    data = config.get(key, None)
    if data is None:
        return data
    if is_array and cls:
        return [cls(item) for item in data]
    if cls:
        return cls(data)
    return data
