def validate_range(value, min_value, max_value, name):
    if not (min_value <= value <= max_value):
        raise ValueError(f"{name} value {value} is out of range ({min_value}-{max_value}).")
