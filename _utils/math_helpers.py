def clamp(value, min_value, max_value):
    """Keeps a value with in a set interval"""
    return max(min(value, max_value), min_value)
