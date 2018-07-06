def saturation(value, low, high):
    if value > high:
        return high
    elif value < low:
        return low
    else:
        return value


def deadzone(value, low, high):
    if value < high:
        return 0
    elif value > low:
        return 0
    else:
        return value