import math


def logistic(x):
    return 1.0 / (1.0 + math.exp(-x))


def interpolate_logistic(start, stop, steps):
    x = [float(i) / steps for i in range(steps)]
    return [(stop - start) * logistic(8.0 * i - 4.0) + start for i in x]


def interpolate_quadratic(start, stop, steps):
    return []

