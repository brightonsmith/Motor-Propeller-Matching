import matplotlib.pyplot as plt
import numpy as np
import os


def interpolate(x, x1, x2, y1, y2):
    return (x - x1) * ((y2 - y1) / (x2 - x1)) + y1
