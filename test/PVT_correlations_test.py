"""
tests for PVT_correlations functions
"""
import PVT_correlations as PVTfunc
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


def pb_Standing_test():
    print(PVTfunc.unf_pb_Valko_MPaa(15,0.8,0.86))

pb_Standing_test()