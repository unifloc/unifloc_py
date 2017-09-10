"""
описание констант и методов конвертации единиц измерения
"""
import scipy.constants as const

g = const.g   # gravity
pi = const.pi 

def psi2pa(value=1):
    return value * const.psi


def bar2psi(value=1):
    return value * const.bar / const.psi


def bar2atm(value=1):
    return value * const.bar / const.atm


def atm2bar(value=1):
    return value * const.atm / const.bar


def c2f(value):
    return const.convert_temperature(value, 'C', 'F')


def f2c(value):
    return const.convert_temperature(value, 'F', 'K')


def c2k(value):
    return const.convert_temperature(value, 'C', 'K')


def m3_2_bbl(value=1):
    return value / const.barrel


def bbl2m3(value=1):
    return value / const.barrel


def m3m3_to_m3t(value, Gamma=1):
    pass
