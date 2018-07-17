"""
описание констант и методов конвертации единиц измерения
"""
import scipy.constants as const

g = const.g   # gravity
pi = const.pi
psc_bar = 1   # pressure standard condition
tsc_c = 15    # temperature standard condition
air_density_sckgm3 = 1.225 # definition from https://en.wikipedia.org/wiki/Density_of_air
z_default = 0.9
gamma_gas_default = 0.8
rsb_default_m3m3 = 100


def convert_pressure(value=1,from_dim='psi', to_dim='bar'):
    psi_list=['psi','PSI']
    if from_dim == 'psi' and to_dim == 'bar':
        return value * const.psi


def bar2psi(value=1):
    return value * const.bar / const.psi

def psi2bar(value=1):
    return value / const.bar * const.psi


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
