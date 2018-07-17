"""
описание констант и методов конвертации единиц измерения для унифлока
"""
import scipy.constants as const
import numpy as np

g = const.g   # gravity
pi = const.pi
psc_bar = 1   # pressure standard condition
tsc_c = 15    # temperature standard condition
const_at = 98066.5 # теническая атмосфера в Па
air_density_sckgm3 = 1.225 # definition from https://en.wikipedia.org/wiki/Density_of_air
z_default = 0.9
gamma_gas_default = 0.8
rsb_default_m3m3 = 100


def convert_pressure(val, old_scale, new_scale):
    """
    Convert from a pressure scale to another one among psi, bar,
    atm scales.


    based on scipy.constants.convert_temperature
    Parameters
    ----------
    val : array_like
        Value(s) of the pressure(s) to be converted expressed in the
        original scale.

    old_scale: str
        Specifies as a string the original scale from which the pressure
        value(s) will be converted. Supported scales are atm ('atm',
        'Atm', 'C' or 'c'), psi ('psi'), bar ('bar') and Pa
        ('Pa').

    new_scale: str
        Specifies as a string the new scale to which the temperature
        value(s) will be converted. Supported scales are atm ('atm',
        'Atm', 'C' or 'c'), psi ('psi'), bar ('bar') and Pa
        ('Pa').

    Returns
    -------
    res : float or array of floats
        Value(s) of the converted pressure(s) expressed in the new scale.

    Notes
    -----
    .. versionadded:: 0.1

    Examples
    --------
    >>> import uconst
    >>> uconst.convert_pressure(1,'atm','psi')
    14.69594877551345

    """
    # Convert from `old_scale` to Pa
    if old_scale.lower() in ['bar', 'бар']:
        tempo = bar2Pa(np.asanyarray(val))
    elif old_scale.lower() in ['atm', 'атм']:
        tempo = atm2Pa(np.asanyarray(val))
    elif old_scale.lower() in ['psi']:
        tempo = psi2Pa(np.asanyarray(val))
    elif old_scale.lower() in ['at', 'ат', 'kgssm2', 'кгссм2']:
        tempo = atm2Pa(np.asanyarray(val))
    elif old_scale.lower() in ['mpa', 'мпа']:
        tempo = np.asanyarray(val) * const.mega
    else:
        raise NotImplementedError("%s scale is unsupported: supported scales "
                                  "are bar, atm, at and, MPa" % old_scale)
    # and from Kelvin to `new_scale`.
    if new_scale.lower() in ['bar', 'бар']:
        res = Pa2bar(tempo)
    elif new_scale.lower() in ['atm', 'атм']:
        res = Pa2atm(tempo)
    elif new_scale.lower() in ['at', 'ат', 'kgssm2', 'кгссм2']:
        res = Pa2at(tempo)
    elif new_scale.lower() in ['psi']:
        res = Pa2psi(tempo)
    elif new_scale.lower() in ['mpa', 'мпа']:
        res = tempo / const.mega
    else:
        raise NotImplementedError("'%s' scale is unsupported: supported scales "
                                  "are bar, atm, at and, MPa" % new_scale)

    return res


def convert_length(val, old_scale, new_scale):
    # TODO надо сделать конвертер для расстояний. Общая функция и набор быстрых функций
    pass


def convert_rate(val, old_scale, new_scale):
    # TODO надо сделать конвертер для дебитов (объемных расходов). Общая функция и набор быстрых функций
    pass


def convert_GOR(val, old_scale, new_scale, gamma_oil=0.86):
    # TODO надо сделать конвертер для газового фактора, газосодержания. Общая функция и набор быстрых функций
    # в том числе надо чтобы поддерживалась конвертация между м3/м3 и м3/т
    pass


# simple unit conversion functions
def psi2Pa(value):
    """
    converts pressure in psi to Pa
    :param value: pressure value in psi
    :return: pressure value in Pa
    """
    return value * const.psi


def Pa2psi(value):
    """
    converts pressure in psi to Pa
    :param value: pressure value in psi
    :return: pressure value in Pa
    """
    return value / const.psi


def bar2Pa(value):
    """
    converts pressure in bar to Pa
    :param value: pressure value in bar
    :return: pressure value in Pa
    """
    return value * const.bar


def Pa2bar(value):
    """
    converts pressure in Pa to bar
    :param value: pressure value in Pa
    :return: pressure value in bar
    """
    return value / const.bar


def atm2Pa(value):
    """
    converts pressure in atm (standard atmosphere) to Pa
    :param value: pressure value in atm
    :return: pressure value in Pa
    """
    return value * const.atm


def Pa2atm(value):
    """
    converts pressure in Pa to atm (standard atmosphere)
    :param value: pressure value in Pa
    :return: pressure value in atm (standard atmosphere)
    """
    return value / const.atm


def at2Pa(value):
    """
    converts pressure in at (technical atmosphere) to Pa
    :param value: pressure value in atm
    :return: pressure value in Pa
    """
    return value * const_at


def Pa2at(value):
    """
    converts pressure in Pa (Pascal) to at (technical atmosphere)
    :param value: pressure value in Pa
    :return: pressure value in at (technical atmosphere)
    """
    return value * const_at


def bar2psi(value):
    return value * const.bar / const.psi


def psi2bar(value):
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


def m3m3_2_m3t(value, gamma=1):
    return value * gamma
