import math
from scipy.optimize import fsolve
import numpy as np


def calc_resistances(nom_power__W, nom_voltage__V, nom_slip, nom_efficient,
                     motor_lamda, alfa_0 = 0.61, moments_division = 1.05):

    """
    Расчет сопротивлений Г-образной схемы замещения

    :param nom_power__W: номинальная мощность АД
    :param nom_voltage__V: номинальное напряджение АД
    :param nom_slip: номинальное скольжение АД
    :param nom_efficient: номинальный КПД АД
    :param motor_lamda: отношение максимального момента к номинальному
    :return: r_2_: активное сопротивление обмотки ротора приведенное к ротору
             r_1: активное сопротивление обмотки статора
             x_k: индуктивное сопротивление обмоток при неподвижном роторе
    """

    a = alfa_0/moments_division * (1 - nom_efficient) / nom_efficient * \
        (1 - nom_slip) / nom_slip - 1

    R_m = 3 * nom_voltage__V**2 * (1 - nom_slip) / \
        (2 * nom_power__W * (motor_lamda + moments_division - 1))

    G = 1 / nom_slip**2 + 2 * a / nom_slip + a**2

    b = (motor_lamda + moments_division - 1) / moments_division

    r_2 = R_m / G * (b/nom_slip + a + math.sqrt((b/nom_slip + a)**2 - G))
    r_1 = a * r_2
    x_k = math.sqrt((R_m-r_1)**2 - r_1**2)
    return r_2, r_1, x_k


def calc_idle(nom_voltage__V, nom_slip, nom_efficient, nom_cos, r_1__Om,
              r_2__Om, x_k__Om):
    """
    Расчет режима холостого хода при номинальном напряжении

    :param nom_voltage__V: номинальное напряджение АД
    :param nom_slip: номинальное скольжение АД
    :param nom_efficient: номинальный КПД АД
    :param nom_cos: номинальный коэффициент мощ
    :param r_1__Om: активное сопротивление статора
    :param r_2__Om: активное сопротивление ротора приведенное к статору
    :param x_k__Om: индуктивное сопротивление обмоток на режиме ХХ
    :return: no_load_current: ток ХХ
             cos_fi_x: коэффициент мощности при ХХ
    """
    # TODO: добавить расчет тока ХХ не при номинальном напряжении(U/f = const)
    moments_division = 1.05

    nom_power__W = 3 * nom_voltage__V**2 * (1 - nom_slip) * r_2__Om / \
                       nom_slip / (moments_division * (
                       (r_1__Om + r_2__Om / nom_slip)**2 + x_k__Om ** 2))

    P_1 = nom_power__W / nom_efficient
    I_1 = P_1 / (3 * nom_voltage__V * nom_cos)

    I_2 = nom_voltage__V / math.sqrt((r_1__Om + r_2__Om / nom_slip) ** 2 +
                                         x_k__Om ** 2)
    fi_nom = math.acos(nom_cos)

    fi_2 = math.atan(x_k__Om / (r_1__Om + r_2__Om / nom_slip))
    I_1x = math.sqrt(I_1 ** 2 + I_2 ** 2 - 2 *
                         I_1 * I_2 * math.cos(fi_nom - fi_2))

    fi_x = math.acos((I_1 * nom_cos - I_2 * math.cos(fi_2)) / I_1x)
    return I_1x, fi_x


def calc_g_circuit(slip, nom_power__W, nom_voltage__V, nom_slip, nom_efficient,
                   nom_cos, frequency__Hz, voltage__V, r_1__Om, r_2__Om,
                   x_k__Om, moments_division=1.05):
    """
    Расчет механической характеристики ассинхроного двигателя

    :param slip: скольжение
    :param nom_power__W: номинальная мощность АД
    :param nom_voltage__V: номинальное напряжение АД
    :param nom_slip: номинальное скольжение АД
    :param nom_efficient: номинальный КПД АД
    :param nom_cos: номинальный коэффициент мощности АД
    :param frequency__Hz: рабочая частота вращения
    :param voltage__V: рабочее напряжение
    :param r_1__Om: активное сопротивление статора
    :param r_2__Om: активное сопротивление ротора приведенное к статору
    :param x_k__Om: индуктивное сопротивление обмоток на режиме ХХ
    :return:M__Nm - момент вращения ротора
            cos_fi - коэффициент мощности
            efficient - КПД АД
            I1__A - потребляемый ток
            P_2__kW - развиваемая мощность статора
    """

    f_nom__Hz = frequency__Hz * (1 - nom_slip)
    M_nom__Nm = nom_power__W / (f_nom__Hz * 2 * math.pi)
    M0__Nm = M_nom__Nm * (moments_division - 1)
    M__Nm = 3 * voltage__V**2 * (r_2__Om / slip) /\
            (frequency__Hz * 2 * math.pi * ((r_1__Om + r_2__Om / slip)**2 +
                                            x_k__Om ** 2)) - M0__Nm

    I2__A = voltage__V / math.sqrt((r_1__Om + r_2__Om / slip)**2 + x_k__Om**2)

    fi_2 = math.atan(x_k__Om/(r_1__Om + r_2__Om/slip))

    Ix__A, fi_x = calc_idle(nom_voltage__V, nom_slip, nom_efficient, nom_cos,
                            r_1__Om, r_2__Om, x_k__Om)

    I1__A = math.sqrt(Ix__A**2 + I2__A**2 + 2 * Ix__A * I2__A *
                      math.cos(fi_x - fi_2))

    cos_fi = (Ix__A * math.cos(fi_x) + I2__A * math.cos(fi_2)) / I1__A
    P_1__kW = 3 * voltage__V * I1__A * cos_fi / 1e3
    P_2__kW = M__Nm * frequency__Hz * 2 * math.pi / 1e3
    efficient = P_2__kW / P_1__kW
    return M__Nm,  cos_fi, efficient, I1__A, P_2__kW


def motor_data_loading(motor_power__W, nom_power__W, nom_voltage__V, nom_slip,
                       nom_efficient, nom_cos, frequency__Hz, voltage__V,
                       r_1__Om, r_2__Om, x_k__Om, moments_division=1.05):
    """
    Расчет характеристик ПЭД в зависимости от загрузки

    :param motor_power__W: рабочая мощность ПЭД
    :param nom_power__W: номинальная мощность АД
    :param nom_voltage__V: номинальное напряжение АД
    :param nom_slip: номинальное скольжение АД
    :param nom_efficient: номинальный КПД АД
    :param nom_cos: номинальный коэффициент мощности АД
    :param frequency__Hz: рабочая частота вращения
    :param voltage__V: рабочее напряжение
    :param r_1__Om: активное сопротивление статора
    :param r_2__Om: активное сопротивление ротора приведенное к статору
    :param x_k__Om: индуктивное сопротивление обмоток на режиме ХХ
    :return:M__Nm - момент вращения ротора
            cos_fi - коэффициент мощности
            efficient - КПД АД
            I1__A - потребляемый ток
            slip - скольжение АД
    """

    def f(slip, args):
        motor_power__W, nom_power__W, nom_voltage__V, nom_slip, nom_efficient, \
        nom_cos, frequency__Hz, voltage__V, r_1__Om, r_2__Om, x_k__Om, \
        moments_division = args

        _, _, _, _, P_2__kW = calc_g_circuit(slip, nom_power__W, nom_voltage__V,
                                             nom_slip, nom_efficient, nom_cos,
                                             frequency__Hz, voltage__V, r_1__Om,
                                             r_2__Om, x_k__Om, moments_division)
        return P_2__kW - motor_power__W/1e3

    slip = fsolve(func=f, args=[motor_power__W, nom_power__W, nom_voltage__V,
                                nom_slip, nom_efficient, nom_cos, frequency__Hz,
                                voltage__V, r_1__Om, r_2__Om, x_k__Om,
                                moments_division], x0=nom_slip/2)

    M__Nm, cos_fi, efficient, I1__A, P_2__kW = calc_g_circuit(
        slip, nom_power__W, nom_voltage__V, nom_slip, nom_efficient, nom_cos,
        frequency__Hz, voltage__V, r_1__Om, r_2__Om, x_k__Om, moments_division)

    return M__Nm, cos_fi, efficient, I1__A, slip, P_2__kW
