# -*- coding: utf-8 -*-
"""
Created on Wed Jul 26 2018

@author: Rinat Khabibullin
         Alexey Vodopyan
"""
import numpy as np
import uscripts.uconst as uc
import scipy.optimize as opt

# Функции для расчета градиента давления по корреляции Ансари


def unf_velocity_bubble2slug(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, vel_liq_super_ms):
    """
    Функция для нахождения критической скорости газа соответствующей переходу от bubble to slug structure (линия B)

    ref: A Comprehensive Mechanistic Model for Upward Two-Phase Flow in Wellbores
    A.M. Ansari, Pakistan Petroleum Ltd.; N.D. Sylvester, U. of Akron; and C. Sarica, O. Shoham, and J.P. Brill, 1994

    :param sigma_liq_Nm:        liquid surface tension, N/m
    :param rho_liq_kgm3:        liquid density, kg/m3
    :param rho_gas_kgm3:        gas density, kg/m3
    :param vel_liq_super_ms:    superficial liquid velocity, m/s
    :return:                    superficial gas velocity (transition to slug), m/s
    """
    v_slip_ms = 1.53 * ((uc.g * sigma_liq_Nm * (rho_liq_kgm3 - rho_gas_kgm3)) / rho_liq_kgm3 ** 2) ** 0.25
    vel_gas_super_slug_ms = 0.25 * v_slip_ms + 0.333 * vel_liq_super_ms
    return vel_gas_super_slug_ms


def unf_velocity_dispersed2bubble(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, f, vel_liq_super_ms, d_m):
    """
    Функция для нахождения критической скорости газа соответствующей переходу от dispersed to bubble structure (линия A)

    ref: A Comprehensive Mechanistic Model for Upward Two-Phase Flow in Wellbores
    A.M. Ansari, Pakistan Petroleum Ltd.; N.D. Sylvester, U. of Akron; and C. Sarica, O. Shoham, and J.P. Brill, 1994

    :param sigma_liq_Nm:        liquid surface tension, N/m
    :param rho_liq_kgm3:        liquid density, kg/m3
    :param rho_gas_kgm3:        gas density, kg/m3
    :param f:                   friction factor from Moody diagram
    :param vel_liq_super_ms:    superficial liquid velocity, m/s
    :param d_m:                 diameter of pipe, m
    :return:                    superficial gas velocity (transition to bubble), m/s
    """
    vel_gas_super0_ms = np.log10(vel_liq_super_ms)

    def dispersedbubble_equation(vel_gas_super):

        disp_eq = 2 * (0.4 * sigma_liq_Nm / ((rho_liq_kgm3 - rho_gas_kgm3) * uc.g)) ** 0.5 * \
            (rho_liq_kgm3 / sigma_liq_Nm) ** (3 / 5) * (f / (2 * d_m)) ** (2 / 5) * (vel_liq_super_ms + vel_gas_super) \
            ** 1.2 - 0.725 - 4.15 * (vel_gas_super / (vel_gas_super + vel_liq_super_ms)) ** 0.5

        return disp_eq
    vel_gas_super_disp_ms = opt.fsolve(dispersedbubble_equation, np.array(vel_gas_super0_ms))
    return vel_gas_super_disp_ms


def unf_velocity_dispersed2slug(vel_liq_super_ms):
    """
    Функция для нахождения критической скорости газа соответствующей переходу от dispersed to slug structure (линия C)

    ref: A Comprehensive Mechanistic Model for Upward Two-Phase Flow in Wellbores
    A.M. Ansari, Pakistan Petroleum Ltd.; N.D. Sylvester, U. of Akron; and C. Sarica, O. Shoham, and J.P. Brill, 1994

    :param vel_liq_super_ms:    superficial liquid velocity, m/s
    :return:                    superficial gas velocity (  transition to slug), m/s
    """
    vel_gas_super_disp2slug_ms = 3.17 * vel_liq_super_ms
    return vel_gas_super_disp2slug_ms


def unf_velocity_slug2annular(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, vel_gas_super_ms, vel_liq_super_ms, mu_gas_cP,
                              mu_liq_cP, d_m, e_m):
    """
    Функция для нахождения критической скорости газа соответствующей переходу от slug to annular structure (линия D)

    ref: A Comprehensive Mechanistic Model for Upward Two-Phase Flow in Wellbores
    A.M. Ansari, Pakistan Petroleum Ltd.; N.D. Sylvester, U. of Akron; and C. Sarica, O. Shoham, and J.P. Brill, 1994

    :param sigma_liq_Nm:
    :param rho_liq_kgm3:
    :param rho_gas_kgm3:
    :param vel_gas_super_ms:
    :param vel_liq_super_ms:
    :param mu_gas_cP:
    :param mu_liq_cP:
    :param d_m:
    :param e_m:
    :return:                    superficial gas velocity (transition to annular), m/s
    """
    mu_liq_Pas = mu_liq_cP / 1000
    mu_gas_Pas = mu_gas_cP / 1000
    vel_gas_super_slug2annular = 3.1 * ((uc.g * sigma_liq_Nm * (rho_liq_kgm3 - rho_gas_kgm3)) / rho_gas_kgm3 ** 2)**0.25
    vel_critical = 10000 * vel_gas_super_ms * mu_gas_Pas / sigma_liq_Nm * (rho_gas_kgm3 / rho_liq_kgm3) ** 0.5
    # Объемная доля захваченных газом капель жидкости
    f_e = 1 - np.exp(-0.125 * (vel_critical - 1.5))
    lyambda_lc = f_e * vel_liq_super_ms / (f_e * vel_liq_super_ms + vel_gas_super_ms)
    # Плотность и вязкость флюида в газовом ядре
    rho_c = rho_liq_kgm3 * lyambda_lc + rho_gas_kgm3 * (1 - lyambda_lc)
    mu_sc_Pas = (mu_liq_Pas * lyambda_lc + mu_gas_Pas * (1 - lyambda_lc))
    # Приведенная скорость газового ядра
    vel_sc = f_e * vel_liq_super_ms + vel_gas_super_ms
    # Число Рейнольдса для газового ядра для определения fsc по диаграмме Муди
    n_re_sc = rho_c * vel_sc * d_m / mu_sc_Pas
    # Относительная шероховатость
    e2d = e_m / d_m
    # Определяем fsc по диаграмме Муди
    f_sc = 0.0152  # Пока так TODO нужно оцифровать графики Муди
    # Определяем число Рейнольдса для для пленки жидкости
    n_re_ls = rho_liq_kgm3 * vel_liq_super_ms * d_m / mu_liq_Pas
    # Определяем fsl по диаграмме Муди
    f_sl = 0.01708  # Пока так TODO нужно оцифровать графики Муди
    # Градиенты давления для пленки жидкости и газового ядра
    dpdl_sl = f_sl * rho_liq_kgm3 * vel_liq_super_ms ** 2 / (2 * d_m)
    dpdl_sc = f_sc * rho_c * vel_sc ** 2 / (2 * d_m)
    # Модифицированные параметры Локхарта Мартинелли
    f_f = f_sl  # TODO этот момент не понятен, почему так берут и что такое f_f
    x_m = ((1 - f_e) ** 2 * (f_f / f_sl) * dpdl_sl / dpdl_sc) ** 0.5
    theta = 90  # TODO можно потом попробовать поиграться с углом, хот эта корреляция для вертикальной трубы
    y_m = uc.g * np.sin(theta * uc.pi / 180) * (rho_liq_kgm3 - rho_c) / dpdl_sc
    delta_0 = 0.25

    def equation(delta):
        if f_e > 0.9:
            equation2solve = y_m - (1 + 300 * delta) / ((1 - 4 * delta * (1 - delta)) ** 2.5 * 4 * delta *
                                                      (1 - delta)) + x_m ** 2 / (4 * delta * (1 - delta)) ** 3
        else:
            equation2solve = y_m - (1 + 24 * (rho_liq_kgm3 / rho_gas_kgm3) ** (1/3) * delta) /\
                         ((1 - 4 * delta * (1 - delta)) ** 2.5 * 4 * delta * (1 - delta)) +\
                         x_m ** 2 / (4 * delta * (1 - delta)) ** 3
        return equation2solve
    delta = opt.newton(equation, delta_0)
    critery1 = (4 * delta * (1 - delta)) + lyambda_lc * (d_m - 2 * delta * d_m) ** 2 / d_m ** 2
    delta_0 = 0.25

    def equation(delta):
        equation2solve = y_m - (2 - 1.5 * 4 * delta * (1 - delta)) * x_m ** 2 / ((4 * delta * (1 - delta)) ** 3 *
                                                                                 (1 - 1.5 * 4 * delta*(1 - delta)))
        return equation2solve
    delta_min = opt.newton(equation, delta_0)
    critery2 = delta_min - delta
    return vel_gas_super_slug2annular, critery1, critery2



def unf_flowpattern_prediction(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, f, vel_gas_super_ms, vel_liq_super_ms, d_m):
    """
    Функция для определения структуры потока газожидкостной смеси
    :param sigma_liq_Nm:
    :param rho_liq_kgm3:
    :param rho_gas_kgm3:
    :param f:
    :param vel_gas_super_ms:
    :param vel_liq_super_ms:
    :param d_m:
    :return:
    """
    d_m_min = 19.01 * (((rho_liq_kgm3 - rho_gas_kgm3) * sigma_liq_Nm)/(rho_liq_kgm3 ** 2 * uc.g)) ** 0.5
    # Проверка соотношения для эмульсионной структуры потока
    if a == b:
        structure = 'dispersed bubble'
    else:
        structure = 'not dispersed bubble'
    # Проверка соотношения для кольцевой структуры потока
    vel_gas_super_ann = 3.1 * ((uc.g * sigma_liq_Nm * (rho_liq_kgm3 - rho_gas_kgm3))/rho_gas_kgm3 ** 2) ** 0.25
    pass


def unf_bubbleflow_model():

    pass


def unf_slugflow_model():
    pass


def unf_annularflow_model():
    pass


def unf_Ansarigradient():
    pass
