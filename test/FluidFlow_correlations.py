# -*- coding: utf-8 -*-
"""
Created on Wed Jul 26 2018

@author: Rinat Khabibullin
         Alexey Vodopyan
"""
import numpy as np
import unittest
import uconst as uc

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
    vel_gas_super0_ms = 1.2

    def dispersedbubble_equation(vel_gas_super):
        disp_eq = 2 * (0.4 * sigma_liq_Nm / ((rho_liq_kgm3 - rho_gas_kgm3) * uc.g)) ** 0.5 * \
                  (rho_liq_kgm3 / sigma_liq_Nm) ** (3 / 5) * (f / (2 * d_m)) ** (2 / 5) * (
                              vel_liq_super_ms + vel_gas_super) \
                  ** 1.2 - 0.725 + 4.15 * (vel_gas_super / (vel_gas_super + vel_liq_super_ms)) ** 0.5
        return disp_eq
    vel_gas_super_disp_ms = fsolve(dispersedbubble_equation, vel_gas_super0_ms)
    return vel_gas_super_disp_ms


def unf_velocity_dispersed2slug(vel_liq_super_ms):
    """
    Функция для нахождения критической скорости газа соответствующей переходу от dispersed to slug structure (линия C)

    ref: A Comprehensive Mechanistic Model for Upward Two-Phase Flow in Wellbores
    A.M. Ansari, Pakistan Petroleum Ltd.; N.D. Sylvester, U. of Akron; and C. Sarica, O. Shoham, and J.P. Brill, 1994

    :param vel_liq_super_ms:    superficial liquid velocity, m/s
    :return:                    superficial gas velocity (transition to slug), m/s
    """
    vel_gas_super_disp2slug_ms = 3.17 * vel_liq_super_ms
    return vel_gas_super_disp2slug_ms


def unf_velocity_slug2annular():
    pass


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
