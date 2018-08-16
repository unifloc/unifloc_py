"""
Created on Mon Aug 13 2018

@author: Rinat Khabibullin
         Alexey Vodopyan
"""
import numpy as np
import uscripts.uconst as uc
import scipy.optimize as opt


def diameter_min(vel_gas, rho_liq, sigma, rho_gas, f_m, beta, vel_liq):
    """
    function for determining minimal diameter between d_cd - critical bubble size above which the bubble is deformed,
    and d_cb - critical bubble size below which migration of bubbles to the upper part of the pipe is prevented

    :param vel_gas: superficial gas velocity
    :param rho_liq: liquid density
    :param sigma: surface tension
    :param rho_gas: gas density
    :param f_m: friction factor
    :param beta: angle of inclination from the horizontal
    :param vel_liq: superficial liquid velocity
    :return: minimal diameter
    """

    d_cd = 2 * ((0.4 * sigma) / ((rho_liq - rho_gas) * uc.g)) ** 0.5
    if beta == 90:
        d_c = d_cd
    else:
        d_cb = (3 / 8) * (rho_liq / (rho_liq - rho_gas)) * f_m * (vel_gas + vel_liq) ** 2 / \
            (uc.g * np.cos(beta * uc.pi / 180))
        d_c = min(d_cd, d_cb)
    return d_c


def reynolds_number(rho, vel, d_m, mu):
    """
    function for determining Reynolds number

    :param rho: density
    :param vel: velocity
    :param d_m: pipe diameter
    :param mu: viscosity
    :return: Reynolds number
    """
    return rho * vel * d_m / mu


def determing_coefficients(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq):
    """
    function for determining coefficients m, n, c_l, c_g

    :param d_m: pipe diameter
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :param vel_gas: superficial gas velocity
    :param vel_liq: superficial liquid velocity
    :return: array of c_l, n, c_g, m
    """
    if reynolds_number(rho_liq, vel_liq, d_m, mu_liq) < 2300:
        c_l = 16
        n = 1
    else:
        c_l = 0.046
        n = 0.2
    if reynolds_number(rho_gas, vel_gas, d_m, mu_gas) < 2300:
        c_g = 16
        m = 1
    else:
        c_g = 0.046
        m = 0.2
    return np.array([c_l, n, c_g, m])


def parameter_x(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq):
    """
    Function for determining Lockhart and Martinelli parameter X

    :param d_m: pipe diameter
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :param vel_gas: superficial gas velocity
    :param vel_liq: superficial liquid velocity
    :return: Lockhart and Martinelli parameter X
    """
    a = determing_coefficients(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
    c_l = a[0]
    n = a[1]
    c_g = a[2]
    m = a[3]
    x = (((4 * c_l / d_m) * reynolds_number(rho_liq, vel_liq, d_m, mu_liq) ** (-n) * (rho_liq * vel_liq ** 2 / 2)) /
         ((4 * c_g / d_m) * reynolds_number(rho_gas, vel_gas, d_m, mu_gas) ** (-m) * (rho_gas * vel_gas **
                                                                                      2 / 2))) ** 0.5
    return x


def parameter_y(d_m, rho_liq, rho_gas, mu_gas, vel_gas, beta):
    """
    Function for determining Lockhart and Martinelli parameter Y

    :param d_m: pipe diameter
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :param mu_gas: gas viscosity
    :param vel_gas: superficial gas velocity
    :param beta: angle of inclination from the horizontal
    :return: Lockhart and Martinelli parameter Y
    """
    if reynolds_number(rho_gas, vel_gas, d_m, mu_gas) < 2300:
        c_g = 16
        m = 1
    else:
        c_g = 0.046
        m = 0.2
    y = ((rho_liq - rho_gas) * uc.g * np.sin(beta * uc.pi / 180)) / ((4 * c_g / d_m) * reynolds_number(rho_gas, vel_gas,
                                                                                                       d_m, mu_gas) **
                                                                     (-m) * (rho_gas * vel_gas ** 2 / 2))
    return y


def combined_momentum_equation(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq, beta):
    """
    Function for determining equilibrium dimensionless liquid level in stratified flow

    :param d_m: pipe diameter
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :param vel_gas: superficial gas velocity
    :param vel_liq: superficial liquid velocity
    :param beta: angle of inclination from the horizontal
    :return: h_l
    """
    x = parameter_x(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
    y = parameter_y(d_m, rho_liq, rho_gas, mu_gas, vel_gas, beta)
    a = determing_coefficients(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
    n = a[1]
    m = a[3]
    h_l_0 = 0.42

    def equation2solve(h_l):
        a_l = 0.25 * (uc.pi - (np.cos(2 * h_l - 1)) ** (-1) + (2 * h_l - 1) * (1 - (2 * h_l - 1) ** 2) ** 0.5)
        a_g = 0.25 * ((np.cos(2 * h_l - 1)) ** (-1) - (2 * h_l - 1) * (1 - (2 * h_l - 1) ** 2) ** 0.5)
        s_l = uc.pi - (np.cos(2 * h_l - 1)) ** (-1)
        s_g = (np.cos(2 * h_l - 1)) ** (-1)
        s_i = (1 - (2 * h_l - 1) ** 2) ** 0.5
        v_l = (0.25 * uc.pi) / a_l
        v_g = (0.25 * uc.pi) / a_g
        d_l = 4 * a_l / s_l
        d_g = 4 * a_g / (s_g + s_i)
        equation = x ** 2 * ((v_l * d_l) ** (-n) * v_l ** 2 * s_l / a_l) - ((v_g * d_g) ** (-m) * v_g ** 2 *
                                                                            (s_g / a_g + s_i / a_l + s_i / a_g)) + 4 * y
        return equation
    h_l = opt.fsolve(equation2solve, np.array(h_l_0))
    return h_l


def bubble2slug(vel_gas, rho_liq, rho_gas, sigma, beta):
    """
    function for construction of boundary transition from bubble to slug structure

    :param vel_gas: superficial gas velocity
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :param sigma: surface tension
    :param beta: angle of inclination from the horizontal
    :return: superficial liquid velocity
    """
    return 3 * vel_gas - 1.1475 * (uc.g * (rho_liq - rho_gas) * sigma / rho_liq ** 2) ** 0.25 * np.sin(beta * uc.pi/180)


def dispersedbubble2churn(vel_gas):
    """
    function for construction of boundary transition from dispersed bubble to churn structure

    :param vel_gas: superficial gas velocity
    :return: superficial liquid velocity
    """
    return vel_gas * 0.923076923


def dispersedbubble2bubble(vel_gas, rho_liq, sigma, rho_gas, f_m, d_m, beta):
    """
    function for construction of boundary transition from dispersed bubble to bubble structure

    :param vel_gas: superficial gas velocity
    :param rho_liq: liquid density
    :param sigma: surface tension
    :param rho_gas: gas density
    :param f_m: friction factor
    :param d_m: pipe diameter
    :param beta: angle of inclination from the horizontal
    :return: superficial liquid velocity
    """
    vel_liq_0 = 1

    def equation2solve(vel_liq):
        d_c = diameter_min(vel_gas, rho_liq, sigma, rho_gas, f_m, beta, vel_liq)
        equation = (0.725 + 4.15 * (vel_gas / (vel_gas + vel_liq)) ** 0.5) * (sigma / rho_liq) ** 0.6 *\
                   ((2 * f_m) / d_m * (vel_gas + vel_liq) ** 3) ** (-0.4) - d_c
        return equation
    vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    return vel_liq


def churn2annular(sigma, rho_liq, rho_gas):
    """
    function for construction of boundary transition from churn to annular structure

    :param sigma: surface tension
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :return: superficial gas velocity
    """
    return 3.1 * ((uc.g * sigma * (rho_liq - rho_gas)) / rho_gas ** 2) ** 0.25


def stratified2nonstratified(rho_gas, rho_liq, vel_gas, d_m, beta):
    """
    """
    (rho_gas / (rho_liq - rho_gas)) ** 0.5 * vel_gas / (d_m * uc.g * np.cos(beta * uc.pi / 180)) ** 0.5
    pass


def slug2churn(vel_gas, rho_liq, sigma, rho_gas, f_m, d_m, beta):
    """
    function for construction of boundary transition from slug to churn structure

    :param vel_gas: superficial gas velocity
    :param rho_liq: liquid density
    :param sigma: surface tension
    :param rho_gas: gas density
    :param f_m: friction factor
    :param d_m: pipe diameter
    :param beta: angle of inclination from the horizontal
    :return: superficial liquid velocity
    """
    vel_liq_0 = 1

    def equation2solve(vel_liq):
        d_c = diameter_min(vel_gas, rho_liq, sigma, rho_gas, f_m, beta, vel_liq)
        equation = 0.058 * (d_c * (2 * f_m / d_m * (vel_gas + vel_liq) ** 3) ** 0.4 * (rho_liq / sigma) ** 0.6 -
                            0.725) ** 2 - 0.52
        return equation
    vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    return vel_liq


def slug2elongatedbubble(vel_gas, rho_liq, sigma, rho_gas, f_m, d_m, beta):
    """
    function for construction of boundary transition from slug to elongated bubble structure

    :param vel_gas: superficial gas velocity
    :param rho_liq: liquid density
    :param sigma: surface tension
    :param rho_gas: gas density
    :param f_m: friction factor
    :param d_m: pipe diameter
    :param beta: angle of inclination from the horizontal
    :return: superficial liquid velocity
    """
    vel_liq_0 = 1

    def equation2solve(vel_liq):
        d_c = diameter_min(vel_gas, rho_liq, sigma, rho_gas, f_m, beta, vel_liq)
        equation = 0.058 * (d_c * (2 * f_m / d_m * (vel_gas + vel_liq) ** 3) ** 0.4 * (rho_liq / sigma) ** 0.6 -
                            0.725) ** 2
        return equation

    vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    return vel_liq



