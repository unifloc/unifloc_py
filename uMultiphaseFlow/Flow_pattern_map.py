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

    :param rho:
    :param vel:
    :param d_m:
    :param mu:
    :return:
    """
    return rho * vel * d_m / mu


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



