"""
Created on Mon Aug 13 2018

@author: Rinat Khabibullin
         Alexey Vodopyan
"""
import numpy as np
import uniflocpy.uTools.uconst as uc
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


def combined_momentum_equation(x, y, d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq):
    """
    Function for determining equilibrium dimensionless liquid level in stratified flow

    :param x: Lockhart and Martinelli parameter X
    :param y: Lockhart and Martinelli parameter Y
    :param d_m: pipe diameter
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :param vel_gas: superficial gas velocity
    :param vel_liq: superficial liquid velocity
    :return: h_l
    """
    a = determing_coefficients(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
    n = a[1]
    m = a[3]
    h_l_0 = 0.42

    def equation2solve(h_l):
        a_l = 0.25 * (uc.pi - np.arccos(2 * h_l - 1) + (2 * h_l - 1) * (1 - (2 * h_l - 1) ** 2) ** 0.5)
        a_g = 0.25 * (np.arccos(2 * h_l - 1) - (2 * h_l - 1) * (1 - (2 * h_l - 1) ** 2) ** 0.5)
        s_l = uc.pi - np.arccos(2 * h_l - 1)
        s_g = np.arccos(2 * h_l - 1)
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


def dimensionless_variables(h_l):
    """
    Function for determining dimensionless variables which depending on dimensionless liquid level in stratified flow

    :param h_l: dimensionless liquid level in stratified flow
    :return: a_l, a_g, s_l, s_g, s_i, v_l, v_g, d_l, d_g
    """
    a_l = 0.25 * (uc.pi - np.arccos(2 * h_l - 1) + (2 * h_l - 1) * (1 - (2 * h_l - 1) ** 2) ** 0.5)
    a_g = 0.25 * (np.arccos(2 * h_l - 1) - (2 * h_l - 1) * (1 - (2 * h_l - 1) ** 2) ** 0.5)
    s_l = uc.pi - np.arccos(2 * h_l - 1)
    s_g = np.arccos(2 * h_l - 1)
    s_i = (1 - (2 * h_l - 1) ** 2) ** 0.5
    v_l = (0.25 * uc.pi) / a_l
    v_g = (0.25 * uc.pi) / a_g
    d_l = 4 * a_l / s_l
    d_g = 4 * a_g / (s_g + s_i)
    return a_l, a_g, s_l, s_g, s_i, v_l, v_g, d_l, d_g


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
        equation = (0.725 + 4.15 * (vel_gas / (vel_gas + vel_liq)) ** 0.5) * (sigma / rho_liq) ** 0.6 *\
                   ((f_m * (vel_gas + vel_liq) ** 3) / (2 * d_m)) ** (-0.4) - 2 * (0.4 * sigma / ((rho_liq - rho_gas)
                                                                                                  * uc.g)) ** 0.5
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


def stratified2nonstratified(rho_gas, rho_liq, vel_gas, d_m, beta, mu_liq, mu_gas):
    """
    function for construction of boundary transition from stratified to nonstratified structure

    :param rho_gas: gas density
    :param rho_liq: liquid density
    :param vel_gas: superficial gas velocity
    :param d_m: pipe diameter
    :param beta: angle of inclination from the horizontal
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :return: superficial liquid velocity
    """

    froude_number = (rho_gas / (rho_liq - rho_gas)) ** 0.5 * vel_gas / (d_m * uc.g * np.cos(beta * uc.pi / 180)) ** 0.5
    vel_liq_0 = 0.005
    
    def equation2solve(vel_liq):
        x = parameter_x(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
        y = parameter_y(d_m, rho_liq, rho_gas, mu_gas, vel_gas, beta)
        h_l = combined_momentum_equation(x, y, d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
        variables = dimensionless_variables(h_l)
        v_g = variables[6]
        s_i = variables[4]
        a_g = variables[1]
        equation = froude_number ** 2 * (v_g ** 2 * s_i / ((1 - h_l) ** 2 * a_g)) - 1
        return equation
    vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    return vel_liq


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


def slug2elongatedbubble(vel_gas, rho_liq, sigma, rho_gas, f_m, d_m, beta, vel_liq):
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
    # vel_liq_0 = 1

    def equation2solve(vel_liq):
        d_c = diameter_min(vel_gas, rho_liq, sigma, rho_gas, f_m, beta, vel_liq)
        equation = 0.058 * (d_c * (2 * f_m / d_m * (vel_gas + vel_liq) ** 3) ** 0.4 * (rho_liq / sigma) ** 0.6 -
                            0.725) ** 2
        return equation
    func = equation2solve(vel_liq)
    # vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    return func


def stratifiedsmooth2stratifiedwavy_c(rho_gas, rho_liq, vel_gas, d_m, beta, mu_liq, mu_gas):
    """
    function for construction of boundary transition from stratified-smooth to stratified-wavy structure
    resulting from the "wind" effect

    :param rho_gas: gas density
    :param rho_liq: liquid density
    :param vel_gas: superficial gas velocity
    :param d_m: pipe diameter
    :param beta: angle of inclination from the horizontal
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :return: superficial liquid velocity
    """
    froude_number = (rho_gas / (rho_liq - rho_gas)) ** 0.5 * vel_gas / (d_m * uc.g * np.cos(beta * uc.pi / 180)) ** 0.5
    vel_liq_0 = 0.0000001

    def equation2solve(vel_liq):
        re_sl = reynolds_number(rho_liq, vel_liq, d_m, mu_liq)
        k = froude_number * re_sl ** 0.5
        # k = froude_number ** 2 * re_sl
        x = parameter_x(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
        y = parameter_y(d_m, rho_liq, rho_gas, mu_gas, vel_gas, beta)
        h_l = combined_momentum_equation(x, y, d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
        variables = dimensionless_variables(h_l)
        v_g = variables[6]
        s = 0.01
        v_l = variables[5]
        equation = k - 2 / (v_l ** 0.5 * v_g * s ** 0.5)
        return equation
    vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    return vel_liq


def stratifiedsmooth2stratifiedwavy_m(rho_gas, rho_liq, vel_gas, d_m, beta, mu_liq, mu_gas):
    """
    function for construction of boundary transition from stratified-smooth to stratified-wavy structure
    resulting from the "wind" effect

    :param rho_gas: gas density
    :param rho_liq: liquid density
    :param vel_gas: superficial gas velocity
    :param d_m: pipe diameter
    :param beta: angle of inclination from the horizontal
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :return: superficial liquid velocity
    """
    froude_number = (rho_gas / (rho_liq - rho_gas)) ** 0.5 * vel_gas / (d_m * uc.g * np.cos(beta * uc.pi / 180)) ** 0.5
    vel_liq_0 = 0.00001

    def equation2solve(vel_liq):
        re_sl = reynolds_number(rho_liq, vel_liq, d_m, mu_liq)
        k = froude_number * re_sl ** 0.5
        x = parameter_x(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
        y = parameter_y(d_m, rho_liq, rho_gas, mu_gas, vel_gas, beta)
        h_l = combined_momentum_equation(x, y, d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
        variables = dimensionless_variables(h_l)
        a_l = variables[0]
        a_g = variables[1]
        a = a_l + a_g
        equation = vel_liq / (uc.g * d_m) ** 0.5 - 1.5 * (h_l ** 0.5 * a_l / a)
        return equation
    vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    return vel_liq


def annular2intermittent(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq):
    """
    function for construction of boundary transition from annular to intermittent structure

    :param d_m: pipe diameter
    :param rho_liq: liquid density
    :param rho_gas: gas density
    :param mu_liq: liquid viscosity
    :param mu_gas: gas viscosity
    :param vel_gas: superficial gas velocity
    :return: superficial liquid velocity
    """
    # vel_liq_0 = 1
    sigma_l = 0.24
    h_l = 4 * (sigma_l - sigma_l ** 2)

    def equation2solve(vel_liq):
        x = parameter_x(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
        equation = (1 + 75 * h_l) / ((1 - h_l) ** 2.5 * h_l) - (1 / h_l ** 3) * x ** 2 -\
                   ((2 - 1.5 * h_l) * x ** 2) / (h_l ** 3 * (1 - 1.5 * h_l))
        return equation
    func = equation2solve(vel_liq)
    # vel_liq = opt.fsolve(equation2solve, np.array(vel_liq_0))
    # vel_liq = opt.least_squares(equation2solve, np.array(vel_liq_0), bounds=(0, 10000))
    return func


def str2nonstr_dim(h_l):
    variables = dimensionless_variables(h_l)
    v_g = variables[6]
    s_i = variables[4]
    a_g = variables[1]
    froude_number_0 = 0.001

    def equation2solve(froude_number):
        equation = froude_number ** 2 * (v_g ** 2 * s_i / ((1 - h_l) ** 2 * a_g)) - 1
        return equation
    froude_number = opt.fsolve(equation2solve, np.array(froude_number_0))
    return froude_number


def strw2srts_dim(h_l):
    variables = dimensionless_variables(h_l)
    v_g = variables[6]
    s = 0.01
    v_l = variables[5]
    k_0 = 1

    def equation2solve(k):
        equation = k - 2 / (v_l ** 0.5 * v_g * s ** 0.5)
        return equation
    k = opt.fsolve(equation2solve, np.array(k_0))
    return k


def annular(rho_gas, rho_liq, vel_gas, d_m, beta, mu_liq, mu_gas, sigma_liq, vel_liq_0):

    def equation2solve_general(vel_liq):
        f_e = 1 - np.exp(-0.125 * ((10 ** 4 * vel_gas * mu_gas / sigma_liq) * (rho_gas / rho_liq) ** 0.5 - 1.5))
        n_ref = rho_liq * vel_liq * d_m * (1 - f_e) / mu_liq
        n_resl = reynolds_number(rho_liq, vel_liq, d_m, mu_liq)
        if n_ref < 2300 and n_ref != 0:
            f_f = 64 / n_ref
        elif n_ref != 0:
            f_f = 0.184 * n_ref ** (-0.2)
        else:
            f_f = 1
        if n_resl < 2300 and n_resl != 0:
            f_sl = 64 / n_resl
        elif n_resl != 0:
            f_sl = 0.184 * n_resl ** (-0.2)
        else:
            f_sl = 1

        v_sc = vel_gas + vel_liq * f_e
        lyambda_lc = vel_liq * f_e / (vel_gas + vel_liq * f_e)
        rho_c = rho_gas * (1 - lyambda_lc) + rho_liq * lyambda_lc
        mu_c = mu_gas * (1 - lyambda_lc) + mu_liq * lyambda_lc
        n_resc = rho_c * v_sc * d_m / mu_c
        if n_resc < 2300 and n_resc != 0:
            f_sc = 64 / n_resc
        else:
            f_sc = 0.184 * n_resc ** (-0.2)
        c_l, n_l, c_c, n_c = determing_coefficients(d_m, rho_liq, rho_c, mu_liq, mu_c, v_sc, vel_liq)
        gradient_sl = - f_sl * rho_liq * vel_liq ** 2 / (2 * d_m)
        gradient_sc = -f_sc * rho_c * v_sc ** 2 / (2 * d_m)
        # gradient_sl = -c_l * 4 * (rho_liq * vel_liq * d_m / mu_liq) ** (-n_l) * (rho_liq * vel_liq ** 2) / (2 * d_m)
        # gradient_sc = -c_c * 4 * (rho_c * v_sc * d_m / mu_c) ** (-n_c) * (rho_c * v_sc ** 2) / (2 * d_m)
        x = (-gradient_sl / -gradient_sc) ** 0.5
        y = (rho_liq - rho_c) * uc.g * np.sin(beta * uc.pi / 180) / (-gradient_sc)

        h_lf_0 = 0.2

        def equation2solve(h_lf):
            delta_0 = 0.05

            def equation2solve_0(delta):
                equation = 4 * delta ** 2 - 4 * delta + h_lf
                return equation
            delta = opt.fsolve(equation2solve_0, delta_0)
            if f_e > 0.9:
                i = (1 + 300 * delta)
            else:
                i = (1 + 24 * (rho_liq / rho_gas) ** (1/3) * delta)
            equation = x ** 2 * (1 - f_e) ** 2 / h_lf ** 3 * (f_f / f_sl) - i / (h_lf * (1 - h_lf) ** 2.5) + y
            return equation
        h_lf = opt.fsolve(equation2solve, np.array(h_lf_0))
        # h_lf = opt.least_squares(equation2solve, np.array(h_lf_0), bounds=(0, 100000))
        h_lf_min_0 = 0.25

        def equation2solve_2(h_lf_min):
            equation = y - ((2 - 1.5 * h_lf_min) * (1 - f_e) ** 2) / (h_lf_min ** 3 * (1 - 1.5 * h_lf_min)) *\
                       (f_f / f_sl) * x ** 2
            return equation
        # h_lf_min = opt.least_squares(equation2solve_2, np.array(h_lf_min_0), bounds=(0, 100000))
        h_lf_min = opt.fsolve(equation2solve_2, np.array(h_lf_min_0))
        equation = h_lf[0] - h_lf_min[0]
        return equation
    vel_liq = opt.fsolve(equation2solve_general, np.array(vel_liq_0))
    # vel_liq = opt.least_squares(equation2solve_general, np.array(vel_liq_0), bounds=(0, 100000))
    return vel_liq
