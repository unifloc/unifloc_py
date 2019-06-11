# -*- coding: utf-8 -*-
"""
Created on Wed Jul 26 2018

@author: Rinat Khabibullin
         Alexey Vodopyan
"""
import numpy as np
import uscripts.uconst as uc
import scipy.optimize as opt
from scipy.integrate import odeint
import uPVT.PVT as PVT

# Расчет распределения температуры в скважине

# Корреляция Беггса-Брилла для многофазного потока


def BHP_BeggsBrill(l, p1, t1, t2, d_m, q_liq_m3d, sigma_liq_Nm, theta, e, wct, rough_pipe=0, gamma_oil=0.86,
                   gamma_gas=0.6, gamma_wat=1.0, rsb_m3m3=200.0, gamma_gassp=0, y_h2s=0, y_co2=0, y_n2=0,
                   s_ppm=0, par_wat=0, pbcal_bar=-1., tpb_C=80, bobcal_m3m3=1.2, muobcal_cP=0.5):

    p_bar = pressure_drop_BeggsBrill(l, p1, t1, t2, d_m, q_liq_m3d, sigma_liq_Nm, theta, e, wct, rough_pipe, gamma_oil,
                             gamma_gas, gamma_wat, rsb_m3m3, gamma_gassp, y_h2s, y_co2, y_n2,
                             s_ppm, par_wat, pbcal_bar, tpb_C, bobcal_m3m3, muobcal_cP)[1]
    BHP_bar = p_bar[-1]
    return BHP_bar


def pressure_drop_BeggsBrill_odeint(l, p1, t1, t2, d_m, q_liq_m3d, sigma_liq_Nm, theta, e, wct, rough_pipe=0, gamma_oil=0.86,
                             gamma_gas=0.6, gamma_wat=1.0, rsb_m3m3=200.0, gamma_gassp=0, y_h2s=0, y_co2=0, y_n2=0,
                             s_ppm=0, par_wat=0, pbcal_bar=-1., tpb_C=80, bobcal_m3m3=1.2, muobcal_cP=0.5):
    """
    Расчет кривой распределения давления по Беггсу-Бриллу c применением odeint

    :param l:
    :param p1:
    :param t1:
    :param t2:
    :param d_m:
    :param q_liq_m3d:
    :param sigma_liq_Nm:
    :param theta:
    :param e:
    :param wct:
    :param rough_pipe:
    :param gamma_oil:
    :param gamma_gas:
    :param gamma_wat:
    :param rsb_m3m3:
    :param gamma_gassp:
    :param y_h2s:
    :param y_co2:
    :param y_n2:
    :param s_ppm:
    :param par_wat:
    :param pbcal_bar:
    :param tpb_C:
    :param bobcal_m3m3:
    :param muobcal_cP:
    :return:
    """
    step = 20
    l_array = np.linspace(0, l, step)
    q_oil_m3d = q_liq_m3d * (1 - wct)
    q_gas_m3d = q_oil_m3d * rsb_m3m3
    q_water_m3d = q_liq_m3d * wct
    t_array = np.linspace(t1, t2, step)

    def BB_gradient(l_array, t_array):

        fl = PVT.FluidMcCain(gamma_oil, gamma_gas, gamma_wat, rsb_m3m3, gamma_gassp, y_h2s, y_co2, y_n2,
                             s_ppm, par_wat, pbcal_bar, tpb_C, bobcal_m3m3, muobcal_cP)
        fl.calc(p1, t_array)
        q_liq = (q_oil_m3d * fl.bo_m3m3 + q_water_m3d * fl.bw_m3m3)
        q_gas = ((q_gas_m3d - fl.rs_m3m3 * q_oil_m3d) * fl.bg_m3m3)
        vel_gas_super_ms = q_gas / (86400 * uc.pi * d_m ** 2 / 4)
        vel_liq_super_ms = q_liq / (86400 * uc.pi * d_m ** 2 / 4)
        rho_liq_kgm3 = ((fl.rho_oil_kgm3 + fl.rs_m3m3 * fl.rho_gas_kgm3) / fl.bo_m3m3) * (1 - wct) + \
                       fl.rho_wat_kgm3 / fl.bw_m3m3 * wct
        rho_gas_kgm3 = fl.rho_gas_kgm3
        mu_liq_cP = fl.mu_oil_cP * (1 - wct) + fl.mu_wat_cP * wct
        mu_gas_cP = fl.mu_gas_cP
        dpdl = uc.Pa2bar(unf_BeggsBrill_gradient(vel_gas_super_ms, vel_liq_super_ms, rho_liq_kgm3, rho_gas_kgm3,
                                                 mu_liq_cP, mu_gas_cP, d_m, sigma_liq_Nm, theta, e, rough_pipe))
        return dpdl
    p_array = odeint(BB_gradient, p1, l_array, args=t_array)

    return l_array, p_array


def pressure_drop_BeggsBrill(l, p1, t1, t2, d_m, q_liq_m3d, sigma_liq_Nm, theta, e, wct, rough_pipe=0, gamma_oil=0.86,
                             gamma_gas=0.6, gamma_wat=1.0, rsb_m3m3=200.0, gamma_gassp=0, y_h2s=0, y_co2=0, y_n2=0,
                             s_ppm=0, par_wat=0, pbcal_bar=-1., tpb_C=80, bobcal_m3m3=1.2, muobcal_cP=0.5):
    """
    Расчет кривой распределения давления по Беггсу-Бриллу

    :param l:
    :param p1:
    :param t1:
    :param t2:
    :param d_m:
    :param q_liq_m3d:
    :param sigma_liq_Nm:
    :param theta:
    :param e:
    :param wct:
    :param rough_pipe:
    :param gamma_oil:
    :param gamma_gas:
    :param gamma_wat:
    :param rsb_m3m3:
    :param gamma_gassp:
    :param y_h2s:
    :param y_co2:
    :param y_n2:
    :param s_ppm:
    :param par_wat:
    :param pbcal_bar:
    :param tpb_C:
    :param bobcal_m3m3:
    :param muobcal_cP:
    :return:
    """
    p_calc = p1
    p_pvt = 0
    t_calc = t1
    step = 20
    delta_l = l / (step - 1)
    delta_t = (t2 - t1) / (step - 1)
    maxiter = 5
    l_calc = 0
    q_oil_m3d = q_liq_m3d * (1 - wct)
    q_gas_m3d = q_oil_m3d * rsb_m3m3
    q_water_m3d = q_liq_m3d * wct
    p_array = [p_calc]
    l_array = [l_calc]
    while l_calc < l:
        delta_p = 0
        counter = 0
        while abs(p_pvt - (p_calc + 0.5 * delta_p)) > 0.5 or counter > maxiter:
            p_pvt = p_calc + 0.5 * delta_p
            t_pvt = t_calc + 0.5 * delta_t
            fl = PVT.FluidMcCain(gamma_oil, gamma_gas, gamma_wat, rsb_m3m3, gamma_gassp, y_h2s, y_co2, y_n2,
                                 s_ppm, par_wat, pbcal_bar, tpb_C, bobcal_m3m3, muobcal_cP)
            fl.calc(p_pvt, t_pvt)
            q_liq = (q_oil_m3d * fl.bo_m3m3 + q_water_m3d * fl.bw_m3m3)
            q_gas = ((q_gas_m3d - fl.rs_m3m3 * q_oil_m3d) * fl.bg_m3m3)
            vel_gas_super_ms = q_gas / (86400 * uc.pi * d_m ** 2 / 4)
            vel_liq_super_ms = q_liq / (86400 * uc.pi * d_m ** 2 / 4)
            rho_liq_kgm3 = ((fl.rho_oil_kgm3 + fl.rs_m3m3 * fl.rho_gas_kgm3) / fl.bo_m3m3) * (1 - wct) +\
                fl.rho_wat_kgm3 / fl.bw_m3m3 * wct
            rho_gas_kgm3 = fl.rho_gas_kgm3
            mu_liq_cP = fl.mu_oil_cP * (1 - wct) + fl.mu_wat_cP * wct
            mu_gas_cP = fl.mu_gas_cP
            dpdl = uc.Pa2bar(unf_BeggsBrill_gradient(vel_gas_super_ms, vel_liq_super_ms, rho_liq_kgm3, rho_gas_kgm3, mu_liq_cP,
                                           mu_gas_cP, d_m, sigma_liq_Nm, theta, e, rough_pipe))
            delta_p = dpdl * delta_l
            counter += 1
        l_calc += delta_l
        l_array.append(l_calc)
        p_calc += delta_p
        p_array.append(p_calc)
        t_calc += delta_t
    return l_array, p_array


def unf_liguid_holdup_BeggsBrill(lyambda_l, flow_pattern, n_lv, n_fr, theta):
    """
    :param lyambda_l:
    :param flow_pattern:
    :param n_lv:
    :param n_fr:
    :param theta:
    :return:
    """
    a = np.array([0.98, 0.855, 1.065])
    b = (0.4846, 0.5351, 0.5824)
    c = (0.0868, 0.0173, 0.0609)
    e = (0.011, 2.96, 1, 4.7)
    f = (-3.768, 0.305, 0, -0.3692)
    g = (3.539, -0.4473, 0, 0.1244)
    h = (-1.614, 0.0978, 0, -0.5056)
    theta_rad = theta * uc.pi / 180
    h_l0 = a[flow_pattern] * lyambda_l ** b[flow_pattern] / n_fr ** c[flow_pattern]
    cc = max(0, (1 - lyambda_l) * np.log(e[flow_pattern] * lyambda_l ** f[flow_pattern] * n_lv ** g[flow_pattern] *
                                        n_fr ** h[flow_pattern]))
    incl_factor = 1 + cc * (np.sin(1.8 * theta_rad) - 0.333 * (np.sin(1.8 * theta_rad)) ** 3) # в экселе тут плюс стоит
    if theta > 0:
        h_l = max(min(h_l0 * incl_factor * 0.924, 1), lyambda_l)  # поправка Payne et al для восходящего потока
    else:
        h_l = min(h_l0 * incl_factor * 0.685, 1)  # поправка Payne et al для нисходящего потока
    return h_l


def unf_friction_factor_BeggsBrill(n_re, d_m, e, rough_pipe = 0):
    """

    :param n_re:
    :param d_m:
    :param e:
    :param rough_pipe:
    :return:
    """
    if n_re > 2000:
        if rough_pipe > 0:
            f_n_0 = (-2 * np.log10((2 * e / d_m) / 3.7 - 5.02 / n_re * np.log10((2 * e / d_m) / 3.7 + 13 / n_re
                                                                                ))) ** (-2)

            def equation2solve(f_n):
                f_n_new = (1.74 - 2 * np.log10(2 * e / d_m + 18.7 / (n_re * f_n ** 0.5))) ** (-2)
                equation = abs(f_n_new - f_n)
                return equation
            f_n = opt.fsolve(equation2solve, np.array(f_n_0), xtol=0.001)
        else:
            f_n = 0.0056 + 0.5 * n_re ** (-0.32)
    else:
        f_n = 64 / n_re
    return f_n


def unf_BeggsBrill_gradient(vel_gas_super_ms, vel_liq_super_ms, rho_liq_kgm3, rho_gas_kgm3, mu_liq_cP, mu_gas_cP,
                            d_m, sigma_liq_Nm, theta, e, rough_pipe = 0):
    """
    Function for calculation pressure-gradient according Beggs&Brill correlation(1973)
    with Payne et al modification(1979)
    :param vel_gas_super_ms:
    :param vel_liq_super_ms:
    :param rho_liq_kgm3:
    :param rho_gas_kgm3:
    :param mu_liq_cP:
    :param mu_gas_cP:
    :param d_m:
    :param sigma_liq_Nm:
    :param theta:
    :param e:
    :param rough_pipe:
    :return:
    """
    n_fr = (vel_gas_super_ms + vel_liq_super_ms) ** 2 / (uc.g * d_m)
    n_lv = vel_liq_super_ms * (rho_liq_kgm3 / (uc.g * sigma_liq_Nm)) ** 0.25
    lyambda_l = vel_liq_super_ms / (vel_liq_super_ms + vel_gas_super_ms)
    vel_mix_ms = vel_gas_super_ms + vel_liq_super_ms
    rho_mix_kgm3 = rho_liq_kgm3 * lyambda_l + rho_gas_kgm3 * (1 - lyambda_l)
    mu_mix_cP = mu_liq_cP * lyambda_l + mu_gas_cP * (1 - lyambda_l)
    n_re = 1000 * rho_mix_kgm3 * vel_mix_ms * d_m / mu_mix_cP
    theta_rad = theta * np.pi / 180

    # Блок определения структуры потока
    l1 = 316 * lyambda_l ** 0.302
    l2 = 0.000925 * lyambda_l ** (-2.468)
    l3 = 0.1 * lyambda_l ** (-1.452)
    l4 = 0.5 * lyambda_l ** (-6.738)

    if (n_fr >= l1 and lyambda_l < 0.4) or (n_fr > l4 and lyambda_l >= 0.4):
        flow_pattern = 2
    elif (l1 >= n_fr > l3 and 0.4 > lyambda_l >= 0.01) or (lyambda_l >= 0.4 and l3 < n_fr <= l4):
        flow_pattern = 1
    elif l3 >= n_fr >= l2 and 0.4 > lyambda_l >= 0.01:
        flow_pattern = 3
    else:
        flow_pattern = 0

    # Блок определения объемного содержания жидкости
    if flow_pattern != 3 and theta >= 0:
        h_l = unf_liguid_holdup_BeggsBrill(lyambda_l, flow_pattern, n_lv, n_fr, theta)
    elif theta >= 0:
        a = (l3 - n_fr) / (l3 - l2)
        h_l = a * unf_liguid_holdup_BeggsBrill(lyambda_l, 0, n_lv, n_fr, theta) + (1 - a) * unf_liguid_holdup_BeggsBrill(
            lyambda_l, 1, n_lv, n_fr, theta)
    else:
        h_l = unf_liguid_holdup_BeggsBrill(lyambda_l, 3, n_lv, n_fr, theta)

    # Блок определения коэффициента трения
    f_n = unf_friction_factor_BeggsBrill(n_re, d_m, e, rough_pipe)
    y = max(lyambda_l / h_l ** 2, 0.001)  # нужно это уточнить у Рината Альфредовича
    if 1.2 > y > 1:
        s = np.log(2.2 * y - 1.2)
    else:
        s = np.log(y) / (-0.0523 + 3.182 * np.log(y) - 0.8725 * (np.log(y)) ** 2 + 0.01853 * (np.log(y)) ** 4)
    f = f_n * np.exp(s)
    rho_s = rho_liq_kgm3 * h_l + rho_gas_kgm3 * (1 - h_l)
    e_k = 0  # нужно уточнить по поводу этого почему не используется
    dpdl = (f * rho_mix_kgm3 * vel_mix_ms ** 2 / (2 * d_m) + rho_s * uc.g * np.sin(theta_rad)) / (1 - e_k)
    return dpdl

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


"""
def unf_flowpattern_prediction(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, f, vel_gas_super_ms, vel_liq_super_ms, d_m):

    Функция для определения структуры потока газожидкостной смеси

    :param sigma_liq_Nm:
    :param rho_liq_kgm3:
    :param rho_gas_kgm3:
    :param f:
    :param vel_gas_super_ms:
    :param vel_liq_super_ms:
    :param d_m:
    :return:

    d_m_min = 19.01 * (((rho_liq_kgm3 - rho_gas_kgm3) * sigma_liq_Nm)/(rho_liq_kgm3 ** 2 * uc.g)) ** 0.5
    # Проверка соотношения для эмульсионной структуры потока
    if a == b:
        structure = 'dispersed bubble'
    else:
        structure = 'not dispersed bubble'
    # Проверка соотношения для кольцевой структуры потока
    vel_gas_super_ann = 3.1 * ((uc.g * sigma_liq_Nm * (rho_liq_kgm3 - rho_gas_kgm3))/rho_gas_kgm3 ** 2) ** 0.25
    pass


def unf_dispersedbubbleflow_model(rho_liq_kgm3, rho_gas_kgm3, lyambda_l, vel_liq_super_ms, vel_gas_super_ms, mu_liq_cP,
                                  mu_gas_cP, d_m):

    Функция для расчета потерь давления для dispersed-bubble structure

    :param rho_liq_kgm3:
    :param rho_gas_kgm3:
    :param lyambda_l:
    :param vel_liq_super_ms:
    :param vel_gas_super_ms:
    :param mu_liq_cP:
    :param mu_gas_cP:
    :param d_m:
    :return:

    rho_tp = rho_liq_kgm3 * lyambda_l + rho_gas_kgm3 * (1 - lyambda_l)
    vel_tp = vel_liq_super_ms + vel_gas_super_ms
    mu_tp = mu_liq_cP * lyambda_l + mu_gas_cP * (1 - lyambda_l)

    n_re_tp = pt.reynolds_number(rho_tp, vel_tp, d_m, mu_tp)
    # TODO f = function(n_re_tp)
    f = 0.01

    dpdl_el = rho_tp * uc.g
    dpdl_f = f * rho_tp * vel_tp ** 2 / (2 * d_m)
    dpdl_sum = dpdl_el + dpdl_f
    return dpdl_sum


def unf_bubblyflow_model(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, vel_liq_super_ms, vel_gas_super_ms, mu_liq_cP,
                         mu_gas_cP, d_m):

    Функция для расчета потерь давления для bubbly structure

    :param sigma_liq_Nm:
    :param rho_liq_kgm3:
    :param rho_gas_kgm3:
    :param vel_liq_super_ms:
    :param vel_gas_super_ms:
    :param mu_liq_cP:
    :param mu_gas_cP:
    :param d_m:
    :return:


    h_l_0 = 0.25

    def equation2solve(h_l):
        equation = 1.53 * (uc.g * sigma_liq_Nm * (rho_liq_kgm3 - rho_gas_kgm3) / rho_liq_kgm3 ** 2) ** 0.25 * h_l **\
                   0.5 - vel_gas_super_ms / (1 - h_l) + 1.2 * (vel_gas_super_ms + vel_liq_super_ms)
        return equation
    h_l = opt.fsolve(equation2solve, np.array(h_l_0))[0]

    rho_tp = rho_liq_kgm3 * h_l + rho_gas_kgm3 * (1 - h_l)
    vel_tp = vel_liq_super_ms + vel_gas_super_ms
    mu_tp = mu_liq_cP * h_l + mu_gas_cP * (1 - h_l)

    n_re_tp = pt.reynolds_number(rho_tp, vel_tp, d_m, mu_tp)
    # TODO f = function(n_re_tp)
    f = 0.01

    dpdl_el = rho_tp * uc.g
    dpdl_f = f * rho_tp * vel_tp ** 2 / (2 * d_m)
    dpdl_sum = dpdl_el + dpdl_f
    return dpdl_sum


def unf_slugflow_model():
    vel_tb = 1.2 * (vel_gas_super_ms + vel_liq_super_ms) + 0.35 * (uc.g * d_m * (rho_liq_kgm3 - rho_gas_kgm3) /
                                                                   rho_liq_kgm3) ** 0.5
    vel_gls = 1.2 * (vel_gas_super_ms + vel_liq_super_ms) + 1.53 * (uc.g * sigma_liq_Nm * (rho_liq_kgm3 - rho_gas_kgm3) /
                                                                   rho_liq_kgm3 ** 2) ** 0.25 *
    pass


def unf_annularflow_model():
    pass


def unf_Ansarigradient():
    pass
"""