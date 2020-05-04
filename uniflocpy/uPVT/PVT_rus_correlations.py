import scipy.optimize as sp  # модуль для решения уравения
import numpy as np
import math

def coef_mt(t_k, rho_deadoil_kgm3, gamma_gas):
    return 1 + 0.029 * (t_k - 293) * (rho_deadoil_kgm3 * gamma_gas * 10 ** (-3) - 0.7966)


def coef_dt(t_k, rho_deadoil_kgm3, gamma_gas):
    return 10 ** (-3) * rho_deadoil_kgm3 * gamma_gas * (4.5 - 0.00305 * (t_k - 293)) - 4.785


def coef_rp(p_mpa, pb_mpa):
    return (1 + np.log10(p_mpa)) / (1 + np.log10(pb_mpa)) - 1


def unf_calc_gas_liberated_and_dissolved(t_k, rho_deadoil_kgm3, gamma_oil, gamma_gas, p_mpa, pb_mpa, rsb_m3t, return_m3m3=True):
    rp = coef_rp(p_mpa, pb_mpa)
    #print(f"rp={rp}")
    dt = coef_dt(t_k, rho_deadoil_kgm3, gamma_gas)
    #print(f"dt={dt}")
    mt = coef_mt(t_k, rho_deadoil_kgm3, gamma_gas)
    #print(f"mt={mt}")
    gas_liberated_m3t = rsb_m3t * rp * mt * (dt * (1 + rp) - 1)
    gas_dissolved_m3t = rsb_m3t * mt - gas_liberated_m3t

    if p_mpa >= pb_mpa:
        if return_m3m3:
            return 0, rsb_m3t * rho_deadoil_kgm3 / 1000 * mt
        else:
            return 0, rsb_m3t * mt

    if return_m3m3:
        return gas_liberated_m3t * rho_deadoil_kgm3 / 1000, gas_dissolved_m3t * rho_deadoil_kgm3 / 1000
        # return gas_liberated_m3t, gas_dissolved_m3t
    else:
        return gas_liberated_m3t, gas_dissolved_m3t


def calc_gas_for_fsolve(rsb_m3m3_real, t_k, rho_deadoil_kgm3, gamma_gas, p_mpa, pb_mpa, rsb_m3m3, return_m3m3=True):
    gas_dissolved_m3m3 = unf_calc_gas_liberated_and_dissolved(t_k, rho_deadoil_kgm3, gamma_gas, pb_mpa, pb_mpa, rsb_m3m3_real, return_m3m3=True)[1]
    return (gas_dissolved_m3m3 - rsb_m3m3) ** 2


def calc_gas_with_fsolve(t_k, rho_deadoil_kgm3, gamma_gas, p_mpa, pb_mpa, rsb_m3m3, return_m3m3=True):
    method = 'excitingmixing'
    answer = sp.root(calc_gas_for_fsolve, rsb_m3m3, args=(t_k, rho_deadoil_kgm3, gamma_gas,
                                                          pb_mpa, pb_mpa, rsb_m3m3, True), tol=0.01,
                     options={'xtol': 0.01, 'maxfev': 25, 'eps': 0.01})
    # print(answer)
    answer = answer.x[0]

    return unf_calc_gas_liberated_and_dissolved(t_k, rho_deadoil_kgm3, gamma_gas, p_mpa, pb_mpa, answer, return_m3m3)


t_k = 300.5
p_mpa = 5.5
p_res_mpa = 17.5
t_res_k = 313
gamma_oil = 0.868
rsb_m3t = 55.6
p_b_mpa = 9.2
gamma_gas = 1.119
y_a = 0.069
u_c1 = 0.355

def calc_pb(pb_mpa, rho_deadoil_kgm3, rsb_m3t, tres_k, t_k, ch4_d = 0.4, n2_d=0.08):
    #gi = rsb_m3m3 * 10**3 * 273 / 293 / rho_deadoil_kgm3
    pbt_mpa = pb_mpa - (tres_k - t_k) / (9.157 + 701.8 / (rsb_m3t * (ch4_d - 0.8* n2_d)))
    return pbt_mpa





def rho_gas_liberated_relative(p_mpa, pb_mpa, gamma_gas, gamma_oil, t_k, rsb_m3t):
    a = 1 + 0.0054 * (t_k - 293)

    u = gamma_oil * rsb_m3t - 186
    rp = coef_rp(p_mpa, pb_mpa)
    result = a * (gamma_gas - 0.0036 * (1 + rp) * (105.7 + u * rp))
    return result




def unf_rho_gas_dissolved_relative(rsb_m3t, gamma_oil, gamma_gas,
                                   gas_liberated_m3t, gas_dissolved_m3t, rho_gas_liberated_d,
                                   p_mpa, pb_mpa, t_k):
    a = 1 + 0.0054 * (t_k - 293)
    # print(a)
    mt = coef_mt(t_k, gamma_oil * 1000, gamma_gas)
    # print(mt)
    rp = coef_rp(p_mpa, pb_mpa)
    # print(rp)
    result = rsb_m3t * (a * mt * gamma_gas -
                        rho_gas_liberated_d * gas_liberated_m3t / rsb_m3t) / gas_dissolved_m3t
    # print(rsb_m3t,a,  mt,gamma_gas, rho_gas_liberated_d, gas_liberated_m3t,  rsb_m3t,gas_dissolved_m3t)
    return result





def unf_b_oil_below_pb_m3m3(t_k, p_mpa, gamma_oil, gamma_gas,

                            rho_gas_dissolved_relative_d, rho_gas_liberated_relative,

                            gas_dissolved_m3t):
    a = 1 + 0.0054 * (t_k - 293)
    # print(a)
    mt = coef_mt(t_k, gamma_oil * 1000, gamma_gas)
    # print(mt)

    lambda_t = 10 ** (-3) * (4.3 - 3.54 * gamma_oil + (1.0337 * rho_gas_dissolved_relative_d) / a + \
                             5.581 * 10 ** (-3) * gamma_oil * (1 - 1.61 * 10 ** (-3) * gas_dissolved_m3t) * \
                             gas_dissolved_m3t)
    # print(lambda_t)
    if 0.78 < gamma_oil <= 0.860:
        alpha_n = 10 ** (-3) * (3.083 - 2.638 * gamma_oil)
    else:
        alpha_n = 10 ** (-3) * (2.513 - 1.975 * gamma_oil)
        #print(alpha_n)

    result = 1 + 1.0733 * gamma_oil * gas_dissolved_m3t * lambda_t / mt + alpha_n * (t_k - 293) - \
             6.5 * 10 ** (-4) * p_mpa
    return result




def unf_rho_oil_kgm3(gamma_oil, rho_gas_dissolved_relative_d,
                     gas_dissolved_m3t, t_k, gamma_gas, b_oil_below_pb_m3m3):
    a = 1 + 0.0054 * (t_k - 293)
    # print(a)
    mt = coef_mt(t_k, gamma_oil * 1000, gamma_gas)
    # print(mt)
    result = gamma_oil * 1000 * (1 +
                                 1.293 * 10 ** (-3) * rho_gas_dissolved_relative_d * gas_dissolved_m3t / \
                                 (a * mt)) / b_oil_below_pb_m3m3
    return result



def unf_mu_dead_oil_293_cp(gamma_oil):
    rho_oil_stkgm3 = gamma_oil * 1000
    if 0.845 < gamma_oil < 0.924:
        return ( (0.658 * rho_oil_stkgm3**2) / (886 * 10**3 - rho_oil_stkgm3 **2) )**2
    else:
        return ( (0.456 * rho_oil_stkgm3**2) / (883 * 10**3 - rho_oil_stkgm3 **2) )**2



def unf_mu_dead_oil_t_cp(mu_dead_oil_293_cp, t_k, gamma_oil):
    a = 10 ** (-0.0175 * (293 - t_k) - 2.58)
    b = (8 * 10**(-5) * gamma_oil * 1000 - 0.047) * mu_dead_oil_293_cp ** (0.13 + 0.002 * (t_k - 293))
    for_exp = b*(293-t_k)
    separate_exp = 2.73**(for_exp)
    result = mu_dead_oil_293_cp * ((t_k - 293)**a) * separate_exp
    if type(result) == complex:
        result = result.real
    return result


def unf_mu_oil_below_pb_cp(mu_dead_oil_t_cp, gas_dissolved_m3t, gamma_oil):
    if 0.78 < gamma_oil <= 0.860:
        alpha_n = 10**(-3)*(3.083-2.638*gamma_oil)
    else:
        alpha_n = 10**(-3)*(2.513-1.975*gamma_oil)
    gas_dissolved_m3t_in_dif_cond = 1.055 * (1 + 5 * alpha_n) * gas_dissolved_m3t * gamma_oil
    a = 1 + 0.0129 * gas_dissolved_m3t_in_dif_cond - 0.0364 * gas_dissolved_m3t_in_dif_cond ** 0.85
    b = 1 + 0.0017 * gas_dissolved_m3t_in_dif_cond - 0.0228 * gas_dissolved_m3t_in_dif_cond ** 0.667
    mu_oil = a * mu_dead_oil_t_cp ** b
    return mu_oil



def unf_sigma_oil_gas_nm(p_mpa, t_k, gamma_gas):
    return (1/10) **(1.58 + 0.05*p_mpa) - 72 * 10**(-6)*(t_k-305)



def unf_rs_res_m3t(rsb_m3t, gamma_oil,gamma_gas, t_k):
    mt = coef_mt(t_k, gamma_oil*1000, gamma_gas)
    return rsb_m3t * mt


def unf_alpha_n(gamma_oil):
    if 0.78 < gamma_oil <= 0.860:
        alpha_n = 10 ** (-3) * (3.083 - 2.638 * gamma_oil)
    else:
        alpha_n = 10 ** (-3) * (2.513 - 1.975 * gamma_oil)
    return alpha_n

def unf_rho_gas_relative_in_oil_res_d(rsb_m3t, rs_res_m3t, t_k, gamma_gas):
    a = 1 + 0.0054 * (t_k - 293)
    mt = coef_mt(t_k, gamma_oil*1000, gamma_gas)
    return a * mt * gamma_gas * rsb_m3t / rs_res_m3t



def unf_b_oil_res_m3m3(gamma_oil, gamma_gas, p_mpa, rs_res_m3t,t_k,  rho_gas_relative_dissolved_d, rsb_m3t):
    a = 1 + 0.0054 * (t_k - 293)  # TODO check
    # print(a)
    alpha_n = unf_alpha_n(gamma_oil)
    # print(alpha_n)
    mt = coef_mt(t_k, gamma_oil * 1000, gamma_gas)
    lambda_coef = (10 ** (-3) * (4.3 - 3.54 * gamma_oil + 1.0337 * rho_gas_relative_dissolved_d / a +
                                 5.581 * 10 ** (-3) * gamma_oil *
                                 (1 - 1.61 * 10 ** (-3) * gamma_oil * rs_res_m3t) *
                                 (rs_res_m3t)
                                 )
                   )
    # print(lambda_coef)
    # print(rs_res_m3t)
    result = (1 +
              1.0733 * gamma_oil * lambda_coef * rs_res_m3t / mt +
              alpha_n * (t_k - 293) -
              6.5 * 10 ** (-4) * p_mpa)
    return result





def unf_rho_oil_above_pb_m3m3(rs_res_m3t, gamma_gas, gamma_oil,
                          t_k, rho_gas_relative_dissolved_d, b_oil_res_m3m3):
    a = 1 + 0.0054 * (t_k - 293)
    mt = coef_mt(t_k, gamma_oil*1000, gamma_gas)


    result = (gamma_oil * 1000 *
             (1 + 1.293 * 10**(-3) * rho_gas_relative_dissolved_d * rs_res_m3t / (a * mt))) / b_oil_res_m3m3
    return result


def unf_mu_oil_above_pb_cp(gamma_oil, rsb_m3t, t_k, p_mpa, p_b_mpa):
    mu_dead_oil_293_cp = unf_mu_dead_oil_293_cp(gamma_oil)
    #print(mu_dead_oil_293_cp)

    mu_dead_oil_t_cp = unf_mu_dead_oil_t_cp(mu_dead_oil_293_cp, t_k, gamma_oil)
    # print(mu_dead_oil_t_cp)

    alpha_n = unf_alpha_n(gamma_oil)
    g_with_star = 1.055 * (1 + 5 * alpha_n) * rsb_m3t * gamma_oil
    # print(g_with_star)
    a = 1 + 0.0129 * g_with_star - 0.0364 * g_with_star ** 0.85
    # print(a)
    b = 1 + 0.0017 * g_with_star - 0.0228 * g_with_star ** 0.667
    # print(b)

    mu_oil_pb_cp = a * mu_dead_oil_t_cp ** b
    # print(mu_oil_pb_cp)

    if mu_oil_pb_cp < 5:
        delta = 0.0114 * mu_oil_pb_cp
    elif 5 <= mu_oil_pb_cp < 10:
        delta = 0.057 + 0.023 * (mu_oil_pb_cp - 5)
    elif 10 <= mu_oil_pb_cp < 25:
        delta = 0.0171 + 0.031 * (mu_oil_pb_cp - 10)
    elif 25 <= mu_oil_pb_cp < 45:
        delta = 0.643 + 0.045 * (mu_oil_pb_cp - 25)
    elif 45 <= mu_oil_pb_cp < 75:
        delta = 1.539 + 0.058 * (mu_oil_pb_cp - 45)
    # elif 75<=mu_oil_pb_cp < 85:
    else:
        delta = 3.286 + 0.100 * (mu_oil_pb_cp - 75)
    # print(delta)
    result = mu_oil_pb_cp * delta * (p_mpa - p_b_mpa)
    # print(mu_oil_pb_cp, delta,p_mpa, p_b_mpa)

    return result

if __name__ == "__main__":

    p_b_t_mpa = calc_pb(p_b_mpa, gamma_oil * 1000, rsb_m3t, t_res_k, t_k,u_c1 , y_a)
    print(f"p_b_t_mpa = {p_b_t_mpa}")


    gas_liberated_m3t, gas_dissolved_m3t = unf_calc_gas_liberated_and_dissolved(t_k, gamma_oil * 1000, gamma_oil, gamma_gas,
                                                                                p_mpa, p_b_t_mpa, rsb_m3t, False)
    print(f"gas_liberated_m3t = {gas_liberated_m3t}")
    print(f"gas_dissolved_m3t = {gas_dissolved_m3t}")

    rho_gas_liberated_d = rho_gas_liberated_relative(p_mpa, p_b_t_mpa, gamma_gas,
                                                     gamma_oil, t_k , rsb_m3t)

    print(f"rho_gas_liberated_d = {rho_gas_liberated_d}")


    rho_gas_dissolved_relative_d = unf_rho_gas_dissolved_relative(rsb_m3t, gamma_oil, gamma_gas,
                                                                  gas_liberated_m3t, gas_dissolved_m3t,
                                                                  rho_gas_liberated_d,
                                                                  p_mpa, p_b_t_mpa, t_k)


    print(f"rho_gas_dissolved_relative_d = {rho_gas_dissolved_relative_d}")


    b_oil_m3m3 = unf_b_oil_below_pb_m3m3(t_k, p_mpa, gamma_oil, gamma_gas, rho_gas_dissolved_relative_d, rho_gas_liberated_d,
                                         gas_dissolved_m3t)

    print(f"b_oil_m3m3 = {b_oil_m3m3}")

    rho_oil_kgm3 = unf_rho_oil_kgm3(gamma_oil, rho_gas_dissolved_relative_d, gas_dissolved_m3t,
                                   t_k, gamma_gas, b_oil_m3m3)

    print(f"rho_oil_kgm3 = {rho_oil_kgm3}")


    mu_dead_oil_293_cp = unf_mu_dead_oil_293_cp(gamma_oil)
    print(f"mu_dead_oil_293_cp = {mu_dead_oil_293_cp}")


    mu_dead_oil_t_cp = unf_mu_dead_oil_t_cp(mu_dead_oil_293_cp, t_k, gamma_oil)
    print(f"mu_dead_oil_t_cp = {mu_dead_oil_t_cp}")

    mu_oil_below_pb_cp = unf_mu_oil_below_pb_cp(mu_dead_oil_t_cp, gas_dissolved_m3t, gamma_oil)
    print(f"mu_oil_below_pb_cp = {mu_oil_below_pb_cp}")


    sigma_oil_gas_nm = unf_sigma_oil_gas_nm(p_mpa, t_k, gamma_gas)

    print(f"sigma_oil_gas_nm = {sigma_oil_gas_nm}")




    print('Расчет нефти в пластовых условиях')

    t_k = 313
    p_mpa = 17.5
    rs_res_m3t = unf_rs_res_m3t(rsb_m3t, gamma_oil,gamma_gas, t_k)

    print(f"rs_res_m3t = {rs_res_m3t}")


    rho_gas_relative_in_oil_res_d = unf_rho_gas_relative_in_oil_res_d(rsb_m3t, rs_res_m3t, t_k, gamma_gas)
    print(f"rho_gas_relative_in_oil_res_d = {rho_gas_relative_in_oil_res_d}")



    b_oil_res_m3m3 = unf_b_oil_res_m3m3(gamma_oil,gamma_gas,  p_mpa,rs_res_m3t,t_k, rho_gas_relative_in_oil_res_d, rsb_m3t)
    print(f"b_oil_res_m3m3 = {b_oil_res_m3m3}")

    #b_oil_res_m3m3=1.146


    rho_oil_above_pb_m3m3 = unf_rho_oil_above_pb_m3m3(rs_res_m3t, gamma_gas, gamma_oil,
                              t_k, rho_gas_relative_in_oil_res_d, b_oil_res_m3m3)


    print(f"rho_oil_above_pb_m3m3 = {rho_oil_above_pb_m3m3}")

    mu_oil_above_pb_cp = unf_mu_oil_above_pb_cp(gamma_oil, rsb_m3t, t_k, p_mpa, p_b_mpa)

    print(f"mu_oil_above_pb_cp = {mu_oil_above_pb_cp}")







