import uniflocpy.uPVT.PVT_correlations as PVT
import numpy as np
import scipy.optimize as sp  # модуль для решения уравения
import uniflocpy.uTools.uconst as uconst


def calc_z_in_annulus(gamma_gas, t_c, p_bar):
    gamma_gas = float(gamma_gas)
    t_c = float(t_c)
    p_bar = float(p_bar)
    t_k = uconst.c2k(t_c)
    p_mpa = uconst.bar2MPa(p_bar)
    tpc_k = PVT.unf_pseudocritical_temperature_Standing_K(gamma_gas)
    ppc_mpa = PVT.unf_pseudocritical_pressure_Standing_MPa(gamma_gas)
    #z = PVT.unf_z_factor_Kareem(t_k/tpc_k, p_mpa/ppc_mpa)
    z = PVT.unf_zfactor_DAK_ppr(p_mpa / ppc_mpa, t_k / tpc_k)
    return z


def calc_t_in_annulus(h_m, t_wellhead_c, t_bottomhole_c, h_bottomhole_m):
    return t_wellhead_c + h_m * (t_bottomhole_c- t_wellhead_c)/h_bottomhole_m


def calc_p_in_annulus_for_scipy(p_bar, h_m, p_wellhead_bar, t_wellhead_c, gamma_gas, t_bottomhole_c, h_bottomhole_m):
    p_mean_bar = (p_bar + p_wellhead_bar) / 2
    t_mean_c = (t_wellhead_c + calc_t_in_annulus(h_m, t_wellhead_c, t_bottomhole_c, h_bottomhole_m)) / 2

    t_mean_k = (uconst.c2k(t_wellhead_c) + uconst.c2k(
        calc_t_in_annulus(h_m, t_wellhead_c, t_bottomhole_c, h_bottomhole_m))) / 2

    z_mean = calc_z_in_annulus(gamma_gas, t_mean_c, p_mean_bar)
    # print(p_mean_bar, t_mean_c, z_mean)
    p_wellhead_mpa = uconst.bar2MPa(p_wellhead_bar)
    # print(z_mean)
    power = (0.03415 * gamma_gas * h_m) / (z_mean * t_mean_k)
    # print(power)
    p_new_mpa = p_wellhead_mpa * np.exp(power)
    p_new_bar = p_new_mpa * 10
    return (p_new_bar - p_bar) ** 2


def calc_p_in_annulus_bar(h_m, p_wellhead_bar, t_wellhead_c, gamma_gas,t_bottomhole_c, h_bottomhole_m, dranchuk_using=False):
    method = 'excitingmixing'
    answer = sp.root(calc_p_in_annulus_for_scipy, p_wellhead_bar,
                     args=(h_m, p_wellhead_bar, t_wellhead_c, gamma_gas,t_bottomhole_c, h_bottomhole_m), tol=0.01,
                     #options={'xtol': 0.01,
                      #        #'maxfev': 50,
                      #        'eps': 0.01}
                             )

    #print(answer)
    answer = answer.x[0]
    return answer