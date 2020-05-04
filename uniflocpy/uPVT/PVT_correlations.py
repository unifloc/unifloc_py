# -*- coding: utf-8 -*-
"""
Created on Sat May  5 13:59:06 2018

@author: Rinat Khabibullin
         Alexey Vodopyan

Модуль содержит корреляции, необходимые для вычисления PVT свойств нефти, газа и воды.
"""
import numpy as np
import unittest
import uniflocpy.uTools.uconst as uc
import scipy.optimize as opt

# uPVT свойства для нефти


def unf_pb_Standing_MPaa(rsb_m3m3, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
        bubble point pressure calculation according to Standing (1947) correlation

    :param rsb_m3m3: solution ration at bubble point, must be given, m3/m3
    :param gamma_oil: specific oil density (by water)
    :param gamma_gas: specific gas density (by air)
    :param t_K: temperature, K
    :return: bubble point pressure abs in MPa

    ref1 "A Pressure-Volume-Temperature Correlation for Mixtures of California Oil and Gases",
    M.B. Standing, Drill. & Prod. Prac., API, 1947.

    ref2  "Стандарт компании Юкос. Физические свойства нефти. Методы расчета." Афанасьев В.Ю., Хасанов М.М. и др. 2002 г
    """

    min_rsb = 1.8
    rsb_old = rsb_m3m3
    if rsb_m3m3 < min_rsb:
        rsb_m3m3 = min_rsb
    # мольная доля газа
    yg = 1.225 + 0.001648 * t_K - 1.769 / gamma_oil
    pb_MPaa = 0.5197 * (rsb_m3m3 / gamma_gas) ** 0.83 * 10 ** yg
    # для низких значений газосодержания зададим асимптотику Pb = 1 атма при Rsb = 0
    # для больших значений газосодержания не корректируем то, что дает корреляция
    if rsb_old < min_rsb:
        pb_MPaa = (pb_MPaa - 0.1013) * rsb_old / min_rsb + 0.1013  # 0.101325
    return pb_MPaa


def unf_pb_Valko_MPaa(rsb_m3m3, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
        bubble point pressure calculation according to Valko McCain (2002) correlation

    :param rsb_m3m3: solution ration at bubble point, must be given, m3/m3
    :param gamma_oil: specific oil density (by water)
    :param gamma_gas: specific gas density (by air)
    :param t_K: temperature, K
    :return: bubble point pressure abs in MPa

    ref SPE  "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities"
    W. D. McCain Jr.,P.P. Valko,
    """

    min_rsb = 1.8
    max_rsb = 800
    rsb_old = rsb_m3m3
    if rsb_m3m3 < min_rsb:
        rsb_m3m3 = min_rsb
    if rsb_m3m3 > max_rsb:
        rsb_m3m3 = max_rsb

    z1 = -4.81413889469569 + 0.748104504934282 * np.log(rsb_m3m3) \
        + 0.174372295950536 * np.log(rsb_m3m3) ** 2 - 0.0206 * np.log(rsb_m3m3) ** 3
    z2 = 25.537681965 - 57.519938195 / gamma_oil + 46.327882495 / gamma_oil ** 2 - 13.485786265 / gamma_oil ** 3
    z3 = 4.51 - 10.84 * gamma_gas + 8.39 * gamma_gas ** 2 - 2.34 * gamma_gas ** 3
    z4 = 6.00696e-8 * t_K ** 3 - 8.554832172e-5 * t_K ** 2 + 0.043155018225018 * t_K - 7.22546617091445
    z = z1 + z2 + z3 + z4

    pb_atma = 119.992765886175 * np.exp(0.0075 * z ** 2 + 0.713 * z)
    pb_MPaa = pb_atma / 10.1325
    """
    для низких значений газосодержания зададим асимптотику Pb = 1 атма при Rsb = 0
    корреляция Valko получена с использованием непараметрической регресии GRACE метод (SPE 35412)
    особенность подхода - за пределеми интервала адаптации ассимптотики не соблюдаются
    поэтому их устанавливаем вручную
    для больших значений газосодержания продолжим линейный тренд корреляции
    """
    if rsb_old < min_rsb:
        pb_MPaa = (pb_MPaa - 0.1013) * rsb_old / min_rsb + 0.1013
    if rsb_old > max_rsb:
        pb_MPaa = (pb_MPaa - 0.1013) * rsb_old / max_rsb + 0.1013

    return pb_MPaa

"""
def unf_pb_AlMarhoun_MPaa(rsb_m3m3, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    
    pass

        bubble point pressure calculation according to Al-Marhoun (1985) correlation

    Middle-East oil

    :param rsb_m3m3: solution ration at bubble point, must be given, m3/m3
    :param gamma_oil: specific oil density (by water)
    :param gamma_gas: specific gas density (by air)
    :param t_K: temperature, K
    :return: bubble point pressure abs in MPa

        ref1 PYT Correlations for Middle East Crude Oils, Muhammad All AI.Marhoun, Journal of Petroleum Technology. May 1988

    

    pass
"""


def unf_rs_Standing_m3m3(p_MPaa, pb_MPaa=0, rsb_m3m3=0, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
        Gas-oil ratio calculation inverse of Standing (1947) correlation for bubble point pressure

    :param p_MPaa: pressure, MPa
    :param pb_MPaa: buble point pressure, MPa
    :param rsb_m3m3: gas-oil ratio at the bubble point pressure, m3/m3
    :param gamma_oil: specific oil density (by water)
    :param gamma_gas: specific gas density (by air)
    :param t_K: temperature, K
    :return: gas-oil ratio in m3/m3

    ref1 "A Pressure-Volume-Temperature Correlation for Mixtures of California Oil and Gases",
    M.B. Standing, Drill. & Prod. Prac., API, 1947.

    ref2  "Стандарт компании Юкос. Физические свойства нефти. Методы расчета." Афанасьев В.Ю., Хасанов М.М. и др. 2002 г
    может считать в случае если нет давления насыщения и газосодержания при давлении насыщения, корреляция не точная
    """

    if pb_MPaa == 0 or rsb_m3m3 == 0:
        # мольная доля газа
        yg = 1.225 + 0.001648 * t_K - 1.769 / gamma_oil
        rs_m3m3 = gamma_gas * (1.92 * p_MPaa / 10 ** yg) ** 1.204
    elif p_MPaa < pb_MPaa:
        rs_m3m3 = rsb_m3m3 * (p_MPaa / pb_MPaa) ** 1.204
    else:
        rs_m3m3 = rsb_m3m3
    return rs_m3m3


def unf_rs_Velarde_m3m3(p_MPaa, pb_MPaa=10, rsb_m3m3=100, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
        Solution Gas-oil ratio calculation according to Velarde McCain (1999) correlation

    :param p_MPaa: pressure, MPa
    :param pb_MPaa: buble point pressure, MPa
    :param rsb_m3m3: gas-oil ratio at the bubble point pressure, m3/m3
    :param gamma_oil: specific oil density (by water)
    :param gamma_gas: specific gas density (by air)
    :param t_K: temperature, K
    :return: gas-oil ratio in m3/m3

        ref1 "Correlation of Black Oil Properties at Pressures Below Bubblepoint Pressure—A New Approach",
    J. VELARDE, T.A. BLASINGAME Texas A&M University, W.D. MCCAIN, JR. S.A. Holditch & Associates, Inc 1999

    """

    api = uc.gamma_oil2api(gamma_oil)
    t_F = uc.k2f(t_K)
    pb_psia = uc.MPa2psi(pb_MPaa)
    if pb_psia > 14.7:
        # TODO тут похоже идет перевод из абсолютных давлений в избыточные - может вынести это в отдельные фукнции?
        pr = (uc.MPa2psi(p_MPaa) - 14.7) / (pb_psia - 14.7)
    else:
        pr = 0
    if pr <= 0:
        rs_m3m3 = 0
    elif pr < 1:
        A0 = 9.73 * 10 ** (-7)
        A1 = 1.672608
        A2 = 0.929870
        A3 = 0.247235
        A4 = 1.056052
        B0 = 0.022339
        B1 = -1.004750
        B2 = 0.337711
        B3 = 0.132795
        B4 = 0.302065
        C0 = 0.725167
        C1 = -1.485480
        C2 = -0.164741
        C3 = -0.091330
        C4 = 0.047094
        a1 = A0 * gamma_gas ** A1 * api ** A2 * t_F ** A3 * (pb_psia - 14.7) ** A4
        a2 = B0 * gamma_gas ** B1 * api ** B2 * t_F ** B3 * (pb_psia - 14.7) ** B4
        a3 = C0 * gamma_gas ** C1 * api ** C2 * t_F ** C3 * (pb_psia - 14.7) ** C4
        pr = (uc.Pa2psi(p_MPaa * 10 ** 6) - 14.7) / (pb_psia - 14.7)
        rsr = a1 * pr ** a2 + (1 - a1) * pr ** a3
        rs_m3m3 = rsr * rsb_m3m3
    else:
        rs_m3m3 = rsb_m3m3
    return rs_m3m3


def unf_rsb_Mccain_m3m3(rsp_m3m3, gamma_oil, psp_MPaa=0, tsp_K=0):
    """
        Solution Gas-oil ratio at bubble point pressure calculation according to McCain (2002) correlation
    taking into account the gas losses at separator and stock tank

    :param rsp_m3m3: separator producing gas-oil ratio, m3m3
    :param gamma_oil: specific oil density(by water)
    :param psp_MPaa: pressure in separator, MPaa
    :param tsp_K: temperature in separator, K
    :return: solution gas-oil ratio at bubble point pressure, rsb in m3/m3

    часто условия в сепараторе неизвестны, может считать и без них по приблизительной формуле

    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002
    """

    rsp_scfstb = uc.m3m3_2_scfstb(rsp_m3m3)
    if psp_MPaa > 0 and tsp_K > 0:
        api = uc.gamma_oil2api(gamma_oil)
        psp_psia = uc.Pa2psi(psp_MPaa * 10 ** 6)
        tsp_F = uc.k2f(tsp_K)
        z1 = -8.005 + 2.7 * np.log(psp_psia) - 0.161 * np.log(psp_psia) ** 2
        z2 = 1.224 - 0.5 * np.log(tsp_F)
        z3 = -1.587 + 0.0441 * np.log(api) - 2.29 * 10 ** (-5) * np.log(api) ** 2
        z = z1 + z2 + z3
        rst_scfstb = np.exp(3.955 + 0.83 * z - 0.024 * z ** 2 + 0.075 * z ** 3)
        rsb = rsp_scfstb + rst_scfstb
    elif rsp_m3m3 >= 0:
        rsb = 1.1618 * rsp_scfstb
    else:
        rsb = 0
    rsb = uc.scfstb2m3m3(rsb)
    return rsb


def unf_gamma_gas_Mccain(rsp_m3m3, rst_m3m3, gamma_gassp, gamma_oil, psp_MPaa=0, tsp_K=0):
    """
        Correlation for weighted-average specific gravities of surface gases

    :param rsp_m3m3: separator producing gas-oil ratio, m3m3
    :param rst_m3m3: stock-tank producing gas-oil ratio, m3m3
    :param gamma_gassp:  separator gas specific gravity
    :param gamma_oil: specific oil density(by water)
    :param psp_MPaa: pressure in separator, MPaa
    :param tsp_K: temperature in separator, K
    :return: weighted-average specific gravities of surface gases

    часто условия в сепараторе неизвестны, может считать и без них по приблизительной формуле

    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002

    """

    if psp_MPaa > 0 and rsp_m3m3 > 0 and rst_m3m3 > 0 and gamma_gassp > 0:
        api = uc.gamma_oil2api(gamma_oil)
        psp_psia = uc.Pa2psi(psp_MPaa * 10 ** 6)
        tsp_F = uc.k2f(tsp_K)
        rsp_scfstb = uc.m3m3_2_scfstb(rsp_m3m3)
        rst_scfstb = uc.m3m3_2_scfstb(rst_m3m3)
        z1 = -17.275 + 7.9597 * np.log(psp_psia) - 1.1013 * np.log(psp_psia) ** 2 + 2.7735 * 10 ** (-2) \
            * np.log(psp_psia) ** 3 + 3.2287 * 10 ** (-3) * np.log(psp_psia) ** 4
        z2 = -0.3354 - 0.3346 * np.log(rsp_scfstb) + 0.1956 * np.log(rsp_scfstb) ** 2 - 3.4374 * 10 ** (-2) \
            * np.log(rsp_scfstb) ** 3 + 2.08 * 10 ** (-3) * np.log(rsp_scfstb) ** 4
        z3 = 3.705 - 0.4273 * api + 1.818 * 10 ** (-2) * api ** 2 - 3.459 * 10 ** (-4) \
            * api ** 3 + 2.505 * 10 ** (-6) * api ** 4
        z4 = -155.52 + 629.61 * gamma_gassp - 957.38 * gamma_gassp ** 2 + 647.57 \
            * gamma_gassp ** 3 - 163.26 * gamma_gassp ** 4
        z5 = 2.085 - 7.097 * 10 ** (-2) * tsp_F + 9.859 * 10 ** (-4) * tsp_F ** 2 \
            - 6.312 * 10 ** (-6) * tsp_F ** 3 + 1.4 * 10 ** (-8) * tsp_F ** 4
        z = z1 + z2 + z3 + z4 + z5
        # Stock-tank gas specific gravity
        gamma_gasst = 1.219 + 0.198 * z + 0.0845 * z ** 2 + 0.03 * z ** 3 + 0.003 * z ** 4
        gamma_gas = (gamma_gassp * rsp_scfstb + gamma_gasst * rst_scfstb) / (rsp_scfstb + rst_scfstb)
    elif gamma_gassp >= 0:
        gamma_gas = 1.066 * gamma_gassp
    else:
        gamma_gas = 0
    return gamma_gas


def unf_fvf_Mccain_m3m3_below(density_oilsto_kgm3, rs_m3m3, density_oil_kgm3, gamma_gas):
    """
        Oil Formation Volume Factor according McCain correlation for pressure below bubble point pressure

    :param density_oilsto_kgm3: density of stock-tank oil, kgm3
    :param rs_m3m3: solution gas-oil ratio, m3m3
    :param density_oil_kgm3: Oil density at reservoir conditions, kgm3
    :param gamma_gas: specific gas  density(by air)
    :return: formation volume factor bo, m3m3

    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    """

    density_oilsto_lbft3 = uc.kgm3_2_lbft3(density_oilsto_kgm3)
    density_oil_lbft3 = uc.kgm3_2_lbft3(density_oil_kgm3)
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    bo = (density_oilsto_lbft3 + 0.01357 * rs_scfstb * gamma_gas) / density_oil_lbft3
    return bo


def unf_fvf_VB_m3m3_above(bob, cofb_1MPa, pb_MPaa, p_MPaa):
    """
        Oil Formation Volume Factor according equation for pressure above bubble point pressure

    :param bob: formation volume factor at bubble point pressure, m3m3
    :param cofb_1MPa: weighted-average oil compressibility from bubblepoint pressure to a higher pressure of interest,1/MPa
    :param pb_MPaa: bubble point pressure, MPa
    :param p_MPaa: pressure, MPa
    :return: formation volume factor bo,m3m3

    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    ! Actually, this correlation is belonged ro Vasquez & Beggs (1980). In some sources is
    noted that this is Standing correlation.

    ref2 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA

    """

    if p_MPaa <= pb_MPaa:
        bo = bob
    else:
        bo = bob * np.exp(cofb_1MPa * (pb_MPaa - p_MPaa))
    return bo


def unf_compressibility_saturated_oil_McCain_1Mpa(p_mpa, pb_mpa, t_k, gamma_oil, rsb_m3m3):
    """
        Oil compressibility below bubble point (saturated oil)

    :param p_mpa:
    :param pb_mpa:
    :param t_k:
    :param gamma_oil:
    :param rsb_m3m3:
    :return:

    ref1 https://www.researchgate.net/publication/
    254529353_The_Oil_Compressibility_Below_Bubble_Point_Pressure_Revisited_-_Formulations_and_Estimations

    ref2 https://www.onepetro.org/download/journal-paper/SPE-15664-PA?id=journal-paper%2FSPE-15664-PA
    """
    rsb_scfstb = uc.m3m3_2_scfstb(rsb_m3m3)
    t_F = uc.k2r(t_k)
    api = uc.gamma_oil2api(gamma_oil)
    p_psia = uc.Pa2psi(p_mpa * 10 ** 6)
    pb_psia = uc.Pa2psi(pb_mpa * 10 ** 6)
    co_1psi = np.exp(-7.573 - 1.450 * np.log(p_psia) - 0.383 * np.log(pb_psia) + 1.402 * np.log(t_F) +
                     0.256 * np.log(api) + 0.449 * np.log(rsb_scfstb))
    co_1MPa = uc.compr_1psi_2_1pa(co_1psi) * 10 ** 5 # TODO нужно исправить
    return co_1MPa


def unf_compressibility_oil_VB_1Mpa(rs_m3m3, t_K, gamma_oil, p_MPaa, gamma_gas=0.6): # TODO above bubble point!!
    """
        Oil compressibility according to Vasquez & Beggs (1980) correlation

    :param rs_m3m3: solution gas-oil ratio, m3m3
    :param t_K: temperature, K
    :param gamma_oil: specific oil density (by water)
    :param p_MPaa: pressure, MPaa
    :param gamma_gas: specific gas density (by air)
    :return: coefficient of isothermal compressibility co_1MPa, 1/MPa

    ref1 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA

    """

    if p_MPaa > 0:
        rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
        t_F = uc.k2f(t_K)
        api = uc.gamma_oil2api(gamma_oil)
        p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
        co_1MPa = uc.compr_1psi_2_1pa(10 ** 6 * (-1433 + 5 * rs_scfstb + 17.2 * t_F - 1180 * gamma_gas + 12.61 *
                                                 api) / (10 ** 5 * p_psia))
    else:
        co_1MPa = 0
    return co_1MPa


def unf_fvf_Standing_m3m3_saturated(rs_m3m3, gamma_gas, gamma_oil, t_K):
    """
        Oil Formation Volume Factor according Standing equation at bubble point pressure

    :param rs_m3m3: solution gas-oil ratio, m3m3
    :param gamma_gas: specific gas density (by air)
    :param gamma_oil: specific oil density (by water)
    :param t_K: temperature, K
    :return: formation volume factor at bubble point pressure bo,m3m3

    ref1 Volumetric and phase behavior of oil field hydrocarbon systems / M.B. Standing Standing, M. B. 1981
    """

    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    t_F = uc.k2f(t_K)
    bo = 0.972 + 1.47 * 10 ** (-4) * (rs_scfstb * (gamma_gas / gamma_oil) ** 0.5 + 1.25 * t_F) ** 1.175
    return bo


def unf_density_oil_Mccain(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, gamma_gas, t_K, gamma_oil, gamma_gassp = 0):
    """
        Oil density according Standing, M.B., 1977; Witte, T.W., Jr., 1987; and McCain, W.D., Jr. and Hill, N.C.,
    1995 correlation.

    :param p_MPaa: pressure, MPaa
    :param pb_MPaa: bubble point pressure, MPaa
    :param co_1MPa: coefficient of isothermal compressibility, 1/MPa
    :param rs_m3m3: solution gas-oil ratio, m3m3
    :param gamma_gas: specific gas density (by air)
    :param t_K: temperature, K
    :param gamma_oil: specific oil density (by water)
    :param gamma_gassp: specific gas density in separator(by air)
    :return: oil density,kg/m3

    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    """

    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    ro_po = 52.8 - 0.01 * rs_scfstb  # первое приближение
    if gamma_gassp == 0:  # если нет данных о плотности газа в сепараторе
        gamma_gassp = gamma_gas
    epsilon = 0.000001
    maxiter = 100  # максимальное число иттераций
    counter = 0
    ro_po_current = ro_po
    ro_po_previous = 0
    # TODO в идеале нужно также как для z-фактора сделать через scipy а не через цикл
    while abs(ro_po_current - ro_po_previous) > epsilon and counter < maxiter:
        ro_po_previous = ro_po_current
        ro_a = -49.8930 + 85.0149 * gamma_gassp - 3.70373 * gamma_gassp * ro_po +\
            0.0479818 * gamma_gassp * ro_po ** 2 + 2.98914 * ro_po - 0.0356888 * ro_po ** 2
        ro_po = (rs_scfstb * gamma_gas + 4600 * gamma_oil) / (73.71 + rs_scfstb * gamma_gas / ro_a)
        ro_po_current = ro_po
        counter = counter + 1
    p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
    t_F = uc.k2f(t_K)
    if p_MPaa < pb_MPaa:
        dro_p = (0.167 + 16.181 * 10 ** (-0.0425 * ro_po)) * (p_psia / 1000) - 0.01 * (
                    0.299 + 263 * 10 ** (-0.0603 * ro_po)) * (p_psia / 1000) ** 2
        ro_bs = ro_po + dro_p
        dro_t = (0.00302 + 1.505 * ro_bs ** (-0.951)) * (t_F - 60) ** 0.938 - (0.0216 - 0.0233 * 10 ** (-0.0161 * ro_bs)
                                                                               ) * (t_F - 60) ** 0.475
        ro_or = ro_bs - dro_t
    else:
        pb_psia = uc.Pa2psi(pb_MPaa * 10 ** 6)
        dro_p = (0.167 + 16.181 * 10 ** (-0.0425 * ro_po)) * (pb_psia / 1000) - 0.01 * (
                    0.299 + 263 * 10 ** (-0.0603 * ro_po)) * (pb_psia / 1000) ** 2
        ro_bs = ro_po + dro_p
        dro_t = (0.00302 + 1.505 * ro_bs ** (-0.951)) * (t_F - 60) ** 0.938 - (
                    0.0216 - 0.0233 * 10 ** (-0.0161 * ro_bs)) * (t_F - 60) ** 0.475
        ro_orb = ro_bs - dro_t
        co_1psi = uc.compr_1pa_2_1psi(co_1MPa / 10 ** 6)
        ro_or = ro_orb * np.exp(co_1psi * (p_psia - pb_psia))
    ro_or = uc.lbft3_2_kgm3(ro_or)
    return ro_or


def unf_density_oil_Standing(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, bo_m3m3, gamma_gas, gamma_oil):
    """
        Oil density according Standing correlation.

    :param p_MPaa: pressure, MPaa
    :param pb_MPaa: bubble point pressure, MPaa
    :param co_1MPa: coefficient of isothermal compressibility, 1/MPa
    :param rs_m3m3: solution gas-oil ratio, m3m3
    :param bo_m3m3: oil formation volume factor, m3m3
    :param gamma_gas: specific gas density (by air)
    :param gamma_oil: specific oil density (by water)
    :return: oil density,kg/m3

    ref1 book Brill 2006, Production Optimization Using Nodal Analysis
    """
    po = (1000 * gamma_oil + 1.224 * gamma_gas * rs_m3m3) / bo_m3m3
    if p_MPaa > pb_MPaa:  # TODO возможно это надо оставить, при давлении выше P нас, уже есть в fvf_oil степень с co
        po = po * np.exp(co_1MPa * (p_MPaa - pb_MPaa))
    return po


def unf_deadoilviscosity_Beggs_cP(gamma_oil, t_K):
    """
        Correlation for dead oil viscosity

    :param gamma_oil: specific oil density (by water)
    :param t_K: temperature, K
    :return: dead oil viscosity,cP

    ref1 Beggs, H.D. and Robinson, J.R. “Estimating the Viscosity of Crude Oil Systems.”
    Journal of Petroleum Technology. Vol. 27, No. 9 (1975)

    """
    api = uc.gamma_oil2api(gamma_oil)
    t_F = uc.k2f(t_K)
    c = 10 ** (3.0324 - 0.02023 * api) * t_F ** (-1.163)
    viscosity_cP = 10 ** c - 1
    return viscosity_cP


def unf_deadoilviscosity_BeggsRobinson_VBA_cP(gamma_oil, t_K):
    x = (1.8 * t_K - 460) ** (-1.163) * np.exp(13.108 - 6.591 / gamma_oil)
    return 10 ** x - 1


def unf_deadoilviscosity_Standing(gamma_oil, t_k):
    return (0.32 + 1.8 * 10 ** 7 / (141.5 / gamma_oil - 131.5) ** 4.53) * (360 / (1.8 * t_k - 260)) ** (10 ** (0.43 + 8.33 / (141.5 / gamma_oil - 131.5)))


def unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, rs_m3m3):
    """
        Correlation for oil viscosity for pressure below bubble point

    :param deadoilviscosity_cP: dead oil viscosity,cP
    :param rs_m3m3: solution gas-oil ratio, m3m3
    :return: oil viscosity,cP

    ref1 Beggs, H.D. and Robinson, J.R. “Estimating the Viscosity of Crude Oil Systems.”
    Journal of Petroleum Technology. Vol. 27, No. 9 (1975)

    """
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    a = 10.715 * (rs_scfstb + 100) ** (-0.515)
    b = 5.44 * (rs_scfstb + 150) ** (-0.338)
    viscosity_cP = a * deadoilviscosity_cP ** b
    return viscosity_cP


def unf_undersaturatedoilviscosity_VB_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP):
    """
        Viscosity correlation for pressure above bubble point

    :param p_MPaa: pressure, MPaa
    :param pb_MPaa: bubble point pressure, MPaa
    :param bubblepointviscosity_cP: oil viscosity at bubble point pressure, cP
    :return: oil viscosity,cP

    ref2 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA

    """
    p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
    pb_psia = pb_MPaa * 145.03773800721814
    m = 2.6 * p_psia ** 1.187 * np.exp(-11.513 - 8.98 * 10 ** (-5) * p_psia)
    viscosity_cP = bubblepointviscosity_cP * (p_psia / pb_psia) ** m
    return viscosity_cP


def unf_undersaturatedoilviscosity_Petrovsky_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP):
    """
        Viscosity correlation for pressure above bubble point

    :param p_MPaa: pressure, MPaa
    :param pb_MPaa: bubble point pressure, MPaa
    :param bubblepointviscosity_cP: oil viscosity at bubble point pressure, cP
    :return: oil viscosity,cP

    ref 1 Petrosky, G.E. and Farshad, F.F. “Viscosity Correlations for Gulf of Mexico Crude
    Oils.” Paper SPE 29468. Presented at the SPE Production Operations Symposium,
    Oklahoma City (1995)
    """
    A = -1.0146 + 1.3322 * np.log(bubblepointviscosity_cP) - 0.4876 * np.log(
        bubblepointviscosity_cP) ** 2 - 1.15036 * np.log(bubblepointviscosity_cP) ** 3
    p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
    pb_psia = uc.Pa2psi(pb_MPaa * 10 ** 6)
    viscosity_cP = bubblepointviscosity_cP + 1.3449 * 10 ** (-3) * (p_psia - pb_psia) * 10 ** A
    return viscosity_cP


def unf_oil_viscosity_Beggs_VB_cP(deadoilviscosity_cP, rs_m3m3, p_MPaa, pb_MPaa):
    """
        Function for calculating the viscosity at any pressure

    :param deadoilviscosity_cP: dead oil viscosity,cP
    :param rs_m3m3: solution gas-oil ratio, m3m3
    :param p_MPaa: pressure, MPaa
    :param pb_MPaa: bubble point pressure, MPaa
    :return: oil viscosity,cP
    """
    saturatedviscosity_cP = unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, rs_m3m3)
    if p_MPaa <= pb_MPaa:
        viscosity_cP = saturatedviscosity_cP
    else:
        viscosity_cP = unf_undersaturatedoilviscosity_VB_cP(p_MPaa, pb_MPaa, saturatedviscosity_cP)
    return viscosity_cP


def unf_pb_Glaso_MPaa(rs_m3m3, t_K, gamma_oil, gamma_gas):
    """
        Glaso correlation(1980) for bubble point pressure

    :param rs_m3m3: gas-oil ratio in m3/m3
    :param t_K: temperature in K
    :param gamma_oil: oil density (by water)
    :param gamma_gas: gas density (by air)
    :return: bubble point pressure im MPa abs

    ref Generalized Pressure-Volume-Temperature Correlations, Glaso, 1980
    """

    # TODO также необходимо дополнить код, поправками на неув составляющую в нефти, в статье есть
    api = uc.gamma_oil2api(gamma_oil)
    t_F = uc.k2f(t_K)
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    pb = (rs_scfstb / gamma_gas) ** 0.816 * (t_F ** 0.172 / api ** 0.989)
    pb = uc.psi2Pa(10 ** (1.7669 + 1.7447 * np.log10(pb) - 0.30218 * np.log10(pb) ** 2)) / 10 ** 6
    return pb


def unf_fvf_Glaso_m3m3_saturated(rs_m3m3, t_K, gamma_oil, gamma_gas):
    """
        Glaso correlation(1980) for formation volume factor at bubble point pressure

    :param rs_m3m3: gas-oil ratio in m3/m3
    :param t_K: temperature in K
    :param gamma_oil: oil density (by water)
    :param gamma_gas: gas density (by air)
    :return: formation volume factor at bubble point pressure in m3/m3

    ref Generalized Pressure-Volume-Temperature Correlations, Glaso, 1980
    """

    t_F = uc.k2f(t_K)
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    bob = rs_scfstb * (gamma_gas / gamma_oil) ** 0.526 + 0.968 * t_F
    bob = 10 ** (-6.58511 + 2.91329 * np.log10(bob) - 0.27683 * np.log10(bob) ** 2) + 1
    return bob


def unf_fvf_Glaso_m3m3_below(rs_m3m3, t_K, gamma_oil, gamma_gas, p_MPaa):
    """
        Glaso correlation(1980) for total formation volume factor below bubble point pressure

    :param rs_m3m3: gas-oil ratio in m3/m3
    :param t_K: temperature in K
    :param gamma_oil: oil density (by water)
    :param gamma_gas: gas density (by air)
    :param p_MPaa: pressure in MPaa
    :return: total formation volume factor below bubble point pressure in m3/m3

    ref Generalized Pressure-Volume-Temperature Correlations, Glaso, 1980
    """

    t_F = uc.k2f(t_K)
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
    bt = rs_scfstb * t_F ** 0.5 / gamma_gas ** 0.3 * gamma_oil ** (2.9 * 10 ** (-0.00027 * rs_scfstb)) * p_psia ** \
        (-1.1089)
    bt = 10 ** (8.0135 * 10 ** (-2) + 4.7257 * 10 ** (-1) * np.log10(bt) + 1.7351 * 10 ** (-1) * np.log10(bt) ** 2)
    return bt


def unf_McCain_specificgravity(p_MPaa, rsb_m3m3, t_K, gamma_oil, gamma_gassp):
    """
    :param p_MPaa: pressure in MPaa
    :param rsb_m3m3: gas-oil ratio at bubble poinr pressure, m3/m3
    :param t_K: temperature in K
    :param gamma_oil: specific oil density(by water)
    :param gamma_gassp: specific gas density(by air) in separator
    :return: reservoir free gas specific gravity

    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    """

    api = uc.gamma_oil2api(gamma_oil)
    rsb_scfstb = uc.m3m3_2_scfstb(rsb_m3m3)
    p_psia = uc.MPa2psi(p_MPaa)
    t_F = uc.k2f(t_K)
    gamma_gasr = 1 / (-208.0797 / p_psia + 22.885 / p_psia ** 2 - 0.000063641 * p_psia + 3.38346 / t_F ** 0.5 -
                      0.000992 * t_F - 0.000081147 * rsb_scfstb - 0.001956 * api + 1.081956 / gamma_gassp + 0.394035 *
                      gamma_gassp ** 2)
    return gamma_gasr


def unf_heat_capacity_oil_Gambill_JkgC(gamma_oil, t_c):
    """
        Oil heat capacity in SI. Gambill correlation

    :param gamma_oil: specific oil density(by water)
    :param t_c: temperature in C
    :return: heat capacity in SI - JkgC

    ref1 Book: Brill J. P., Mukherjee H. K. Multiphase flow in wells. –
    Society of Petroleum Engineers, 1999. – Т. 17. in Page 122
    """

    t_f = uc.c2f(t_c)
    api = uc.gamma_oil2api(gamma_oil)
    heat_capacity_oil_btulbmF = ((0.388 + 0.00045 * t_f) / gamma_oil ** (1/2))
    return uc.btulbmF2kJkgK(heat_capacity_oil_btulbmF) * 1000


def unf_heat_capacity_oil_Wes_Wright_JkgC(gamma_oil, t_c):
    """
        Oil heat capacity in SI. Wes Wright method

    :param gamma_oil: specific oil density(by water)
    :param t_c: temperature in C
    :return: heat capacity in SI - JkgC

    ref1 https://www.petroskills.com/blog/entry/crude-oil-and-changing-temperature#.XQkEnogzaM8
    """
    return ((2 * 10** (-3) * t_c - 1.429 ) * gamma_oil +
            (2.67 * 10 ** (-3)) * t_c + 3.049) * 1000


def unf_thermal_conductivity_oil_Abdul_Seoud_Moharam_WmK(gamma_oil, t_c):
    """
        Oil thermal conductivity Abdul-Seoud and Moharam correlation

    :param gamma_oil: specific oil density(by water)
    :param t_c: temperature in C
    :return: thermal conductivity in SI - wt / m K

    ref1 Tovar L. P. et al. Overview and computational approach for studying the physicochemical characterization
    of high-boiling-point petroleum fractions (350 C+) //
    Oil & Gas Science and Technology–Revue d’IFP Energies nouvelles. – 2012. – Т. 67. – №. 3. – С. 451-477.
    """
    t_k = uc.c2k(t_c)
    return (2.540312 * (gamma_oil / t_k) ** 0.5) - 0.014485


def unf_thermal_conductivity_oil_Smith_WmK(gamma_oil, t_c):
    """
        Oil thermal conductivity Smith correlation for 273 < T < 423 K

    :param gamma_oil: specific oil density(by water)
    :param t_c: temperature in C
    :return: thermal conductivity in SI - wt / m K

    ref1 Das D. K., Nerella S., Kulkarni D. Thermal properties of petroleum and gas-to-liquid products //
    Petroleum science and technology. – 2007. – Т. 25. – №. 4. – С. 415-425.   """
    t_k = uc.c2k(t_c)
    return (0.137 / (gamma_oil * 1000) * (1 - 0.00054 * (t_k - 273)) * 10 ** 3)


def unf_thermal_conductivity_oil_Cragoe_WmK(gamma_oil, t_c):
    """
        Oil thermal conductivity Cragoe correlation for 273 < T < 423 K

    :param gamma_oil: specific oil density(by water)
    :param t_c: temperature in C
    :return: thermal conductivity in SI - wt / m K

    ref1 Das D. K., Nerella S., Kulkarni D. Thermal properties of petroleum and gas-to-liquid products //
    Petroleum science and technology. – 2007. – Т. 25. – №. 4. – С. 415-425.   """
    t_k = uc.c2k(t_c)
    return (0.118 /(gamma_oil * 1000) * (1 - 0.00054 * (t_k - 273)) * 10 ** 3)




"""
В дальнейшем функции для нефти будут дополняться, а пока перейдем к uPVT для газа
"""


# uPVT свойства для газа

def unf_pseudocritical_temperature_K(gamma_gas, y_h2s=0.0, y_co2=0.0, y_n2=0.0):
    """
        Correlation for pseudocritical temperature taking into account the presense of non-hydrocarbon gases

    :param gamma_gas: specific gas density (by air)
    :param y_h2s: mole fraction of the hydrogen sulfide
    :param y_co2: mole fraction of the carbon dioxide
    :param y_n2: mole fraction of the nitrogen
    :return: pseudocritical temperature, K

    ref 1 Piper, L.D., McCain, W.D., Jr., and Corredor, J.H. “Compressibility Factors for
    Naturally Occurring Petroleum Gases.” Gas Reservoir Engineering. Reprint Series. Richardson,
    TX: SPE. Vol. 52 (1999) 186–200
    """
    """
    tc_h2s_K,            critical temperature for hydrogen sulfide, K
    tc_co2_K,            critical temperature for carbon dioxide, K
    tc_n2_K,             critical temperature for nitrogen, K
    pc_h2s_MPaa,         critical pressure for hydrogen sulfide, MPaa
    pc_co2_MPaa,         critical pressure for carbon dioxide, MPaa
    pc_n2_MPaa,          critical pressure for nitrogen, MPaa
    """
    tc_h2s_R = uc.k2r(373.6)
    tc_co2_R = uc.k2r(304.13)
    tc_n2_R = uc.k2r(126.25)
    pc_h2s_psia = uc.Pa2psi(10 ** 6 * 9.007)

    pc_co2_psia = uc.Pa2psi(10 ** 6 * 7.375)
    pc_n2_psia = uc.Pa2psi(10 ** 6 * 3.4)
    J = 1.1582 * 10 ** (-1) - 4.5820 * 10 ** (-1) * y_h2s * (tc_h2s_R / pc_h2s_psia) - 9.0348 * 10 ** (-1) * y_co2 *\
        (tc_co2_R / pc_co2_psia) - 6.6026 * 10 ** (-1) * y_n2 * (
                        tc_n2_R / pc_n2_psia) + 7.0729 * 10 ** (-1) * gamma_gas - 9.9397 * 10 ** (-2) * gamma_gas ** 2
    K = 3.8216 - 6.5340 * 10 ** (-2) * y_h2s * (tc_h2s_R / pc_h2s_psia) - 4.2113 * 10 ** (-1) * y_co2 \
        * (tc_co2_R / pc_co2_psia) - 9.1249 * 10 ** (-1) * y_n2 * (
                        tc_n2_R / pc_n2_psia) + 1.7438 * 10 * gamma_gas - 3.2191 * gamma_gas ** 2
    tpc_R = K ** 2 / J
    tpc_K = uc.r2k(tpc_R)
    return tpc_K


def unf_pseudocritical_pressure_MPa(gamma_gas, y_h2s=0.0, y_co2=0.0, y_n2=0.0):
    """
        Correlation for pseudocritical pressure taking into account the presense of non-hydrocarbon gases

    :param gamma_gas: specific gas density (by air)
    :param y_h2s: mole fraction of the hydrogen sulfide
    :param y_co2: mole fraction of the carbon dioxide
    :param y_n2: mole fraction of the nitrogen
    :return: pseudocritical pressure, MPa

    ref 1 Piper, L.D., McCain, W.D., Jr., and Corredor, J.H. “Compressibility Factors for
    Naturally Occurring Petroleum Gases.” Gas Reservoir Engineering. Reprint Series. Richardson,
    TX: SPE. Vol. 52 (1999) 186–200
    """
    """          
    tc_h2s_K,            critical temperature for hydrogen sulfide, K
    tc_co2_K,            critical temperature for carbon dioxide, K
    tc_n2_K,             critical temperature for nitrogen, K
    pc_h2s_MPaa,         critical pressure for hydrogen sulfide, MPaa
    pc_co2_MPaa,         critical pressure for carbon dioxide, MPaa
    pc_n2_MPaa,          critical pressure for nitrogen, MPaa               
    """
    tc_h2s_R = uc.k2r(373.6)
    tc_co2_R = uc.k2r(304.13)
    tc_n2_R = uc.k2r(126.25)
    pc_h2s_psia = uc.Pa2psi(10 ** 6 * 9.007)
    pc_co2_psia = uc.Pa2psi(10 ** 6 * 7.375)
    pc_n2_psia = uc.Pa2psi(10 ** 6 * 3.4)
    J = 1.1582 * 10 ** (-1) - 4.5820 * 10 ** (-1) * y_h2s * (tc_h2s_R / pc_h2s_psia) - \
        9.0348 * 10 ** (-1) * y_co2 * (tc_co2_R / pc_co2_psia) - 6.6026 * 10 ** (-1) * y_n2 * (
                        tc_n2_R / pc_n2_psia) + 7.0729 * 10 ** (-1) * gamma_gas - 9.9397 * 10 ** (-2) * gamma_gas ** 2
    K = 3.8216 - 6.5340 * 10 ** (-2) * y_h2s * (tc_h2s_R / pc_h2s_psia) - \
        4.2113 * 10 ** (-1) * y_co2 * (tc_co2_R / pc_co2_psia) - 9.1249 * 10 ** (-1) * y_n2 * (
                        tc_n2_R / pc_n2_psia) + 1.7438 * 10 * gamma_gas - 3.2191 * gamma_gas ** 2
    tpc_R = K ** 2 / J
    ppc_psia = tpc_R / J
    ppc_MPa = uc.psi2Pa(ppc_psia / 10 ** 6)
    return ppc_MPa


def unf_pseudocritical_temperature_Standing_K(gamma_gas):  # VBA
    return 93.3 + 180 * gamma_gas - 6.94 * gamma_gas ** 2


def unf_pseudocritical_pressure_Standing_MPa(gamma_gas):  # VBA
    return 4.6 + 0.1 * gamma_gas - 0.258 * gamma_gas ** 2


def unf_zfactor_BrillBeggs(ppr, tpr):
    """
        Correlation for z-factor according Beggs & Brill correlation (1977)

    используется для приближения функции дранчука

    :param ppr: preudoreduced pressure
    :param tpr: pseudoreduced temperature
    :return: z-factor

    Можно использовать при tpr<=2 и ppr<=4
    при tpr <== 1.5 ppr<=10
    """

    a = 1.39 * (tpr - 0.92) ** 0.5 - 0.36 * tpr - 0.101
    b = (0.62 - 0.23 * tpr) * ppr
    c = (0.066/(tpr - 0.86) - 0.037) * ppr ** 2
    d = (0.32/(10 ** (9 * (tpr - 1)))) * ppr ** 6
    e = b + c + d
    f = (0.132 - 0.32 * np.log10(tpr))
    g = 10 ** (0.3106 - 0.49 * tpr + 0.1824 * tpr ** 2)
    z = a + (1 - a) * np.exp(-e) + f * ppr ** g
    return z


def unf_zfactor_DAK(p_MPaa, t_K, ppc_MPa, tpc_K):
    """
        Correlation for z-factor

    :param p_MPaa: pressure, MPaa
    :param t_K: temperature, K
    :param ppc_MPa: pseudocritical pressure, MPa
    :param tpc_K: pseudocritical temperature, K
    :return: z-factor

    range of applicability is (0.2<=ppr<30 and 1.0<tpr<=3.0) and also ppr < 1.0 for 0.7 < tpr < 1.0

    ref 1 Dranchuk, P.M. and Abou-Kassem, J.H. “Calculation of Z Factors for Natural
    Gases Using Equations of State.” Journal of Canadian Petroleum Technology. (July–September 1975) 34–36.

    """

    ppr = p_MPaa / ppc_MPa
    tpr = t_K / tpc_K
    z0 = 1
    ropr0 = 0.27 * (ppr / (z0 * tpr))

    def f(variables):
        z, ropr = variables
        func = np.zeros(2)
        func[0] = 0.27 * (ppr / (z * tpr)) - ropr
        func[1] = -z + 1 + (0.3265 - 1.0700 / tpr - 0.5339 / tpr**3 + 0.01569 / tpr ** 4 - 0.05165 / tpr ** 5) * ropr +\
            (0.5475 - 0.7361 / tpr + 0.1844 / tpr ** 2) * ropr ** 2 - 0.1056 * (-0.7361 / tpr + 0.1844 / tpr ** 2) *\
            ropr ** 5 + 0.6134 * (1 + 0.7210 * ropr ** 2) * (ropr ** 2 / tpr ** 3) * np.exp(-0.7210 * ropr ** 2)
        return func
    solution = opt.fsolve(f, np.array([z0, ropr0]))
    return solution[0]


def unf_zfactor_DAK_ppr(ppr, tpr):
    """
        Correlation for z-factor

    :param ppr: pseudoreduced pressure
    :param tpr: pseudoreduced temperature
    :return: z-factor

    range of applicability is (0.2<=ppr<30 and 1.0<tpr<=3.0) and also ppr < 1.0 for 0.7 < tpr < 1.0

    ref 1 Dranchuk, P.M. and Abou-Kassem, J.H. “Calculation of Z Factors for Natural
    Gases Using Equations of State.” Journal of Canadian Petroleum Technology. (July–September 1975) 34–36.

    """

    z0 = 1
    ropr0 = 0.27 * (ppr / (z0 * tpr))

    def f(variables):
        z, ropr = variables
        func = np.zeros(2)
        func[0] = 0.27 * (ppr / (z * tpr)) - ropr
        func[1] = -z + 1 + (0.3265 - 1.0700 / tpr - 0.5339 / tpr**3 + 0.01569 / tpr ** 4 - 0.05165 / tpr ** 5) * ropr +\
            (0.5475 - 0.7361 / tpr + 0.1844 / tpr ** 2) * ropr ** 2 - 0.1056 * (-0.7361 / tpr + 0.1844 / tpr ** 2) *\
            ropr ** 5 + 0.6134 * (1 + 0.7210 * ropr ** 2) * (ropr ** 2 / tpr ** 3) * np.exp(-0.7210 * ropr ** 2)
        return func
    solution = opt.fsolve(f, np.array([z0, ropr0]))
    """
    def f(z):
        func = -z + 1 + (0.3265 - 1.0700 / tpr - 0.5339 / tpr ** 3 + 0.01569 / tpr ** 4 - 0.05165 / tpr ** 5) *\
               (0.27 * (ppr / (z * tpr))) + (0.5475 - 0.7361 / tpr + 0.1844 / tpr ** 2) * (0.27 * (ppr / (z * tpr))) **\
               2 - 0.1056 * (-0.7361 / tpr + 0.1844 / tpr ** 2) * (0.27 * (ppr / (z * tpr))) ** 5 + 0.6134 *\
               (1 + 0.7210 * (0.27 * (ppr / (z * tpr))) ** 2) * ((0.27 * (ppr / (z * tpr))) ** 2 / tpr ** 3) *\
               np.exp(-0.7210 * (0.27 * (ppr / (z * tpr))) ** 2)
        return func
    solution = opt.newton(f, z0, maxiter=150, tol=1e-4)
    """
    return solution[0]


def unf_z_factor_Kareem(Tpr, Ppr):
    """
    based on  https://link.springer.com/article/10.1007/s13202-015-0209-3
    Kareem, L.A., Iwalewa, T.M. & Al-Marhoun, M.
    New explicit correlation for the compressibility factor of natural gas: linearized z-factor isotherms.
    J Petrol Explor Prod Technol 6, 481–492 (2016).
    https://doi.org/10.1007/s13202-015-0209-3
    :param Tpr:
    :param Ppr:
    :return:
    """
    a = [0] * 20
    a[1] = 0.317842
    a[11] = -1.966847
    a[2] = 0.382216
    a[12] = 21.0581
    a[3] = -7.768354
    a[13] = -27.0246
    a[4] = 14.290531
    a[14] = 16.23
    a[5] = 0.000002
    a[15] = 207.783
    a[6] = -0.004693
    a[16] = -488.161
    a[7] = 0.096254
    a[17] = 176.29
    a[8] = 0.16672
    a[18] = 1.88453
    a[9] = 0.96691
    a[19] = 3.05921
    a[10] = 0.063069

    t = 1 / Tpr
    AA = a[1]* t * np.exp(a[2] * (1 - t) ** 2) * Ppr
    BB = a[3]* t + a[4] * t ** 2 + a[5] * t ** 6 * Ppr ** 6
    CC = a[9]+ a[8] * t * Ppr + a[7] * t ** 2 * Ppr ** 2 + a[6] * t ** 3 * Ppr ** 3
    DD = a[10] * t * np.exp(a[11] * (1 - t) ** 2)
    EE = a[12] * t + a[13] * t ** 2 + a[14] * t ** 3
    FF = a[15] * t + a[16] * t ** 2 + a[17] * t ** 3
    GG = a[18] + a[19] * t

    DPpr = DD * Ppr
    y = DPpr / ((1 + AA ** 2) / CC - AA ** 2 * BB / (CC ** 3))

    z = DPpr * (1 + y + y ** 2 - y ** 3) / (DPpr + EE * y ** 2 - FF * y ** GG) / ((1 - y) ** 3)

    return z


def unf_compressibility_gas_Mattar_1MPa(p_MPaa, t_K, ppc_MPa, tpc_K):
    """
        Correlation for gas compressibility

    :param p_MPaa: pressure, MPaa
    :param t_K: temperature, K
    :param ppc_MPa: pseudocritical pressure, MPa
    :param tpc_K: pseudocritical temperature, K
    :return: gas compressibility, 1/MPa

    ref 1 Mattar, L., Brar, G.S., and Aziz, K. 1975. Compressibility of Natural Gases.
    J Can Pet Technol 14 (4): 77. PETSOC-75-04-08

    """

    ppr = p_MPaa / ppc_MPa
    tpr = t_K / tpc_K
    z0 = 1
    ropr0 = 0.27 * (ppr / (z0 * tpr))

    def f(variables):
        z = variables[0]
        ropr = variables[1]
        func = np.zeros(2)
        func[0] = 0.27 * (ppr / (z * tpr)) - ropr
        func[1] = -z + 1 + (0.3265 - 1.0700 / tpr - 0.5339 / tpr ** 3 + 0.01569 / tpr ** 4 - 0.05165 / tpr ** 5) * ropr +\
            (0.5475 - 0.7361 / tpr + 0.1844 / tpr ** 2) * ropr ** 2 - 0.1056 * (-0.7361 / tpr + 0.1844 / tpr ** 2) *\
            ropr ** 5 + 0.6134 * (1 + 0.7210 * ropr ** 2) * (ropr ** 2 / tpr ** 3) * np.exp(-0.7210 * ropr ** 2)
        return func
    solution = np.array(opt.fsolve(f, np.array([z0, ropr0])))
    z_derivative = 0.3265 - 1.0700 / tpr - 0.5339 / tpr ** 3 + 0.01569 / tpr ** 4 - 0.05165 / tpr ** 5 + 2 *\
        solution[1] * (0.5475 - 0.7361 / tpr + 0.1844 / tpr ** 2) - 5 * 0.1056 * (-0.7361 / tpr + 0.1844 / tpr ** 2) *\
        solution[1] ** 4 + 2 * 0.6134 * solution[1] / tpr ** 3 * (1 + 0.7210 * solution[1] ** 2 - 0.7210 ** 2 *
                                                             solution[1] ** 4) * np.exp(-0.7210 * solution[1] ** 2)
    cpr = 1 / solution[1] - 0.27 / (solution[0] ** 2 * tpr) * (z_derivative / (1 + solution[1] * z_derivative /
                                                                               solution[0]))
    cg = cpr / ppc_MPa
    return cg


def unf_gasviscosity_Lee_cP(t_K, p_MPaa, z, gamma_gas):
    """
        Lee correlation for gas viscosity

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :param z: z-factor
    :param gamma_gas: specific gas density (by air)
    :return: gas viscosity,cP

    ref 1 Lee, A.L., Gonzalez, M.H., and Eakin, B.E. “The Viscosity of Natural Gases.” Journal
    of Petroleum Technology. Vol. 18 (August 1966) 997–1,000.
    """

    t_R = uc.k2r(t_K)
    m = 28.966 * gamma_gas  # Molar mass
    a = ((9.379 + 0.01607 * m) * t_R ** 1.5)/(209.2 + 19.26 * m + t_R)
    b = 3.448 + 986.4/t_R + 0.01009 * m
    c = 2.447 - 0.2224 * b
    ro_gas = p_MPaa * m/(z * t_K * 8.31)
    gasviscosity_cP = 10**(-4) * a * np.exp(b * ro_gas**c)
    return gasviscosity_cP


def unf_gas_fvf_m3m3(t_K, p_MPaa, z):
    """
        Equation for gas FVF

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :param z: z-factor
    :return: formation volume factor for gas bg, m3/m3
    """

    bg = 101.33 * 10**(-3) * t_K * z / (1 * 293.15 * p_MPaa) # тут от нормальный условий по температуре
    return bg


def unf_fvf_gas_vba_m3m3(T_K, z, P_MPa):
    return 0.00034722 * T_K * z / P_MPa  # от какой температуры?


def unf_gas_density_kgm3(t_K, p_MPaa, gamma_gas, z):
    """
        Equation for gas density from state equation

    :param t_K: temperature
    :param p_MPaa: pressure
    :param gamma_gas: specific gas density by air
    :param z: z-factor
    :return: gas density
    """
    m = gamma_gas * 0.029
    p_Pa = 10 ** 6 * p_MPaa
    rho_gas = p_Pa * m / (z * 8.31 * t_K)
    return rho_gas


def unf_gas_density_VBA_kgm3(gamma_gas, bg_m3m3):
    return gamma_gas * uc.air_density_sckgm3 / bg_m3m3


def unf_heat_capacity_gas_Mahmood_Moshfeghian_JkgC(p_MPaa, t_K, gamma_gas):
    """
        Gas heat capacity by Mahmood Moshfeghian for 0.1 to 20 MPa
    should be noted that the concept of heat capacity is valid only for the single phase region.

    :param p_MPaa: pressure, MPaa
    :param t_K: temperature, K
    :param gamma_gas: specific gas density by air
    :return: gas heat capacity in JkgC = JkgK

    ref1 https://www.jmcampbell.com/tip-of-the-month/2009/07/
    variation-of-natural-gas-heat-capacity-with-temperature-pressure-and-relative-density/
    """
    t_c = uc.k2c(t_K)
    a = 0.9
    b = 1.014
    c = -0.7
    d = 2.170
    e = 1.015
    f = 0.0214
    return ((a * (b**t_c) * (t_c**c) + d * (e**p_MPaa) * (p_MPaa**f)) * ((gamma_gas / 0.60) ** 0.025)) * 1000


def unf_thermal_conductivity_gas_methane_WmK(t_c): # TODO заменить
    """
        Теплопроводность метана

    :param t_c: температура в С
    :return: теплопроводность в Вт / м К

    Данная функкия является линейным приближением табличных значений при 1 бар
    требует корректировки, является временной затычкой, взята от безысходности
    """
    return (42.1 + (42.1 - 33.1)/(80-18)*(t_c - 80))/1000
# uPVT свойства для сжимаемости нефти(требует немного свойств газа)

"""
def unf_weightedcompressibility_oil_Mccain_1MPa_greater(gamma_oil, gamma_gas, pb_MPa, p_MPa, rsb_m3m3, tres_K, gamma_gassp = 0):

    pass


def unf_compressibility_oil_Mccain_1MPa_greater(gamma_oil, gamma_gas, pb_MPa, p_MPa, rsb_m3m3, tres_K, gamma_gassp = 0):

    pass


def unf_compressibility_oil_Mccain_1MPa_lower():
    pass
"""

# uPVT свойства для воды


def unf_density_brine_uniflocvba_kgm3(gamma_w, bw_m3m3):
    """
        Equation from UniflocVBA

    :param gamma_w:
    :param bw_m3m3:
    :return:
    """
    rho_wat_rc_kgm3 = 1000 * gamma_w / bw_m3m3
    return rho_wat_rc_kgm3


def unf_density_brine_Spivey_kgm3(t_K, p_MPaa, s_ppm, par=1):
    """
        Modified Spivey et al. correlation for brine(water) density (2009)

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :param s_ppm: salinity, ppm
    :param par: parameter, 0 - methane-free brine, 1 - brine containing dissolved methane
    :return: density, kg/m3

        корреляция позволяет найти плотность соленой воды с растворенным в ней метаном

        ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.

    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    """

    t_C = uc.k2c(t_K)
    s = s_ppm / 1000000
    m = 1000 * s / (58.4428 * (1 - s))
    # Первым шагом вычисляется плотность чистой воды при давлении 70 MPa и температуре
    ro_w70 = (-0.127213 * (t_C/100)**2 + 0.645486 * (t_C/100) + 1.03265)/(-0.070291 * (t_C/100)**2 +
                                                                          0.639589 * (t_C/100) + 1)
    # Температурные коэффициенты сжимаемости чистой воды
    ew = (4.221 * (t_C / 100)**2 - 3.478 * (t_C/100) + 6.221)/(0.5182 * (t_C/100)**2 - 0.4405 * (t_C/100) + 1)
    fw = (-11.403 * (t_C / 100) ** 2 + 29.932 * (t_C / 100) + 27.952) / (0.20684 * (t_C / 100) ** 2 +
                                                                         0.3768 * (t_C / 100) + 1)
    iw70 = np.log(abs(ew + fw))/ew
    iw = np.log(abs(ew*(p_MPaa/70) + fw))/ew
    # Плотность чистой воды при T, P
    ro_w = ro_w70 * np.exp(iw - iw70)
    # Температурные коэффициенты плотности раствора
    d_m1 = -1.1149 * 10 ** (-4) * (t_C/100)**2 + 1.7105 * 10 ** (-4) * (t_C/100) - 4.3766 * 10 ** (-4)
    d_m2 = (-8.878 * 10 ** (-4) * (t_C/100)**2 - 1.388 * 10 ** (-4) - 2.96318 * 10 ** (-3))/(0.51103 * (t_C / 100) + 1)
    d_m3 = (2.1466 * 10 ** (-3) * (t_C / 100) ** 2 + 1.2427 * 10 ** (-2) * (t_C / 100) + 4.2648 * 10 ** (-2)) / \
           (-8.1009 * 10 ** (-2) * (t_C / 100) ** 2 + 0.525417 * (t_C / 100) + 1)
    d_m4 = 2.356 * 10 ** (-4) * (t_C/100)**2 - 3.636 * 10 ** (-4) * (t_C/100) - 2.278 * 10 ** (-4)
    # Плотность раствора воды с хлоридом натрия при давлении 70 MPa и температуре
    ro_b70 = ro_w70 + d_m1 * m ** 2 + d_m2 * m ** 1.5 + d_m3 * m + d_m4 * m ** 0.5
    # Температурные коэффициенты сжимаемости раствора
    eb = ew + 0.1249
    f_m1 = (-0.617 * (t_C / 100) ** 2 - 0.747 * (t_C / 100) - 0.4339) / (10.26 * (t_C / 100) + 1)
    f_m2 = (9.917 * (t_C / 100) + 5.1128) / (3.892 * (t_C / 100) + 1)
    f_m3 = 0.0365 * (t_C / 100) ** 2 - 0.0369 * (t_C / 100)
    fb = fw + f_m1 * m ** 1.5 + f_m2 * m + f_m3 * m ** 0.5
    ib70 = np.log(abs(eb + fb))/eb
    ib = np.log(abs(eb * p_MPaa / 70 + fb)) / eb
    # Плотность раствора при T, P
    ro_b = ro_b70 * np.exp(ib - ib70)
    if s == 0:
        ro = ro_w
    elif par == 0:
        ro = ro_b
    elif par == 1:
        # Найдем растворимость метана в растворе
        # Сперва определим давление насыщенных паров для чистой воды
        eps = 1 - t_K/647.096
        p_sigma = 22.064 * np.exp(647.096/t_K * (-7.85951783 * eps + 1.84408259 * eps ** 1.5 - 11.7866497 *
                                                 eps ** 3 + 22.6807411 * eps ** 3.5 - 15.9619719 * eps ** 4 +
                                                 1.80122502 * eps ** 7.5))
        # Определим коэффициенты растворимости метана
        a = -0.004462 * (t_C / 100) - 0.06763
        b = -0.03602 * (t_C / 100) ** 2 + 0.18917 * (t_C / 100) + 0.97242
        c = (0.6855 * (t_C / 100) ** 2 - 3.1992 * (t_C / 100) - 3.7968) / (0.07711 * (t_C / 100) ** 2 + 0.2229 *
                                                                           (t_C / 100) + 1)
        # Растворимость метана в чистой воде
        m_ch4_w = np.exp(a * (np.log(p_MPaa - p_sigma))**2 + b * np.log(p_MPaa - p_sigma) + c)
        # Далее найдем коэффициенты взаимодействия
        lyambda = -0.80898 + 1.0827 * 10 ** (-3) * t_C + 183.85 / t_C + 3.924 * 10 ** (-4) * p_MPaa - 1.97 *\
            10 ** (-6) * p_MPaa ** 2
        dzeta = -3.89 * 10 ** (-3)
        # Растворимость метана в растворе
        # нужно отметить, что в обозначениях было сложно разобраться, для понимания этой формулы лучше читать статью
        m_ch4_b = m_ch4_w * np.exp(-2 * lyambda * m - dzeta*m**2)
        # Производные необходимые для расчета формулы
        derivative1 = 7.6985890 * 10 ** (-2) - 5.0253331 * 10 ** (-5) * t_K - 30.092013 / t_K +\
            4.8468502 * 10 ** 3 / t_K ** 2
        derivative2 = 3.924 * 10 ** (-4) - 2 * 1.97 * 10 ** (-6) * p_MPaa
        # Парциальный объем метана в растворе
        v_ch4_b = 8.314467 * t_K * (derivative1 + 2 * m * derivative2)
        # Удельный объем раствора без метана
        v_b0 = 1 / ro_b
        # Плотность раствора с растворенным в нем метаном
        ro_b_ch4 = (1000 + m * 58.4428 + m_ch4_b * 16.043)/((1000 + m * 58.4428) * v_b0 + m_ch4_b * v_ch4_b)
        ro = ro_b_ch4
    else:
        ro = 0
    ro = ro * 1000
    return ro


def unf_compressibility_brine_Spivey_1MPa(t_K, p_MPaa, s_ppm, z=1.0, par=1):
    """
        Modified Spivey et al. correlation for brine(water) compressibility (2009)

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :param s_ppm: salinity, ppm
    :param z: z-factor
    :param par: parameter, 0 - methane-free brine, 1 - brine containing dissolved methane,
                            2 - brine containing partially dissolved methane
    :return: compressibility, 1/MPa

    корреляция позволяет найти сжимаемость соленой воды с частично или полностью растворенным в ней метаном

    ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.

    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    """

    t_C = uc.k2c(t_K)
    s = s_ppm / 1000000
    m = 1000 * s / (58.4428 * (1 - s))
    # Первым шагом вычисляется плотность чистой воды при давлении 70 MPa и температуре
    ro_w70 = (-0.127213 * (t_C / 100) ** 2 + 0.645486 * (t_C / 100) + 1.03265) / (-0.070291 * (t_C / 100) ** 2 +
                                                                                  0.639589 * (t_C / 100) + 1)
    # Температурные коэффициенты сжимаемости чистой воды
    ew = (4.221 * (t_C / 100)**2 - 3.478 * (t_C/100) + 6.221)/(0.5182 * (t_C/100)**2 - 0.4405 * (t_C/100) + 1)
    fw = (-11.403 * (t_C / 100) ** 2 + 29.932 * (t_C / 100) + 27.952) / (0.20684 * (t_C / 100) ** 2 +
                                                                         0.3768 * (t_C / 100) + 1)
    # Сжимаемость чистой воды при T, P
    c_w = (1/70) * 1 / (ew * (p_MPaa/70) + fw)
    # Температурные коэффициенты плотности раствора
    d_m1 = -1.1149 * 10 ** (-4) * (t_C/100)**2 + 1.7105 * 10 ** (-4) * (t_C/100) - 4.3766 * 10 ** (-4)
    d_m2 = (-8.878 * 10 ** (-4) * (t_C/100)**2 - 1.388 * 10 ** (-4) - 2.96318 * 10 ** (-3))/(0.51103 * (t_C / 100) + 1)
    d_m3 = (2.1466 * 10 ** (-3) * (t_C / 100) ** 2 + 1.2427 * 10 ** (-2) * (t_C / 100) + 4.2648 * 10 ** (-2)) / \
           (-8.1009 * 10 ** (-2) * (t_C / 100) ** 2 + 0.525417 * (t_C / 100) + 1)
    d_m4 = 2.356 * 10 ** (-4) * (t_C/100)**2 - 3.636 * 10 ** (-4) * (t_C/100) - 2.278 * 10 ** (-4)
    # Плотность раствора воды с хлоридом натрия при давлении 70 MPa и температуре
    ro_b70 = ro_w70 + d_m1 * m ** 2 + d_m2 * m ** 1.5 + d_m3 * m + d_m4 * m ** 0.5
    # Температурные коэффициенты сжимаемости раствора
    eb = ew + 0.1249
    f_m1 = (-0.617 * (t_C / 100) ** 2 - 0.747 * (t_C / 100) - 0.4339) / (10.26 * (t_C / 100) + 1)
    f_m2 = (9.917 * (t_C / 100) + 5.1128) / (3.892 * (t_C / 100) + 1)
    f_m3 = 0.0365 * (t_C / 100) ** 2 - 0.0369 * (t_C / 100)
    fb = fw + f_m1 * m ** 1.5 + f_m2 * m + f_m3 * m ** 0.5
    ib70 = np.log(abs(eb + fb)) / eb
    ib = np.log(abs(eb * p_MPaa / 70 + fb)) / eb
    # Плотность раствора при T, P
    ro_b = ro_b70 * np.exp(ib - ib70)
    # Сжимаемость соленого раствора при T, P
    c_b = (1/70) * 1 / (eb * (p_MPaa/70) + fb)
    # Найдем растворимость метана в растворе
    # Сперва определим давление насыщенных паров для чистой воды
    eps = 1 - t_K/647.096
    p_sigma = 22.064 * np.exp(647.096/t_K * (-7.85951783 * eps + 1.84408259 * eps ** 1.5 - 11.7866497 * eps ** 3 +
                                             22.6807411 * eps ** 3.5 - 15.9619719 * eps ** 4 + 1.80122502 * eps ** 7.5))
    # Определим коэффициенты растворимости метана
    a = -0.004462 * (t_C / 100) - 0.06763
    b = -0.03602 * (t_C / 100) ** 2 + 0.18917 * (t_C / 100) + 0.97242
    c = (0.6855 * (t_C / 100) ** 2 - 3.1992 * (t_C / 100) - 3.7968) / (0.07711 * (t_C / 100) ** 2 + 0.2229 * (t_C / 100)
                                                                       + 1)
    # Растворимость метана в чистой воде
    m_ch4_w = np.exp(a * (np.log(p_MPaa - p_sigma))**2 + b * np.log(p_MPaa - p_sigma) + c)
    # Далее найдем коэффициенты взаимодействия
    lyambda = -0.80898 + 1.0827 * 10 ** (-3) * t_C + 183.85 / t_C + 3.924 * 10 ** (-4) * p_MPaa - 1.97 * \
        10 ** (-6) * p_MPaa ** 2
    dzeta = -3.89 * 10 ** (-3)
    # Растворимость метана в растворе
    # нужно отметить, что в обозначениях было сложно разобраться, для понимания этой формулы лучше читать статью
    m_ch4_b = m_ch4_w * np.exp(-2 * lyambda * m - dzeta*m**2)
    # Производные необходимые для расчета формулы
    derivative1 = 7.6985890 * 10 ** (-2) - 5.0253331 * 10 ** (-5) * t_K - 30.092013 / t_K +\
        4.8468502 * 10 ** 3 / t_K ** 2
    derivative2 = 3.924 * 10 ** (-4) - 2 * 1.97 * 10 ** (-6) * p_MPaa
    # Парциальный объем метана в растворе
    v_ch4_b = 8.314467 * t_K * (derivative1 + 2 * m * derivative2)
    # Удельный объем раствора без метана
    v_b0 = 1 / ro_b
    # Необходимые вторые производные
    dderivative1 = - v_b0 * c_b
    dderivative2 = 2 * (-1.97 * 10 ** (-6))
    # Производная парциального объема метана в растворе
    derivative_v_ch4_b = 8.314467 * t_K * (2 * m * dderivative2)
    # Сжимаемость раствора в котором метан растворен полностью (однофазная система)
    c_b_ch4_u = -((1000 + m * 58.4428) * dderivative1 + m_ch4_b * derivative_v_ch4_b) / ((1000 + m * 58.4428) * v_b0
                                                                                        + m_ch4_b * v_ch4_b)
    # Найдем сжимаемость раствора в случае двуфазной системы
    # Необходимая производная от растворимости метана по давлению
    derivative_m_ch4_b = m_ch4_w * ((2 * a * np.log(p_MPaa - p_sigma) + b)/(p_MPaa - p_sigma) - 2 * m * derivative2)
    # Молярный объем метана в газовой фазе
    vm_ch4_g = z * 8.314467 * t_K / p_MPaa
    # Cжимаемость раствора в случае двуфазной системы
    c_b_ch4_s = -((1000 + m * 58.4428) * dderivative1 + m_ch4_b * derivative_v_ch4_b + derivative_m_ch4_b *
                 (v_ch4_b - vm_ch4_g)) / ((1000 + m * 58.4428) * v_b0 + m_ch4_b * v_ch4_b)
    if s == 0:
        c_1MPa = c_w
    elif par == 0:
        c_1MPa = c_b
    elif par == 1:
        c_1MPa = c_b_ch4_u
    elif par == 2:
        c_1MPa = c_b_ch4_s
    else:
        c_1MPa = 0
    return c_1MPa


def unf_fvf_brine_McCain_m3m3(t_K, p_MPaa):
    """
        FVF of brine by McCain

        https://petrowiki.org/Produced_water_formation_volume_factor

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :return: formation volume factor, m3/m3
    """
    t_f = uc.k2f(t_K)
    p_psi = uc.bar2psi(p_MPaa * 10)
    dvwp = -1.95301 * 10 ** (-9) * p_psi * t_f - 1.72834 * 10 ** (-13) * p_psi ** 2 * t_f - 3.58922 * 10 ** (
        -7) * p_psi - 2.25341 * 10 ** (-10) * p_psi ** 2
    dvwt = -1.0001 * 10 ** (-2) + 1.33391 * 10 ** (-4) * t_f + 5.50654 * 10 ** (-7) * t_f ** 2
    fvf_brine_McCain = (1 + dvwp) * (1 + dvwt)
    return fvf_brine_McCain


def unf_fvf_brine_Spivey_m3m3(t_K, p_MPaa, s_ppm):  # TODO check
    """
        Modified Spivey et al. correlation for brine(water) formation volume factor (2009)

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :param s_ppm: salinity, ppm
    :return: formation volume factor, m3/m3

    корреляция позволяет найти объемный коэффициент для соленой воды с учетом растворенного метана

    ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.

    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    """

    t_C = uc.k2c(t_K)
    s = s_ppm / 1000000
    m = 1000 * s / (58.4428 * (1 - s))
    # Первым шагом вычисляется плотность чистой воды при давлении 70 MPa и температуре
    ro_w70 = (-0.127213 * (t_C / 100) ** 2 + 0.645486 * (t_C / 100) + 1.03265) / (-0.070291 * (t_C / 100) ** 2 +
                                                                                  0.639589 * (t_C / 100) + 1)
    ro_w70_sc = (-0.127213 * (20 / 100) ** 2 + 0.645486 * (20 / 100) + 1.03265) / (-0.070291 * (20 / 100) ** 2 +
                                                                                   0.639589 * (20 / 100) + 1)
    # Температурные коэффициенты сжимаемости чистой воды
    ew = (4.221 * (t_C / 100) ** 2 - 3.478 * (t_C / 100) + 6.221) / (
                0.5182 * (t_C / 100) ** 2 - 0.4405 * (t_C / 100) + 1)
    fw = (-11.403 * (t_C / 100) ** 2 + 29.932 * (t_C / 100) + 27.952) / (0.20684 * (t_C / 100) ** 2 +
                                                                         0.3768 * (t_C / 100) + 1)
    ew_sc = (4.221 * (20 / 100) ** 2 - 3.478 * (20 / 100) + 6.221) / (
                0.5182 * (20 / 100) ** 2 - 0.4405 * (20 / 100) + 1)
    fw_sc = (-11.403 * (20 / 100) ** 2 + 29.932 * (20 / 100) + 27.952) / (0.20684 * (20 / 100) ** 2 +
                                                                          0.3768 * (20 / 100) + 1)
    # Температурные коэффициенты плотности раствора
    d_m1 = -1.1149 * 10 ** (-4) * (t_C / 100) ** 2 + 1.7105 * 10 ** (-4) * (t_C / 100) - 4.3766 * 10 ** (-4)
    d_m2 = (-8.878 * 10 ** (-4) * (t_C / 100) ** 2 - 1.388 * 10 ** (-4) - 2.96318 * 10 ** (-3)) / (
                0.51103 * (t_C / 100) + 1)
    d_m3 = (2.1466 * 10 ** (-3) * (t_C / 100) ** 2 + 1.2427 * 10 ** (-2) * (t_C / 100) + 4.2648 * 10 ** (-2)) / \
           (-8.1009 * 10 ** (-2) * (t_C / 100) ** 2 + 0.525417 * (t_C / 100) + 1)
    d_m4 = 2.356 * 10 ** (-4) * (t_C / 100) ** 2 - 3.636 * 10 ** (-4) * (t_C / 100) - 2.278 * 10 ** (-4)
    d_m1_sc = -1.1149 * 10 ** (-4) * (20 / 100) ** 2 + 1.7105 * 10 ** (-4) * (20 / 100) - 4.3766 * 10 ** (-4)
    d_m2_sc = (-8.878 * 10 ** (-4) * (20 / 100) ** 2 - 1.388 * 10 ** (-4) - 2.96318 * 10 ** (-3)) / (
            0.51103 * (20 / 100) + 1)
    d_m3_sc = (2.1466 * 10 ** (-3) * (20 / 100) ** 2 + 1.2427 * 10 ** (-2) * (20 / 100) + 4.2648 * 10 ** (-2)) / \
        (-8.1009 * 10 ** (-2) * (20 / 100) ** 2 + 0.525417 * (20 / 100) + 1)
    d_m4_sc = 2.356 * 10 ** (-4) * (20 / 100) ** 2 - 3.636 * 10 ** (-4) * (20 / 100) - 2.278 * 10 ** (-4)
    # Плотность раствора воды с хлоридом натрия при давлении 70 MPa и температуре
    ro_b70 = ro_w70 + d_m1 * m ** 2 + d_m2 * m ** 1.5 + d_m3 * m + d_m4 * m ** 0.5
    ro_b70_sc = ro_w70_sc + d_m1_sc * m ** 2 + d_m2_sc * m ** 1.5 + d_m3_sc * m + d_m4_sc * m ** 0.5
    # Температурные коэффициенты сжимаемости раствора
    eb = ew + 0.1249
    eb_sc = ew_sc + 0.1249
    f_m1 = (-0.617 * (t_C / 100) ** 2 - 0.747 * (t_C / 100) - 0.4339) / (10.26 * (t_C / 100) + 1)
    f_m2 = (9.917 * (t_C / 100) + 5.1128) / (3.892 * (t_C / 100) + 1)
    f_m3 = 0.0365 * (t_C / 100) ** 2 - 0.0369 * (t_C / 100)
    f_m1_sc = (-0.617 * (20 / 100) ** 2 - 0.747 * (20 / 100) - 0.4339) / (10.26 * (20 / 100) + 1)
    f_m2_sc = (9.917 * (20 / 100) + 5.1128) / (3.892 * 20 / 100 + 1)
    f_m3_sc = 0.0365 * (20 / 100) ** 2 - 0.0369 * (20 / 100)
    fb = fw + f_m1 * m ** 1.5 + f_m2 * m + f_m3 * m ** 0.5
    fb_sc = fw_sc + f_m1_sc * m ** 1.5 + f_m2_sc * m + f_m3_sc * m ** 0.5
    ib70 = np.log(abs(eb + fb)) / eb
    ib70_sc = np.log(abs(eb_sc + fb_sc)) / eb_sc
    ib = np.log(abs(eb * p_MPaa / 70 + fb)) / eb
    ib_sc = np.log(abs(eb_sc * p_MPaa / 70 + fb_sc)) / eb_sc
    # Плотность раствора при T, P
    ro_b = ro_b70 * np.exp(ib - ib70)
    ro_b_sc = ro_b70_sc * np.exp(ib_sc - ib70_sc)
    # Найдем растворимость метана в растворе
    # Сперва определим давление насыщенных паров для чистой воды
    eps = 1 - t_K/647.096
    p_sigma = 22.064 * np.exp(647.096/t_K * (-7.85951783 * eps + 1.84408259 * eps ** 1.5 - 11.7866497 * eps ** 3 +
                                             22.6807411 * eps ** 3.5 - 15.9619719 * eps ** 4 + 1.80122502 * eps ** 7.5))
    # Определим коэффициенты растворимости метана
    a = -0.004462 * (t_C / 100) - 0.06763
    b = -0.03602 * (t_C / 100) ** 2 + 0.18917 * (t_C / 100) + 0.97242
    c = (0.6855 * (t_C / 100) ** 2 - 3.1992 * (t_C / 100) - 3.7968) / (0.07711 * (t_C / 100) ** 2 + 0.2229 * (t_C / 100)
                                                                       + 1)
    # Растворимость метана в чистой воде
    m_ch4_w = np.exp(a * (np.log(p_MPaa - p_sigma))**2 + b * np.log(p_MPaa - p_sigma) + c)
    # Далее найдем коэффициенты взаимодействия
    lyambda = -0.80898 + 1.0827 * 10 ** (-3) * t_C + 183.85 / t_C + 3.924 * 10 ** (-4) * p_MPaa - 1.97 * 10 ** (-6) *\
        p_MPaa ** 2
    dzeta = -3.89 * 10 ** (-3)
    # Растворимость метана в растворе
    # нужно отметить, что в обозначениях было сложно разобраться, для понимания этой формулы лучше читать статью
    m_ch4_b = m_ch4_w * np.exp(-2 * lyambda * m - dzeta*m**2)
    # Производные необходимые для расчета формулы
    derivative1 = 7.6985890 * 10 ** (-2) - 5.0253331 * 10 ** (-5) * t_K - 30.092013 / t_K + 4.8468502 * 10 ** 3 /\
        t_K ** 2
    derivative2 = 3.924 * 10 ** (-4) - 2 * 1.97 * 10 ** (-6) * p_MPaa
    # Парциальный объем метана в растворе
    v_ch4_b = 8.314467 * t_K * (derivative1 + 2 * m * derivative2)
    # Удельный объем раствора без метана
    v_b0 = 1 / ro_b
    v_b0_sc = 1 / ro_b_sc
    bw = ((1000 + m * 58.4428) * v_b0 + m_ch4_b * v_ch4_b)/((1000 + m * 58.4428) * v_b0_sc)
    return bw


def unf_gwr_brine_Spivey_m3m3(s_ppm, z):  # TODO check
    """
        Modified Spivey et al. correlation for solution gas-water ratio of methane in brine(2009)

    :param s_ppm: salinity, ppm
    :param z: z-factor
    :return: GWR, m3/m3

    корреляция позволяет найти газосодержание метана в соленой воде

    ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.

    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    """

    s = s_ppm / 1000000
    m = 1000 * s / (58.4428 * (1 - s))
    t_C_sc = 20
    t_K_sc = 293.15
    p_MPaa_sc = 0.1013
    # Первым шагом вычисляется плотность чистой воды при давлении 70 MPa и температуре
    ro_w70_sc = (-0.127213 * (t_C_sc/100)**2 + 0.645486 * (t_C_sc/100) + 1.03265)/(-0.070291 * (t_C_sc/100)**2 +
                                                                                   0.639589 * (t_C_sc/100) + 1)
    # Температурные коэффициенты сжимаемости чистой воды
    ew_sc = (4.221 * (t_C_sc / 100) ** 2 - 3.478 * (t_C_sc / 100) + 6.221) / (0.5182 * (t_C_sc / 100) ** 2 - 0.4405 *
                                                                              (t_C_sc / 100) + 1)
    fw_sc = (-11.403 * (t_C_sc / 100) ** 2 + 29.932 * (t_C_sc / 100) + 27.952) / (0.20684 * (t_C_sc / 100) ** 2 +
                                                                                  0.3768 * (t_C_sc / 100) + 1)
    # Температурные коэффициенты плотности раствора
    d_m1 = -1.1149 * 10 ** (-4) * (t_C_sc/100)**2 + 1.7105 * 10 ** (-4) * (t_C_sc/100) - 4.3766 * 10 ** (-4)
    d_m2 = (-8.878 * 10 ** (-4) * (t_C_sc/100)**2 - 1.388 * 10 ** (-4) - 2.96318 * 10 ** (-3))/(0.51103 * (t_C_sc /
                                                                                                           100) + 1)
    d_m3 = (2.1466 * 10 ** (-3) * (t_C_sc / 100) ** 2 + 1.2427 * 10 ** (-2) * (t_C_sc / 100) + 4.2648 * 10 ** (-2)) / \
           (-8.1009 * 10 ** (-2) * (t_C_sc / 100) ** 2 + 0.525417 * (t_C_sc / 100) + 1)
    d_m4 = 2.356 * 10 ** (-4) * (t_C_sc/100)**2 - 3.636 * 10 ** (-4) * (t_C_sc/100) - 2.278 * 10 ** (-4)
    # Плотность раствора воды с хлоридом натрия при давлении 70 MPa и температуре
    ro_b70_sc = ro_w70_sc + d_m1 * m ** 2 + d_m2 * m ** 1.5 + d_m3 * m + d_m4 * m ** 0.5
    # Температурные коэффициенты сжимаемости раствора
    eb_sc = ew_sc + 0.1249
    f_m1 = (-0.617 * (t_C_sc / 100) ** 2 - 0.747 * (t_C_sc / 100) - 0.4339) / (10.26 * (t_C_sc / 100) + 1)
    f_m2 = (9.917 * (t_C_sc / 100) + 5.1128) / (3.892 * (t_C_sc / 100) + 1)
    f_m3 = 0.0365 * (t_C_sc / 100) ** 2 - 0.0369 * (t_C_sc / 100)
    fb_sc = fw_sc + f_m1 * m ** 1.5 + f_m2 * m + f_m3 * m ** 0.5
    ib70_sc = np.log(abs(eb_sc + fb_sc))/eb_sc
    ib_sc = np.log(abs(eb_sc * p_MPaa_sc / 70 + fb_sc)) / eb_sc
    # Плотность раствора при T, P
    ro_b_sc = ro_b70_sc * np.exp(ib_sc - ib70_sc)
    # Найдем растворимость метана в растворе
    # Сперва определим давление насыщенных паров для чистой воды
    eps = 1 - t_K_sc/647.096
    p_sigma = 22.064 * np.exp(647.096/t_K_sc * (-7.85951783 * eps + 1.84408259 * eps ** 1.5 - 11.7866497 *
                                                eps ** 3 + 22.6807411 * eps ** 3.5 - 15.9619719 * eps ** 4 +
                                                1.80122502 * eps ** 7.5))
    # Определим коэффициенты растворимости метана
    a = -0.004462 * (t_C_sc / 100) - 0.06763
    b = -0.03602 * (t_C_sc / 100) ** 2 + 0.18917 * (t_C_sc / 100) + 0.97242
    c = (0.6855 * (t_C_sc / 100) ** 2 - 3.1992 * (t_C_sc / 100) - 3.7968) / (0.07711 * (t_C_sc / 100) ** 2 + 0.2229 *
                                                                             (t_C_sc / 100) + 1)
    # Растворимость метана в чистой воде
    m_ch4_w = np.exp(a * (np.log(p_MPaa_sc - p_sigma))**2 + b * np.log(p_MPaa_sc - p_sigma) + c)
    # Далее найдем коэффициенты взаимодействия
    lyambda = -0.80898 + 1.0827 * 10 ** (-3) * t_C_sc + 183.85 / t_C_sc + 3.924 * 10 ** (-4) * p_MPaa_sc - 1.97 * 10 **\
        (-6) * p_MPaa_sc ** 2
    dzeta = -3.89 * 10 ** (-3)
    # Растворимость метана в растворе
    # нужно отметить, что в обозначениях было сложно разобраться, для понимания этой формулы лучше читать статью
    m_ch4_b = m_ch4_w * np.exp(-2 * lyambda * m - dzeta*m**2)
    # Удельный объем раствора без метана
    v_b0_sc = 1 / ro_b_sc
    # Молярный объем метана в газовой фазе
    vm_ch4_g_sc = z * 8.314467 * t_K_sc / p_MPaa_sc
    # Найдем GWR
    gwr = m_ch4_b * vm_ch4_g_sc / ((1000 + m * 58.4428) * v_b0_sc)
    return gwr


def unf_viscosity_brine_McCain_cp(t_K, p_MPaa, s_ppm):  #TODO check
    """
        McCain correlation for brine(water) viscosity

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :param s_ppm: salinity, ppm
    :return: viscosity, cP

    ref 1 McCain, W.D. Jr.: McCain, W.D. Jr. 1990. The Properties of Petroleum Fluids, second edition. Tulsa,
    Oklahoma: PennWell Books.

    ref 2 https://petrowiki.org/Produced_water_properties#cite_note-r1-1
    """

    wpTDS = s_ppm / 10000

    a = 109.574 - 8.40564 * wpTDS + 0.313314 * wpTDS ** 2 + 0.00872213 * wpTDS ** 3
    b = -1.12166 + 0.0263951 * wpTDS - 0.000679461 * wpTDS ** 2 - 5.47119 * 10 ** (-5) * wpTDS ** 3 + 1.55586 * 10 ** (
        -6) * wpTDS ** 4

    visc = a * (1.8 * t_K - 460) ** b
    p_psi = uc.bar2psi(p_MPaa * 10)
    viscosity_cp = visc * (0.9994 + 4.0295 * 10 ** (-5) * p_psi + 3.1062 * 10 ** (-9) * p_psi ** 2)
    return viscosity_cp


def unf_viscosity_brine_MaoDuan_cP(t_K, p_MPaa, s_ppm):  #TODO check
    """
        Mao-Duan correlation for brine(water) viscosity (2009)

    :param t_K: temperature, K
    :param p_MPaa: pressure, MPaa
    :param s_ppm: salinity, ppm
    :return: viscosity, cP

    корреляция позволяет найти вязкость соленой воды

    ref 1 Mao, S., and Duan, Z. “The Viscosity of Aqueous Alkali-Chloride Solutions up to
    623 K, 1,000 bar, and High Ionic Strength.” International Journal of Thermophysics.
    Vol. 30 (2009) 1,510–1,523.

    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    """
    t_C = uc.k2c(t_K)
    s = s_ppm / 1000000
    m = 1000 * s / (58.4428 * (1 - s))
    # Первым шагом вычисляется плотность чистой воды при давлении 70 MPa и температуре
    ro_w70 = (-0.127213 * (t_C/100)**2 + 0.645486 * (t_C/100) + 1.03265)/(-0.070291 * (t_C/100)**2 +
                                                                          0.639589 * (t_C/100) + 1)
    # Температурные коэффициенты сжимаемости чистой воды
    ew = (4.221 * (t_C / 100)**2 - 3.478 * (t_C/100) + 6.221)/(0.5182 * (t_C/100)**2 - 0.4405 * (t_C/100) + 1)
    fw = (-11.403 * (t_C / 100) ** 2 + 29.932 * (t_C / 100) + 27.952) / (0.20684 * (t_C / 100) ** 2 +
                                                                         0.3768 * (t_C / 100) + 1)
    iw70 = np.log(abs(ew + fw))/ew
    iw = np.log(abs(ew*(p_MPaa/70) + fw))/ew
    # Плотность чистой воды при T, P
    ro_w = ro_w70 * np.exp(iw - iw70)
    # Вязкость чистой воды
    viscosity = np.exp(0.28853170 * 10 ** 7 * t_K ** (-2) - 0.11072577 * 10 ** 5 * t_K ** (-1) - 0.90834095 * 10 +
                       0.30925651 * 10 ** (-1) * t_K - 0.27407100 * 10 ** (-4) * t_K ** 2 + ro_w *
                       (-0.19298951 * 10 ** 7 * t_K ** (-2) + 0.56216046 * 10 ** 4 * t_K ** (-1) + 0.13827250 *
                        10 ** 2 - 0.47609523 * 10 ** (-1) * t_K + 0.35545041 * 10 ** (-4) * t_K ** 2))
    # Коэффициенты, зависящие от температуры
    a = -0.213119213 + 0.13651589 * 10 ** (-2) * t_K - 0.12191756 * 10 ** (-5) * t_K ** 2
    b = -0.69161945 * 10 ** (-1) - 0.27292263 * 10 ** (-3) * t_K + 0.20852448 * 10 ** (-6) * t_K ** 2
    c = -0.25988855 * 10 ** (-2) + 0.77989227 * 10 ** (-5) * t_K
    # Относительная вязкость
    viscosity_rel = np.exp(a * m + b * m ** 2 + c * m ** 3)
    # Вязкость соленой воды
    viscosity_cP = 1000 * viscosity * viscosity_rel
    return viscosity_cP


# TODO добавить для термодинамических свойств воды учет давления, температуры, солености
"""
Следующие свойства взяты при давлении 1 бар для дистилированной воды
за границей применимости 5 < T < 95 C произведена линейная экстраполяция
"""


def unf_heat_capacity_water_IAPWS_JkgC(t_c):  # TODO заменить
    """
        Теплоемкость дистилированной воды в диапазоне 5 < T < 95 C при 1 бар
    выше диапазона - линейная экстраполяция

    :param t_c: температура в С
    :return: теплоемкость в Дж / кг С

    ref1 https://syeilendrapramuditya.wordpress.com/2011/08/20/water-thermodynamic-properties/
    """
    def cor_in_range_5_95(t_c):
        return (4.214 - 2.286*10**(-3) * t_c +
               4.991 * 10**(-5) * t_c ** 2 -
               4.519 * 10**(-7) * t_c ** 3 +
               1.857 * 10**(-9) * t_c ** 4)
    if t_c < 95:
        return cor_in_range_5_95(t_c)* 1000
    else:
        return (cor_in_range_5_95(95) +
                (cor_in_range_5_95(95) - cor_in_range_5_95(85)) / 10 *
                (t_c - 95)) * 1000


def unf_thermal_conductivity_water_IAPWS_WmC(t_c):  # TODO заменить
    """
        Теплопроводность дистилированной воды в диапазоне 5 < T < 95 C при 1 бар
    выше диапазона - линейная экстраполяция

    :param t_c: температура в С
    :return: теплопроводность в Вт / м С

    ref1 https://syeilendrapramuditya.wordpress.com/2011/08/20/water-thermodynamic-properties/
    """
    def cor_in_range_5_95(t_c):
        return (0.5636 + 1.946 * 10**(-3) * t_c -
                8.151 * 10** (-6) *t_c **2)
    if t_c < 95:
        return cor_in_range_5_95(t_c)
    else:
        return (cor_in_range_5_95(95) +
                (cor_in_range_5_95(95) - cor_in_range_5_95(85)) / 10 *
                (t_c - 95))


def unf_thermal_expansion_coefficient_water_IAPWS_1C(t_c):  # TODO заменить
    """
        Коэффициент термического расширения дистилированной воды в диапазоне 5 < T < 95 C при 1 бар
    выше диапазона - линейная экстраполяция

    :param t_c: температура в С
    :return: Коэффициент термического расширения в 1 / с

    ref1 https://syeilendrapramuditya.wordpress.com/2011/08/20/water-thermodynamic-properties/
    """
    return 7.957 * 10**(-5) + 7.315 * 10**(-6) * t_c


# Корреляцияные зависимости для нефтяных систем


def unf_surface_tension_go_Abdul_Majeed_Nm(t_K, gamma_oil, rs_m3m3):
    """
        Корреляция Абдул-Маджида (2000 г.) для поверхностного натяжения нефти, насыщенной газом

    :param t_K: температура, градусы Кельвина
    :param gamma_oil: относительная плотность нефти
    :param rs_m3m3: газосодержание, м3 / м3
    :return: поверхностное натяжение на границе нефть-газ, Н / м

        Источник: Справочник инженера-нефтяника. Том 1. Введение в нефтяной инжиниринг. Газпром Нефть
    """
    t_C = uc.k2c(t_K)
    surface_tension_dead_oil_dynes_cm = (1.17013 - 1.694 * 10 ** (-3) * (1.8 * t_C + 32)) * (
                38.085 - 0.259 * (141.5 / gamma_oil - 131.5))
    relative_surface_tension_go_od = (0.056379 + 0.94362 * np.exp(-21.6128 * 10 ** (-3) * rs_m3m3))
    surface_tension_dynes_cm = surface_tension_dead_oil_dynes_cm * relative_surface_tension_go_od
    return uc.dyncm2nm(surface_tension_dynes_cm)


def unf_surface_tension_go_Baker_Swerdloff_Nm(t_K, gamma_oil, p_MPa):
    """
        Корреляция Бэйкера и Свердлоффа (1955 г.) для поверхностного натяжения нефти, насыщенной газом

    :param t_K: температура, градусы Кельвина
    :param gamma_oil: относительная плотность нефти
    :param p_MPa: давление, МПа
    :return: поверхностное натяжения на границе нефть-газ в Н /м

    Источник: Справочник инженера-нефтяника. Том 1. Введение в нефтяной инжиниринг. Газпром Нефть
    """
    p_bar = uc.convert_pressure(p_MPa, 'MPa', 'bar')
    t_C = uc.k2c(t_K)
    surface_tension_dead_oil_20_c_dynes_cm = 39 - 0.2571 * (141.5 / gamma_oil - 131.5 )
    surface_tension_dead_oil_38_c_dynes_cm = 37.5 - 0.2571 * (141.5 / gamma_oil - 131.5 )
    if t_C <= 20:
        surface_tension_dead_oil_dynes_cm = surface_tension_dead_oil_20_c_dynes_cm
    elif t_C >= 38:
        surface_tension_dead_oil_dynes_cm = surface_tension_dead_oil_38_c_dynes_cm
    else:
        surface_tension_dead_oil_dynes_cm = (surface_tension_dead_oil_20_c_dynes_cm -
                                             (t_C - 20) * (surface_tension_dead_oil_20_c_dynes_cm -
                                                           surface_tension_dead_oil_38_c_dynes_cm) / 18)
    surface_tension_go_Baker_Swerdloff_dynes_cm = (surface_tension_dead_oil_dynes_cm *
                                                   np.exp(-125.1763 * 10 ** (-4) * p_bar))

    return uc.dyncm2nm(surface_tension_go_Baker_Swerdloff_dynes_cm)


def unf_surface_tension_gw_Sutton_Nm(rho_water_kgm3, rho_gas_kgm3, t_c):  # TODO поправка на соленость добавить
    """
        Корреляция Саттона для поверхностного натяжения на границе вода-газ

    :param rho_water_kgm3: плотность воды кг / м3
    :param rho_gas_kgm3:  плотность газа кг / м3
    :param t_c: температура в С
    :return: поверхностное натяжение на границе вода-газ, Н / м

    ref 1 Pereira L. et al. Interfacial tension of reservoir fluids: an integrated experimental
    and modelling investigation : дис. – Heriot-Watt University, 2016. page 41

    ref2 Ling K. et al. A new correlation to calculate oil-water interfacial tension
    //SPE Kuwait International Petroleum Conference and Exhibition. – Society of Petroleum Engineers, 2012.

    """
    rho_water_gcm3 = rho_water_kgm3 / 1000
    rho_gas_gcm3 = rho_gas_kgm3 / 1000
    t_r = uc.c2r(t_c)
    surface_tension_dyncm = ((1.53988 * (rho_water_gcm3 - rho_gas_gcm3) + 2.08339) /
            ((t_r /302.881) ** (0.821976 - 0.00183785 * t_r +
            0.00000134016 * t_r ** 2))) ** 3.6667
    return uc.dyncm2nm(surface_tension_dyncm)


def unf_surface_tension_Baker_Sverdloff_vba_nm(p_atma, t_C, gamma_o_):
    t_F = t_C * 1.8 + 32
    P_psia = p_atma / 0.068046
    P_MPa = p_atma / 10
    ST68 = 39 - 0.2571 * (141.5 / gamma_o_ - 131.5)
    ST100 = 37.5 - 0.2571 * (141.5 / gamma_o_ - 131.5)
    if t_F < 68:
        STo = ST68
    else:
        Tst = t_F
        if t_F > 100:
            Tst = 100
        STo = (68 - (((Tst - 68) * (ST68 - ST100)) / 32)) * np.exp(-0.00086306 * P_psia)

    STw74 = (75 - (1.108 * (P_psia) ** 0.349))
    STw280 = (53 - (0.1048 * (P_psia) ** 0.637))

    if t_F < 74:
        STw = STw74
    else:
        Tstw = t_F
        if t_F > 280:
            Tstw = 280
        STw = STw74 - (((Tstw - 74) * (STw74 - STw280)) / 206)
    STw = 10 ** (-(1.19 + 0.01 * P_MPa)) * 1000
    ST_oilgas_dyncm_ = STo
    ST_watgas_dyncm_ = STw
    return [uc.dyncm2nm(ST_oilgas_dyncm_), uc.dyncm2nm(ST_watgas_dyncm_)]

import scipy.optimize as sp  # модуль для решения уравения

def coef_mt(t_k, rho_deadoil_kgm3, gamma_gas):
    return 1 + 0.029 * (t_k - 293) * (rho_deadoil_kgm3 * gamma_gas * 10 ** (-3) - 0.7966)


def coef_dt(t_k, rho_deadoil_kgm3, gamma_gas):
    return 10 ** (-3) * rho_deadoil_kgm3 * gamma_gas * (4.5 - 0.00305 * (t_k - 293)) - 4.785


def coef_rp(p_mpa, pb_mpa):
    return (1 + np.log10(p_mpa)) / (1 + np.log10(pb_mpa)) - 1


def unf_calc_gas_liberated_and_dissolved(t_k, rho_deadoil_kgm3, gamma_oil, gamma_gas, p_mpa, pb_mpa, rsb_m3m3, return_m3m3=True):
    rsb_m3t = rsb_m3m3 / gamma_oil
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
            return 0, rsb_m3t * mt * rho_deadoil_kgm3 / 1000
        else:
            return 0, rsb_m3t * mt

    if return_m3m3:
        return gas_liberated_m3t * rho_deadoil_kgm3 / 1000, gas_dissolved_m3t * rho_deadoil_kgm3 / 1000
        # return gas_liberated_m3t, gas_dissolved_m3t
    else:
        return gas_liberated_m3t, gas_dissolved_m3t


def calc_gas_for_fsolve(rsb_m3m3_real, t_k, rho_deadoil_kgm3, gamma_oil, gamma_gas, p_mpa, pb_mpa, rsb_m3m3, return_m3m3=True):
    gas_dissolved_m3m3 = unf_calc_gas_liberated_and_dissolved(t_k, rho_deadoil_kgm3,  gamma_oil, gamma_gas, pb_mpa, pb_mpa, rsb_m3m3_real, return_m3m3)[1]
    return (gas_dissolved_m3m3 - rsb_m3m3) ** 2


def calc_gas_with_fsolve(t_k, rho_deadoil_kgm3, gamma_oil,  gamma_gas, p_mpa, pb_mpa, rsb_m3m3, return_m3m3=True):
    method = 'excitingmixing'
    answer = sp.root(calc_gas_for_fsolve, rsb_m3m3, args=(t_k, rho_deadoil_kgm3,gamma_oil, gamma_gas,
                                                          pb_mpa, pb_mpa, rsb_m3m3, return_m3m3), tol=0.01)
    answer = answer.x[0]
    return unf_calc_gas_liberated_and_dissolved(t_k, rho_deadoil_kgm3,gamma_oil, gamma_gas, p_mpa, pb_mpa, answer, return_m3m3)

def calc_pb(pb_mpa, rho_deadoil_kgm3, rsb_m3m3, tres_k, t_k):
    gi = rsb_m3m3 * 10**3 * 273 / 293 / rho_deadoil_kgm3
    pbt_mpa = pb_mpa - (tres_k - t_k) / (9.157 + 701.8 / (gi * (0.4 - 0.8* 0.08)))
    return pbt_mpa