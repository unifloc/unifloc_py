# -*- coding: utf-8 -*-
"""
Created on Sat May  5 13:59:06 2018

@author: Rinat Khabibullin
         Alexey Vodopyan
"""
import numpy as np
import unittest
import uscripts.uconst as uc
import scipy.optimize as opt

# uPVT свойства для нефти


def unf_pb_Standing_MPaa(rsb_m3m3, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    bubble point pressure calculation according to Standing (1947) correlation

    ref1 "A Pressure-Volume-Temperature Correlation for Mixtures of California Oil and Gases",
    M.B. Standing, Drill. & Prod. Prac., API, 1947.

    ref2  "Стандарт компании Юкос. Физические свойства нефти. Методы расчета." Афанасьев В.Ю., Хасанов М.М. и др. 2002 г

    return bubble point pressure abs in MPa
    rsb_m3m3,       solution ration at bubble point, must be given, m3/m3
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
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
        pb_MPaa = (pb_MPaa - 0.101325) * rsb_old / min_rsb + 0.101325
    return pb_MPaa


def unf_pb_Valko_MPaa(rsb_m3m3, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    bubble point pressure calculation according to Valko McCain (2002) correlation

    ref SPE  "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities"
    W. D. McCain Jr.,P.P. Valko, 
    
    return bubble point pressure abs in MPa
    rsb_m3m3,       solution ration at bubble point, must be given, m3/m3
    gamma_oil=0.86, specific oil density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
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


def unf_pb_AlMarhoun_MPaa(rsb_m3m3, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    bubble point pressure calculation according to Al-Marhoun (1985) correlation

    Middle-East oil

    ref1 PYT Correlations for Middle East Crude Oils, Muhammad All AI.Marhoun, Journal of Petroleum Technology. May 1988

    return bubble point pressure abs in MPa
    rsb_m3m3,       solution ration at bubble point, must be given, m3/m3
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
    """
    pass


def unf_rs_Standing_m3m3(p_MPaa, pb_MPaa=0, rsb_m3m3=0, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    Gas-oil ratio calculation inverse of Standing (1947) correlation for bubble point pressure

    ref1 "A Pressure-Volume-Temperature Correlation for Mixtures of California Oil and Gases",
    M.B. Standing, Drill. & Prod. Prac., API, 1947.

    ref2  "Стандарт компании Юкос. Физические свойства нефти. Методы расчета." Афанасьев В.Ю., Хасанов М.М. и др. 2002 г
    может считать в случае если нет давления насыщения и газосодержания при давлении насыщения, корреляция не точная
    return gas-oil ratio in m3/m3
    p_MPaa,         pressure, MPa
    gamma_oil=0.86, specific oil density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
    pb_MPaa=0,      buble point pressure, MPa
    rsb_m3m3=0,     gas-oil ratio at the bubble point pressure, m3/m3
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

    ref1 "Correlation of Black Oil Properties at Pressures Below Bubblepoint Pressure—A New Approach",
    J. VELARDE, T.A. BLASINGAME Texas A&M University, W.D. MCCAIN, JR. S.A. Holditch & Associates, Inc 1999

    return gas-oil ratio in m3/m3
    p_MPaa,         pressure, MPaa
    gamma_oil=0.86, specific oil density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
    pb_MPaa=10,     buble point pressure, MPaa
    rsb_m3m3=100,   gas-oil ratio at the bublepoint pressure, m3/m3
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


def unf_rsb_Mccain_m3m3(rsp_m3m3,gamma_oil, psp_MPaa=0, tsp_K=0):
    """
    Solution Gas-oil ratio at bubble point pressure calculation according to McCain (2002) correlation 
    taking into account the gas losses at separator and stock tank
    
    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002
    
    return solution gas-oil ratio at bubble point pressure, rsb in m3/m3
    часто условия в сепараторе неизвестны, может считать и без них по приблизительной формуле
    rsp_m3m3,             separator producing gas-oil ratio, m3m3
    gamma_oil,            specific oil density(by water)
    psp_MPaa = 0,         pressure in separator, MPaa
    tsp_K = 0,            temperature in separator, K
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
    
    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002
    
    return weighted-average specific gravities of surface gases
    часто условия в сепараторе неизвестны, может считать и без них по приблизительной формуле
    rsp_m3m3,             separator producing gas-oil ratio, m3m3
    rst_m3m3,             stock-tank producing gas-oil ratio, m3m3
    gamma_gassp,          separator gas specific gravity
    gamma_oil,            specific oil density(by water)
    psp_MPaa = 0,         pressure in separator, MPaa
    tsp_K = 0,            temperature in separator, K
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
    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    
    return formation volume factor bo,m3m3
    density_oilsto_kgm3,             density of stock-tank oil, kgm3
    rs_m3m3,                         solution gas-oil ratio, m3m3
    density_oil_kgm3,                Oil density at reservoir conditions, kgm3
    gamma_gas,                       specific gas  density(by air)
    """
    density_oilsto_lbft3 = uc.kgm3_2_lbft3(density_oilsto_kgm3)
    density_oil_lbft3 = uc.kgm3_2_lbft3(density_oil_kgm3)
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    bo = (density_oilsto_lbft3 + 0.01357 * rs_scfstb * gamma_gas) / density_oil_lbft3
    return bo


def unf_fvf_VB_m3m3_above(bob, cofb_1MPa, pb_MPaa, p_MPaa):
    """  
    Oil Formation Volume Factor according equation for pressure above bubble point pressure
    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    
    ! Actually, this correlation is belonged ro Vasquez & Beggs (1980). In some sources is
    noted that this is Standing correlation.
    ref2 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA

    return formation volume factor bo,m3m3
    bob,           formation volume factor at bubble point pressure, m3m3
    cofb_1MPa,     weighted-average oil compressibility from bubblepoint pressure to a higher pressure of interest,1/MPa
    pb_MPaa,       bubble point pressure, MPa
    p_MPaa,        pressure, MPa
    """
    if p_MPaa <= pb_MPaa:
        bo = bob
    else:
        pb_psia = uc.Pa2psi(pb_MPaa * 10 ** 6)
        p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
        cofb_1psi = uc.compr_1pa_2_1psi(cofb_1MPa / 10 ** 6)
        bo = bob * np.exp(cofb_1psi * (pb_psia - p_psia))
    return bo


def unf_compressibility_oil_VB_1Mpa(rs_m3m3, t_K, gamma_oil, p_MPaa, gamma_gas=0.6):
    """  
    oil compressibility according to Vasquez & Beggs (1980) correlation 
    ref1 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA

    return coefficient of isothermal compressibility co_1MPa,1/MPa
    rs_m3m3,             solution gas-oil ratio, m3m3
    t_K,                 temperature, K
    gamma_oil,           specific oil density (by water)
    p_MPaa,              pressure, MPaa
    gamma_gas=0.6,       specific gas density (by air)
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
    ref1 Volumetric and phase behavior of oil field hydrocarbon systems / M.B. Standing Standing, M. B. 1981

    return formation volume factor at bubble point pressure bo,m3m3
    rs_m3m3,             solution gas-oil ratio, m3m3
    gamma_gas,       specific gas density (by air)
    gamma_oil,      specific oil density (by water)
    t_K,                 temperature, K
    """
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    t_F = uc.k2f(t_K)
    bo = 0.972 + 1.47 * 10 ** (-4) * (rs_scfstb * (gamma_gas / gamma_oil) ** 0.5 + 1.25 * t_F) ** 1.175
    return bo


def unf_density_oil_Mccain(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, gamma_gas, t_K, gamma_oil, gamma_gassp = 0):
    """  
    Oil density according Standing, M.B., 1977; Witte, T.W., Jr., 1987; and McCain, W.D., Jr. and Hill, N.C.,
    1995 correlation.
    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    return oil density,kg/m3
    p_MPaa,              pressure, MPaa
    pb_MPaa,             bubble point pressure, MPaa
    co_1MPa,             coefficient of isothermal compressibility, 1/MPa
    rs_m3m3,             solution gas-oil ratio, m3m3
    gamma_gas,           specific gas density (by air)
    t_K,                 temperature, K
    gamma_oil,           specific oil density (by water)
    gamma_gassp = 0,     specific gas density in separator(by air)
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
    ref1 book Brill 2006, Production Optimization Using Nodal Analysis

    return oil density,kg/m3
    p_MPaa,              pressure, MPaa
    pb_MPaa,             bubble point pressure, MPaa
    co_1MPa,             coefficient of isothermal compressibility, 1/MPa
    rs_m3m3,             solution gas-oil ratio, m3m3
    b0_m3m3,             oil formation volume factor, m3m3
    gamma_gas,           specific gas density (by air)
    gamma_oil,           specific oil density (by water)
    """
    po = (1000 * gamma_oil + 1.224 * gamma_gas * rs_m3m3) / bo_m3m3
    if p_MPaa > pb_MPaa:
        po = po * np.exp(co_1MPa * (p_MPaa - pb_MPaa))
    return po


def unf_deadoilviscosity_Beggs_cP(gamma_oil, t_K):
    """
    Correlation for dead oil viscosity
    ref1 Beggs, H.D. and Robinson, J.R. “Estimating the Viscosity of Crude Oil Systems.”
    Journal of Petroleum Technology. Vol. 27, No. 9 (1975)

    return dead oil viscosity,cP
    gamma_oil,           specific oil density (by water)
    t_K,                 temperature, K
    """
    api = uc.gamma_oil2api(gamma_oil)
    t_F = uc.k2f(t_K)
    c = 10 ** (3.0324 - 0.02023 * api) * t_F ** (-1.163)
    viscosity_cP = 10 ** c - 1
    return viscosity_cP


def unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, rs_m3m3):
    """
    Correlation for oil viscosity for pressure below bubble point
    ref1 Beggs, H.D. and Robinson, J.R. “Estimating the Viscosity of Crude Oil Systems.”
    Journal of Petroleum Technology. Vol. 27, No. 9 (1975)

    return oil viscosity,cP
    deadoilviscosity_cP,           dead oil viscosity,cP
    rs_m3m3,                       solution gas-oil ratio, m3m3
    """
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    a = 10.715 * (rs_scfstb + 100) ** (-0.515)
    b = 5.44 * (rs_scfstb + 150) ** (-0.338)
    viscosity_cP = a * deadoilviscosity_cP ** b
    return viscosity_cP


def unf_undersaturatedoilviscosity_VB_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP):
    """"
    viscosity correlation for pressure above bubble point
    ref2 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA

    return oil viscosity,cP
    p_MPaa,                      pressure, MPaa
    pb_MPaa,                     bubble point pressure, MPaa
    bubblepointviscosity_cP,     oil viscosity at bubble point pressure, cP
    """
    p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
    pb_psia = pb_MPaa * 145.03773800721814
    m = 2.6 * p_psia ** 1.187 * np.exp(-11.513 - 8.98 * 10 ** (-5) * p_psia)
    viscosity_cP = bubblepointviscosity_cP * (p_psia / pb_psia) ** m
    return viscosity_cP


def unf_undersaturatedoilviscosity_Petrovsky_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP):
    """
    viscosity correlation for pressure above bubble point
    ref 1 Petrosky, G.E. and Farshad, F.F. “Viscosity Correlations for Gulf of Mexico Crude
    Oils.” Paper SPE 29468. Presented at the SPE Production Operations Symposium,
    Oklahoma City (1995)

    return oil viscosity,cP
    p_MPaa,                      pressure, MPaa
    pb_MPaa,                     bubble point pressure, MPaa
    bubblepointviscosity_cP,     oil viscosity at bubble point pressure, cP
    """
    A = -1.0146 + 1.3322 * np.log(bubblepointviscosity_cP) - 0.4876 * np.log(
        bubblepointviscosity_cP) ** 2 - 1.15036 * np.log(bubblepointviscosity_cP) ** 3
    p_psia = uc.Pa2psi(p_MPaa * 10 ** 6)
    pb_psia = uc.Pa2psi(pb_MPaa * 10 ** 6)
    viscosity_cP = bubblepointviscosity_cP + 1.3449 * 10 ** (-3) * (p_psia - pb_psia) * 10 ** A
    return viscosity_cP


def unf_oil_viscosity_Beggs_VB_cP(deadoilviscosity_cP, rs_m3m3, p_MPaa, pb_MPaa):
    """
    function for calculating the viscosity at any pressure

    return oil viscosity,cP
    deadoilviscosity_cP,           dead oil viscosity,cP
    rs_m3m3,                       solution gas-oil ratio, m3m3
    p_MPaa,                        pressure, MPaa
    pb_MPaa,                       bubble point pressure, MPaa
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
    ref Generalized Pressure-Volume-Temperature Correlations, Glaso, 1980
    :param rs_m3m3: gas-oil ratio in m3/m3
    :param t_K: temperature in K
    :param gamma_oil: oil density (by water)
    :param gamma_gas: gas density (by air)
    :return: bubble point pressure im MPa abs
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
    ref Generalized Pressure-Volume-Temperature Correlations, Glaso, 1980
    :param rs_m3m3: gas-oil ratio in m3/m3
    :param t_K: temperature in K
    :param gamma_oil: oil density (by water)
    :param gamma_gas: gas density (by air)
    :return: formation volume factor at bubble point pressure in m3/m3
    """
    t_F = uc.k2f(t_K)
    rs_scfstb = uc.m3m3_2_scfstb(rs_m3m3)
    bob = rs_scfstb * (gamma_gas / gamma_oil) ** 0.526 + 0.968 * t_F
    bob = 10 ** (-6.58511 + 2.91329 * np.log10(bob) - 0.27683 * np.log10(bob) ** 2) + 1
    return bob


def unf_fvf_Glaso_m3m3_below(rs_m3m3, t_K, gamma_oil, gamma_gas, p_MPaa):
    """
    Glaso correlation(1980) for total formation volume factor below bubble point pressure
    ref Generalized Pressure-Volume-Temperature Correlations, Glaso, 1980
    :param rs_m3m3: gas-oil ratio in m3/m3
    :param t_K: temperature in K
    :param gamma_oil: oil density (by water)
    :param gamma_gas: gas density (by air)
    :param p_MPaa: pressure in MPaa
    :return: total formation volume factor below bubble point pressure in m3/m3
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

"""
В дальнейшем функции для нефти будут дополняться, а пока перейдем к uPVT для газа
"""


# uPVT свойства для газа

def unf_pseudocritical_temperature_K(gamma_gas, y_h2s=0.0, y_co2=0.0, y_n2=0.0):
    """"
    correlation for pseudocritical temperature taking into account the presense of non-hydrocarbon gases
    ref 1 Piper, L.D., McCain, W.D., Jr., and Corredor, J.H. “Compressibility Factors for 
    Naturally Occurring Petroleum Gases.” Gas Reservoir Engineering. Reprint Series. Richardson,
    TX: SPE. Vol. 52 (1999) 186–200

    return pseudocritical temperature, K
    gamma_gas,           specific gas density (by air)
    tc_h2s_K,            critical temperature for hydrogen sulfide, K
    tc_co2_K,            critical temperature for carbon dioxide, K
    tc_n2_K,             critical temperature for nitrogen, K
    pc_h2s_MPaa,         critical pressure for hydrogen sulfide, MPaa
    pc_co2_MPaa,         critical pressure for carbon dioxide, MPaa
    pc_n2_MPaa,          critical pressure for nitrogen, MPaa
    y_h2s,               mole fraction of the hydrogen sulfide
    y_co2,               mole fraction of the carbon dioxide
    y_n2,                mole fraction of the nitrogen
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
    """"
    correlation for pseudocritical pressure taking into account the presense of non-hydrocarbon gases
    ref 1 Piper, L.D., McCain, W.D., Jr., and Corredor, J.H. “Compressibility Factors for
    Naturally Occurring Petroleum Gases.” Gas Reservoir Engineering. Reprint Series. Richardson,
    TX: SPE. Vol. 52 (1999) 186–200

    return pseudocritical pressure, MPa
    gamma_gas,           specific gas density (by air)
    tc_h2s_K,            critical temperature for hydrogen sulfide, K
    tc_co2_K,            critical temperature for carbon dioxide, K
    tc_n2_K,             critical temperature for nitrogen, K
    pc_h2s_MPaa,         critical pressure for hydrogen sulfide, MPaa
    pc_co2_MPaa,         critical pressure for carbon dioxide, MPaa
    pc_n2_MPaa,          critical pressure for nitrogen, MPaa
    y_h2s,               mole fraction of the hydrogen sulfide
    y_co2,               mole fraction of the carbon dioxide
    y_n2,                mole fraction of the nitrogen
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


def unf_zfactor_BrillBeggs(ppr, tpr):
    """
    correlation for z-factor according Beggs & Brill correlation (1977)

    используется для приближения функции дранчука

    Можно использовать при tpr<=2 и ppr<=4
    при tpr <== 1.5 ppr<=10

    return z-factor
    ppr             preudoreduced pressure
    tpr             pseudoreduced temperature
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
    correlation for z-factor
    ref 1 Dranchuk, P.M. and Abou-Kassem, J.H. “Calculation of Z Factors for Natural
    Gases Using Equations of State.” Journal of Canadian Petroleum Technology. (July–September 1975) 34–36.

    range of applicability is (0.2<=ppr<30 and 1.0<tpr<=3.0) and also ppr < 1.0 for 0.7 < tpr < 1.0

    return z-factor
    p_MPaa,                      pressure, MPaa
    t_K,                         temperature, K
    ppc_MPa                      pseudocritical pressure, MPa
    tpc_K                        pseudocritical temperature, K
    """
    ppr = p_MPaa / ppc_MPa
    tpr = t_K / tpc_K
    z0 = 1
    ropr0 = 0.27 * (ppr / (z0 * tpr))

    def f(variables):
        z,ropr = variables
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
    correlation for z-factor
    ref 1 Dranchuk, P.M. and Abou-Kassem, J.H. “Calculation of Z Factors for Natural
    Gases Using Equations of State.” Journal of Canadian Petroleum Technology. (July–September 1975) 34–36.

    range of applicability is (0.2<=ppr<30 and 1.0<tpr<=3.0) and also ppr < 1.0 for 0.7 < tpr < 1.0

    return z-factor
    p_MPaa,                      pressure, MPaa
    t_K,                         temperature, K
    ppc_MPa                      pseudocritical pressure, MPa
    tpc_K                        pseudocritical temperature, K
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


def unf_compressibility_gas_Mattar_1MPa(p_MPaa, t_K, ppc_MPa, tpc_K):
    """
    correlation for gas compressibility
    ref 1 Mattar, L., Brar, G.S., and Aziz, K. 1975. Compressibility of Natural Gases.
    J Can Pet Technol 14 (4): 77. PETSOC-75-04-08

    return for gas compressibility
    p_MPaa,                      pressure, MPaa
    t_K,                         temperature, K
    ppc_MPa                      pseudocritical pressure, MPa
    tpc_K                        pseudocritical temperature, K
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
    ref 1 Lee, A.L., Gonzalez, M.H., and Eakin, B.E. “The Viscosity of Natural Gases.” Journal
    of Petroleum Technology. Vol. 18 (August 1966) 997–1,000.

    return gas viscosity,cP
    t_K,                         temperature, K
    p_MPaa,                      pressure, MPaa
    z                            z-factor
    gamma_gas                    specific gas density (by air)
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

    return formation volume factor for gas bg, m3/m3
    t_K,                            temperature, K
    p_MPaa,                         pressure, MPaa
    z,                              z-factor
    """
    bg = 101.33 * 10**(-3) * t_K * z / (1 * 293.15 * p_MPaa)
    return bg


# uPVT свойства для воды

def unf_density_brine_Spivey_kgm3(t_K, p_MPaa, s_ppm, par=1):
    """
    Modified Spivey et al. correlation for brine(water) density (2009)

    ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.
    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    корреляция позволяет найти плотность соленой воды с растворенным в ней метаном

    return density, kg/m3
    t_K,                            temperature, K
    p_MPaa,                         pressure, MPaa
    s_ppm,                          salinity, ppm
    par = 0,                        parameter, 0 - methane-free brine, 1 - brine containing dissolved methane
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

    ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.
    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    корреляция позволяет найти сжимаемость соленой воды с частично или полностью растворенным в ней метаном

    return density, kg/m3
    t_K,                            temperature, K
    p_MPaa,                         pressure, MPaa
    s_ppm,                          salinity, ppm
    z=1,                            z-factor
    par = 0,                        parameter, 0 - methane-free brine, 1 - brine containing dissolved methane
                                    2 - brine containing partially dissolved methane
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


def unf_fvf_brine_Spivey_m3m3(t_K, p_MPaa, s_ppm):
    """
    Modified Spivey et al. correlation for brine(water) formation volume factor (2009)

    ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.
    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    корреляция позволяет найти объемный коэффициент для соленой воды с учетом растворенного метана

    return density, kg/m3
    t_K,                            temperature, K
    p_MPaa,                         pressure, MPaa
    s_ppm,                          salinity, ppm
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


def unf_gwr_brine_Spivey_m3m3(s_ppm, z):

    """
    Modified Spivey et al. correlation for solution gas-water ratio of methane in brine(2009)

    ref 1 Spivey, J.P., McCain, W.D., Jr., and North, R. “Estimating Density, Formation
    Volume Factor, Compressibility, Methane Solubility, and Viscosity for Oilfield
    Brines at Temperatures From 0 to 275°C, Pressures to 200 MPa, and Salinities to
    5.7 mole/kg.” Journal of Canadian Petroleum Technology. Vol. 43, No. 7 (July 2004)
    52–61.
    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    корреляция позволяет найти газосодержание метана в соленой воде

    return GWR, m3/m3
    s_ppm,                          salinity, ppm
    z,                              z-factor
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


def unf_viscosity_brine_MaoDuan_cP(t_K, p_MPaa, s_ppm):
    """
    Mao-Duan correlation for brine(water) viscosity (2009)

    ref 1 Mao, S., and Duan, Z. “The Viscosity of Aqueous Alkali-Chloride Solutions up to
    623 K, 1,000 bar, and High Ionic Strength.” International Journal of Thermophysics.
    Vol. 30 (2009) 1,510–1,523.
    ref 2 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011

    корреляция позволяет найти вязкость соленой воды

    return density, kg/m3
    t_K,                            temperature, K
    p_MPaa,                         pressure, MPaa
    s_ppm,                          salinity, ppm
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


# Проверки функций на работоспособность


class TestPVT(unittest.TestCase):
    def test_unf_pb_Standing_MPaa(self):
        rsb_m3m3 = 100
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(unf_pb_Standing_MPaa(rsb_m3m3, gamma_oil, gamma_gas, t_K), 20.170210695316566,
                               delta=0.0001)

    def test_unf_pb_Valko_MPaa(self):
        rsb_m3m3 = 100
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(unf_pb_Valko_MPaa(rsb_m3m3, gamma_oil, gamma_gas, t_K), 22.694051278736964,
                               delta=0.0001)

    def test_unf_rs_Standing_m3m3(self):
        p_MPaa = 10
        pb_MPaa = 15
        rsb = 200
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(unf_rs_Standing_m3m3(p_MPaa, pb_MPaa, rsb, gamma_oil, gamma_gas, t_K),
                               122.74847910146916, delta=0.0001)

    def test_unf_rs_Velarde_m3m3(self):
        p_MPaa = 10
        pb_MPaa = 15
        rsb = 250
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(unf_rs_Velarde_m3m3(p_MPaa, pb_MPaa, rsb, gamma_oil, gamma_gas, t_K),
                               170.25302712587356, delta=0.0001)

    def test_unf_rsb_Mccain_m3m3(self):
        rsp_m3m3 = 150
        gamma_oil = 0.86
        psp_MPaa = 5
        tsp_K = 320
        self.assertAlmostEqual(unf_rsb_Mccain_m3m3(rsp_m3m3, gamma_oil, psp_MPaa, tsp_K),
                               161.03286985548442, delta=0.0001)

    def test_unf_gamma_gas_Mccain(self):
        rsp_m3m3 = 30
        rst_m3m3 = 20
        gamma_gassp = 0.65
        gamma_oil = 0.86
        psp_MPaa = 5
        tsp_K = 350
        self.assertAlmostEqual(unf_gamma_gas_Mccain(rsp_m3m3, rst_m3m3, gamma_gassp, gamma_oil, psp_MPaa, tsp_K),
                               0.7932830162938984, delta=0.0001)

    def test_unf_fvf_Mccain_m3m3_below(self):
        density_oilsto_kgm3 = 800
        rs_m3m3 = 200
        density_oil_kgm3 = 820
        gamma_gas = 0.6
        self.assertAlmostEqual(unf_fvf_Mccain_m3m3_below(density_oilsto_kgm3, rs_m3m3, density_oil_kgm3, gamma_gas),
                               1.1542114715227887, delta=0.0001)

    def test_unf_fvf_VB_m3m3_above(self):
        bob = 1.3
        cofb_1MPa = 3 * 10**(-3)
        pb_MPaa = 12
        p_MPaa = 15
        self.assertAlmostEqual(unf_fvf_VB_m3m3_above(bob, cofb_1MPa, pb_MPaa, p_MPaa), 1.288352492404749, delta=0.0001)

    def test_unf_compressibility_oil_VB_1Mpa(self):
        rs_m3m3 = 200
        t_K = 350
        gamma_oil = 0.86
        p_MPaa = 15
        gamma_gas = 0.6
        self.assertAlmostEqual(unf_compressibility_oil_VB_1Mpa(rs_m3m3, t_K, gamma_oil, p_MPaa, gamma_gas),
                               0.004546552811369566, delta=0.0001)

    def test_unf_fvf_Standing_m3m3_saturated(self):
        rs_m3m3 = 200
        gamma_gas = 0.6
        gamma_oil = 0.86
        t_K = 350
        self.assertAlmostEqual(unf_fvf_Standing_m3m3_saturated(rs_m3m3, gamma_gas, gamma_oil, t_K),
                               1.5527836202040448, delta=0.0001)

    def test_unf_density_oil_Mccain(self):
        p_MPaa = 10
        pb_MPaa = 12
        co_1MPa = 3 * 10**(-3)
        rs_m3m3 = 250
        gamma_gas = 0.6
        t_K = 350
        gamma_oil = 0.86
        gamma_gassp = 0
        self.assertAlmostEqual(unf_density_oil_Mccain(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, gamma_gas, t_K, gamma_oil,
                               gamma_gassp), 630.0536681794456, delta=0.0001)

    def test_unf_density_oil_Standing(self):
        p_MPaa = 10
        pb_MPaa = 12
        co_1MPa = 3 * 10**(-3)
        rs_m3m3 = 250
        bo_m3m3 = 1.1
        gamma_gas = 0.6
        gamma_oil = 0.86
        self.assertAlmostEqual(unf_density_oil_Standing(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, bo_m3m3, gamma_gas, gamma_oil
                                                        ), 948.7272727272725, delta=0.0001)

    def test_unf_deadoilviscosity_Beggs_cP(self):
        gamma_oil = 0.86
        t_K = 350
        self.assertAlmostEqual(unf_deadoilviscosity_Beggs_cP(gamma_oil, t_K), 2.86938394460968, delta=0.0001)

    def test_unf_saturatedoilviscosity_Beggs_cP(self):
        deadoilviscosity_cP = 2.87
        rs_m3m3 = 150
        self.assertAlmostEqual(unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, rs_m3m3), 0.5497153091178292,
                               delta=0.0001)

    def test_unf_undersaturatedoilviscosity_VB_cP(self):
        p_MPaa = 10
        pb_MPaa = 12
        bubblepointviscosity_cP = 1
        self.assertAlmostEqual(unf_undersaturatedoilviscosity_VB_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP),
                               0.9767303348551418, delta=0.0001)

    def test_unf_undersaturatedoilviscosity_Petrovsky_cP(self):
        p_MPaa = 10
        pb_MPaa = 12
        bubblepointviscosity_cP = 1
        self.assertAlmostEqual(unf_undersaturatedoilviscosity_Petrovsky_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP),
                               0.9622774530985722, delta=0.0001)

    def test_unf_oil_viscosity_Beggs_VB_cP(self):
        deadoilviscosity_cP = 2.87
        rs_m3m3 = 150
        p_MPaa = 10
        pb_MPaa = 12
        self.assertAlmostEqual(unf_oil_viscosity_Beggs_VB_cP(deadoilviscosity_cP, rs_m3m3, p_MPaa, pb_MPaa),
                               0.5497153091178292, delta=0.0001)

    def test_unf_pseudocritical_temperature_K(self):
        gamma_gas = 0.6
        y_h2s = 0.01
        y_co2 = 0.03
        y_n2 = 0.02
        self.assertAlmostEqual(unf_pseudocritical_temperature_K(gamma_gas, y_h2s, y_co2, y_n2), 198.0708725589674,
                               delta=0.0001)

    def test_unf_pseudocritical_pressure_MPa(self):
        gamma_gas = 0.6
        y_h2s = 0.01
        y_co2 = 0.03
        y_n2 = 0.02
        self.assertAlmostEqual(unf_pseudocritical_pressure_MPa(gamma_gas, y_h2s, y_co2, y_n2), 5.09893164741181,
                               delta=0.0001)

    def test_unf_zfactor_DAK(self):
        p_MPaa = 10
        t_K = 350
        ppc_MPa = 7.477307083789863
        tpc_K = 239.186917147216
        self.assertAlmostEqual(unf_zfactor_DAK(p_MPaa, t_K, ppc_MPa, tpc_K), 0.8414026170318333, delta=0.0001)

    def test_unf_gasviscosity_Lee_cP(self):
        t_K = 350
        p_MPaa = 10
        z = 0.84
        gamma_gas = 0.6
        self.assertAlmostEqual(unf_gasviscosity_Lee_cP(t_K, p_MPaa, z, gamma_gas), 0.015423237238038448, delta=0.0001)

    def test_unf_gas_fvf_m3m3(self):
        t_K = 350
        p_MPaa = 10
        z = 0.84
        self.assertAlmostEqual(unf_gas_fvf_m3m3(t_K, p_MPaa, z), 0.010162381033600544, delta=0.0001)

    def test_unf_density_brine_Spivey_kgm3(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        par = 1
        self.assertAlmostEqual(unf_density_brine_Spivey_kgm3(t_K, p_MPaa, s_ppm, par), 987.685677686006, delta=0.0001)

    def test_unf_compressibility_brine_Spivey_1MPa(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        z = 1
        par = 0
        self.assertAlmostEqual(unf_compressibility_brine_Spivey_1MPa(t_K, p_MPaa, s_ppm, z, par), 0.0004241522548512511,
                               delta=0.0001)

    def test_unf_fvf_brine_Spivey_m3m3(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        self.assertAlmostEqual(unf_fvf_brine_Spivey_m3m3(t_K, p_MPaa, s_ppm), 1.0279011434122953, delta=0.0001)

    def test_unf_viscosity_brine_MaoDuan_cP(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        self.assertAlmostEqual(unf_viscosity_brine_MaoDuan_cP(t_K, p_MPaa, s_ppm), 0.3745199364964906, delta=0.0001)

    def test_unf_pb_Glaso_MPaa(self):
        rs_m3m3 = 100
        t_K = 350
        gamma_oil = 0.86
        gamma_gas = 0.6
        self.assertAlmostEqual(unf_pb_Glaso_MPaa(rs_m3m3, t_K, gamma_oil, gamma_gas), 23.365669948236604, delta=0.0001)

    def test_unf_fvf_Glaso_m3m3_saturated(self):
        rs_m3m3 = 100
        t_K = 350
        gamma_oil = 0.86
        gamma_gas = 0.6
        self.assertAlmostEqual(unf_fvf_Glaso_m3m3_saturated(rs_m3m3, t_K, gamma_oil, gamma_gas), 1.2514004319480372,
                               delta=0.0001)

    def test_unf_fvf_Glaso_m3m3_below(self):
        rs_m3m3 = 100
        t_K = 350
        gamma_oil = 0.86
        gamma_gas = 0.6
        p_MPaa = 10
        self.assertAlmostEqual(unf_fvf_Glaso_m3m3_below(rs_m3m3, t_K, gamma_oil, gamma_gas, p_MPaa), 1.7091714311161692,
                               delta=0.0001)

    def test_unf_compressibility_gas_Mattar_1MPa(self):
        p_MPaa = 10
        t_K = 350
        ppc_MPa = 7.477307083789863
        tpc_K = 239.186917147216
        self.assertAlmostEqual(unf_compressibility_gas_Mattar_1MPa(p_MPaa, t_K, ppc_MPa, tpc_K), 0.47717077711622113,
                               delta=0.0001)


    def test_unf_McCain_specificgravity(self):
        p_MPaa = 10
        rsb_m3m3 = 100
        t_K = 350
        gamma_oil = 0.8
        gamma_gassp = 0.6
        self.assertAlmostEqual(unf_McCain_specificgravity(p_MPaa, rsb_m3m3, t_K, gamma_oil, gamma_gassp), 0.6004849666507259,
                               delta=0.0001)

    def test_unf_gwr_brine_Spivey_m3m3(self):
        s_ppm = 10000
        z = 1
        self.assertAlmostEqual(unf_gwr_brine_Spivey_m3m3(s_ppm, z), 0.0013095456419714546, delta=0.0001)

    def test_unf_zfactor_BrillBeggs(self):
        ppr = 2
        tpr = 2
        self.assertAlmostEqual(unf_zfactor_BrillBeggs(ppr,tpr), 0.9540692750239955, delta=0.0001)

if __name__ == '__main__':
    unittest.main()
