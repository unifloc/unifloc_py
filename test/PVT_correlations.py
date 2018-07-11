# -*- coding: utf-8 -*-
"""
Created on Sat May  5 13:59:06 2018

@author: Rinat Khabibullin
         Alexey Vodopyan
"""
import numpy as np


# PVT свойства для нефти


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
    t_K=350         temperature, K
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

    ref SPE  "Reservoir oil bubblepoint pressures revisited; solution gas–oilratios and surface gas specific gravities"
    W. D. McCain Jr.,P.P. Valko, 
    
    return bubble point pressure abs in MPa
    rsb_m3m3,       solution ration at bubble point, must be given, m3/m3
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350         temperature, K
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


def unf_rs_Standing_m3m3(p_MPaa, pb_MPaa=0, rsb_m3m3=0, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    Gas-oil ratio calculation inverse of Standing (1947) correlation for bubble point pressure

    ref1 "A Pressure-Volume-Temperature Correlation for Mixtures of California Oil and Gases",
    M.B. Standing, Drill. & Prod. Prac., API, 1947.

    ref2  "Стандарт компании Юкос. Физические свойства нефти. Методы расчета." Афанасьев В.Ю., Хасанов М.М. и др. 2002 г

    return gas-oil ratio abs in m3/m3
    p_MPaa,         pressure, MPa
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
    pb_MPaa=0,      buble point pressure, MPa
    rsb_m3m3=0,     gas-oil ratio at the bublepoint pressure, m3/m3
    """
    if pb_MPaa == 0 or rsb_m3m3 == 0:
        # мольная доля газа
        yg = 1.225 + 0.001648 * t_K - 1.769 / gamma_oil
        rs_m3m3 = gamma_gas * (1.92 * p_MPaa / 10 ** yg) ** 1.204
    else:
        rs_m3m3 = rsb_m3m3 * (p_MPaa / pb_MPaa) ** 1.204
    return rs_m3m3


def unf_rs_Velarde_m3m3(p_MPaa, pb_MPaa=10, rsb_m3m3=100, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    Solution Gas-oil ratio calculation according to Velarde McCain (1999) correlation

    ref1 "Correlation of Black Oil Properties at Pressures Below Bubblepoint Pressure—A New Approach",
    J. VELARDE, T.A. BLASINGAME Texas A&M University, W.D. MCCAIN, JR. S.A. Holditch & Associates, Inc 1999

    return gas-oil ratio abs in m3/m3
    p_MPaa,         pressure, MPaa
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
    pb_MPaa=10,     buble point pressure, MPaa
    rsb_m3m3=100,   gas-oil ratio at the bublepoint pressure, m3/m3
    """
    api = 141.5 / gamma_oil - 131.5  # в будущем когда разберусь с классами и тд нужно сделать через конвертор
    t_F = 1.8 * (t_K - 273.15) + 32  # в будущем когда разберусь с классами и тд нужно сделать через конвертор
    pb_psia = 145.03773800721814 * pb_MPaa  # в будущем когда разберусь с классами и тд нужно сделать через конвертор
    if pb_psia > 14.7:
        pr = (p_MPaa * 145.03773800721814 - 14.7) / (pb_psia - 14.7)
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
        pr = (p_MPaa * 145.03773800721814 - 14.7) / (pb_psia - 14.7)
        rsr = a1 * pr ** a2 + (1 - a1) * pr ** a3
        rs_m3m3 = rsr * rsb_m3m3
    else:
        rs_m3m3 = rsb_m3m3
    return rs_m3m3


def unf_rsb_Mccain_m3m3(psp_MPaa, tsp_K, rsp, gamma_oil):
    """
    Solution Gas-oil ratio at bubble point pressure calculation according to McCain (2002) correlation 
    taking into account the gas losses at separator and stock tank
    
    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002
    
    to be continued
    
    """
    if psp_MPaa > 0 and tsp_K > 0:
        api = 141.5 / gamma_oil - 131.5
        psp_psia = 145.03773800721814 * psp_MPaa
        tsp_F = 1.8 * (tsp_K - 273.15) + 32
        z1 = -8.005 + 2.7 * np.log(psp_psia) - 0.161 * np.log(psp_psia) ** 2
        z2 = 1.224 - 0.5 * np.log(tsp_F)
        z3 = -1.587 + 0.0441 * np.log(api) - 2.29 * 10 ** (-5) * np.log(api) ** 2
        z = z1 + z2 + z3
        rst = np.exp(3.955 + 0.83 * z - 0.024 * z ** 2 + 0.075 * z ** 3)
        rsb = rsp + rst
    elif rsp >= 0:
        rsb = 1.1618 * rsp  # в случае если неизвестны условия в сепараторе, можно произвести приблизительную оценку
    else:
        rsb = 0
    rsb = 6.289814 / 35.31467 * rsb
    return rsb


def unf_gamma_gas_Mccain(psp_MPaa, rsp_m3m3, rst_m3m3, gamma_gassp, tsp_K, gamma_oil):
    """
    Correlation for separator gas specific gravity
    
    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002
    
    to be continued
    
    """
    if psp_MPaa > 0 and rsp_m3m3 > 0 and rst_m3m3 > 0 and gamma_gassp > 0:
        api = 141.5 / gamma_oil - 131.5
        psp_psia = 145.03773800721814 * psp_MPaa
        tsp_F = 1.8 * (tsp_K - 273.15) + 32
        rsp_scfstb = 5.61458333333333 * rsp_m3m3
        rst_scfstb = 5.61458333333333 * rst_m3m3
        z1 = -17.275 + 7.9597 * np.log(psp_psia) - 1.1013 * np.log(psp_psia) ** 2 + 2.7735 * 10 ** (-2) * np.log(
            psp_psia) ** 3 + 3.2287 * 10 ** (-3) * np.log(psp_psia) ** 4
        z2 = -0.3354 - 0.3346 * np.log(rsp_scfstb) + 0.1956 * np.log(rsp_scfstb) ** 2 - 3.4374 * 10 ** (-2) * np.log(
            rsp_scfstb) ** 3 + 2.08 * 10 ** (-3) ** np.log(rsp_scfstb) ** 4
        z3 = 3.705 - 0.4273 * np.log(api) + 1.818 * 10 ** (-2) * np.log(api) ** 2 - 3.459 * 10 ** (-4) * np.log(
            api) ** 3 + 2.505 * 10 ** (-6) * np.log(api) ** 4
        z4 = -155.52 + 629.61 * np.log(gamma_gassp) - 957.38 * np.log(gamma_gassp) ** 2 + 647.57 * np.log(
            gamma_gassp) ** 3 - 163.26 * np.log(gamma_gassp) ** 4
        z5 = 2.085 - 7.097 * 10 ** (-2) * np.log(tsp_F) + 9.859 * 10 ** (-4) * np.log(tsp_F) ** 2 - 6.312 * 10 ** (
            -6) ** np.log(tsp_F) ** 3 + 1.4 * 10 ** (-8) * np.log(tsp_F) ** 4
        z = z1 + z2 + z3 + z4 + z5
        gamma_gasst = 1.219 + 0.198 * z + 0.0845 * z ** 2 + 0.03 * z ** 3 + 0.003 * z ** 4
        gamma_gas = (gamma_gassp * rsp_scfstb + gamma_gasst * rst_scfstb) / (rsp_scfstb + rst_scfstb)
    elif gamma_gassp >= 0:
        gamma_gas = 1.066 * gamma_gassp
    else:
        gamma_gas = 0
    return gamma_gas


def unf_fvf_Mccain_m3m3_below(density_oilsto_kgm3, rs_m3m3, density_oil_kgm3):
    """  
    Oil Formation Volume Factor according McCain correlation for pressure below bubble point pressure
    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    
    to be continued
    
    """
    density_oilsto_lbcuft = 0.0624279606 * density_oilsto_kgm3
    density_oil_lbcuft = 0.0624279606 * density_oil_kgm3
    rs_scfstb = 5.61458333333333 * rs_m3m3
    bo = (density_oilsto_lbcuft + 0.01357 * rs_scfstb) / density_oil_lbcuft
    return bo


def unf_fvf_VB_m3m3_above(bob, cofb_1MPa, pb_MPaa, p_MPaa):
    """  
    Oil Formation Volume Factor according equation for pressure above bubble point pressure
    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    
    ! Actually, this correlation is belonged ro Vasquez & Beggs (1980). In some sources is
    noted that this is Standing correlation.
    ref2 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA
    cofb_1MPa - oil compressibility
    
    to be continued
    
    """
    if p_MPaa <= pb_MPaa:
        bo = bob
    else:
        pb_psia = pb_MPaa * 145.03773800721814
        p_psia = p_MPaa * 145.03773800721814
        cofb_1psi = 1 / 145.03773800722 * cofb_1MPa
        bo = bob * np.exp(cofb_1psi * (pb_psia - p_psia))
    return bo


def unf_compressibility_oil_VB_1Mpa(rs_m3m3, t_K, gamma_oil, p_MPaa, gamma_gas=0.6):
    """  
    oil compressibility according to Vasquez & Beggs (1980) correlation 
    ref1 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA
   
    to be continued
    
    """
    if p_MPaa > 0:
        rs_scfstb = 5.61458333333333 * rs_m3m3
        t_F = 1.8 * (t_K - 273.15) + 32
        api = 141.5 / gamma_oil - 131.5
        p_psia = p_MPaa * 145.03773800721814
        co_1MPa = 145.03773800722 * (-1433 + 5 * rs_scfstb + 17.2 * t_F - 1180 * gamma_gas + 12.61 * api) / (
                    10 ** 5 * p_psia)
    else:
        co_1MPa = 0
    return co_1MPa


def unf_fvf_Standing_m3m3_saturated(rs_m3m3, gamma_gas, gamma_oil, t_K):
    """  
    Oil Formation Volume Factor according Standing equation at bubble point pressure
    ref1 Volumetric and phase behavior of oil field hydrocarbon systems / M.B. Standing Standing, M. B. 1981
    
    to be continued
    
    """
    rs_scfstb = 5.61458333333333 * rs_m3m3
    t_F = 1.8 * (t_K - 273.15) + 32
    bo = 0.972 + 1.47 * 10 ** (-4) * (rs_scfstb * (gamma_gas / gamma_oil) ** 0.5 + 1.25 * t_F) ** 1.175
    return bo


def unf_density_oil_Mccain(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, gamma_gas, t_K, gamma_oil, gamma_gassp = 0):
    """  
    Oil density according Standing, M.B., 1977; Witte, T.W., Jr., 1987; and McCain, W.D., Jr. and Hill, N.C.,
1995 correlation.
    ref1 book Mccain_w_d_spivey_j_p_lenn_c_p_petroleum_reservoir_fluid,third edition, 2011
    
    to be continued
    
    """
    rs_scfstb = 5.61458333333333 * rs_m3m3
    ro_po = 52.8 - 0.01 * rs_scfstb  # первое приближение
    if gamma_gassp == 0:  # если нет данных о плотности газа в сепараторе
        gamma_gassp = gamma_gas
    epsilon = 0.000001
    maxiter = 100  # максимальное число иттераций
    counter = 0
    ro_po_current = ro_po
    ro_po_previous = 0
    while abs(ro_po_current - ro_po_previous) > epsilon and counter < maxiter:
        ro_po_previous = ro_po_current
        ro_a = -49.8930 + 85.0149 * gamma_gassp - 3.70373 * gamma_gassp * ro_po +\
            0.0479818 * gamma_gassp * ro_po ** 2 + 2.98914 * ro_po - 0.0356888 * ro_po ** 2
        ro_po = (rs_scfstb * gamma_gas + 4600 * gamma_oil) / (73.71 + rs_scfstb * gamma_gas / ro_a)
        ro_po_current = ro_po
        counter = counter + 1
    p_psia = p_MPaa * 145.03773800721814
    t_F = 1.8 * (t_K - 273.15) + 32
    if p_MPaa < pb_MPaa:
        dro_p = (0.167 + 16.181 * 10 ** (-0.0425 * ro_po)) * (p_psia / 1000) - 0.01 * (
                    0.299 + 263 * 10 ** (-0.0603 * ro_po)) * (p_psia / 1000) ** 2
        ro_bs = ro_po + dro_p
        dro_t = (0.00302 + 1.505 * ro_bs ** (-0.951)) * (t_F - 60) ** 0.938 - (
                    0.0216 - 0.0233 * 10 ** (-0.0161 * ro_bs)) * (t_F - 60) ** 0.475
        ro_or = ro_bs - dro_t
    else:
        pb_psia = p_MPaa * 145.03773800721814
        dro_p = (0.167 + 16.181 * 10 ** (-0.0425 * ro_po))(pb_psia / 1000) - 0.01 * (
                    0.299 + 263 * 10 ** (-0.0603 * ro_po)) * (pb_psia / 1000) ** 2
        ro_bs = ro_po + dro_p
        dro_t = (0.00302 + 1.505 * ro_bs ** (-0.951)) * (t_F - 60) ** 0.938 - (
                    0.0216 - 0.0233 * 10 ** (-0.0161 * ro_bs)) * (t_F - 60) ** 0.475
        ro_orb = ro_bs - dro_t
        co_1psi = 1 / 145.03773800722 * co_1MPa
        ro_or = ro_orb * np.exp(co_1psi * (p_psia - pb_psia))
    ro_or = 1 / 0.0624279606 * ro_or
    return ro_or


"""
def unf_density_oil_Standing
    
    return po
    
    позже допишу плотность, не получается найти источник
"""
"""
def unf_deadoilviscosity_Standing(gamma_oil, t_K):
    """
#  Correlation for Oil viscosity according to Standing Correlation

#    

""" 
    не могу найти источник
    API = 141.5 / gamma_oil - 131.5
    viscosity__cP = (0.32 + 1.8 * (10 ** 7) / (API ** 4.53)) * \
    (360 / (1.8 * (t__C + 273) - 260)) ** (10 ** (0.43 + 8.33 / (API)))
    return viscosity__cP
"""


def unf_deadoilviscosity_Beggs_cP(gamma_oil, t_K):
    api = 141.5 / gamma_oil - 131.5
    t_F = 1.8 * (t_K - 273.15) + 32
    c = 10 ** (3.0324 - 0.02023 * api) / t_F ** (-1.163)
    viscosity_cP = 10 ** c - 1
    return viscosity_cP


def unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, rs_m3m3):
    # viscosity correlation for pressure below bubble point
    rs_scfstb = 5.61458333333333 * rs_m3m3
    a = 10.715 * (rs_scfstb + 100) ** (-0.515)
    b = 5.44 * (rs_scfstb + 150) ** (-0.338)
    viscosity_cP = a * deadoilviscosity_cP ** b
    return viscosity_cP


def unf_undersaturatedoilviscosity_VB_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP):
    """"
    viscosity correlation for pressure above bubble point
    ref2 Vazquez, M. and Beggs, H.D. 1980. Correlations for Fluid Physical Property Prediction.
    J Pet Technol 32 (6): 968-970. SPE-6719-PA
    """
    p_psia = p_MPaa * 145.03773800721814
    pb_psia = pb_MPaa * 145.03773800721814
    m = 2.6 * p_psia ** 1.187 * np.exp(-11.513 - 8.98 * 10 ** (-5) * p_psia)
    viscosity_cP = bubblepointviscosity_cP * (p_psia / pb_psia) ** m
    return viscosity_cP


def unf_undersaturatedoilviscosity_Petrovsky_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP):
    # viscosity correlation for pressure above bubble point
    # ref 1 Viscosity Correlations for Gulf of Mexico Crude Oils
    A = -1.0146 + 1.3322 * np.log(bubblepointviscosity_cP) - 0.4876 * np.log(
        bubblepointviscosity_cP) ** 2 - 1.15036 * np.log(bubblepointviscosity_cP) ** 3
    p_psia = p_MPaa * 145.03773800721814
    pb_psia = pb_MPaa * 145.03773800721814
    viscosity_cP = bubblepointviscosity_cP + 1.3449 * 10 ** (-3) * (p_psia - pb_psia) * 10 ** A
    return viscosity_cP


def unf_oil_viscosity_Beggs_VB_cP(deadoilviscosity_cP, rs_m3m3, p_MPaa, pb_MPaa):
    # function for calculating the viscosity at any pressure
    saturatedviscosity_cP = unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, rs_m3m3)
    if p_MPaa <= pb_MPaa:
        viscosity_cP = saturatedviscosity_cP
    else:
        viscosity_cP = unf_undersaturatedoilviscosity_VB_cP(p_MPaa, pb_MPaa, saturatedviscosity_cP)
    return viscosity_cP


"""
В дальнейшем функции для нефти будут дополняться, а пока перейдем к PVT для газа
"""


# PVT свойства для газа

def unf_pseudocritical_properties_MPa_K(gamma_gas, tc_h2s_K=0, tc_co2_K=0, tc_n2_K=0, pc_h2s_MPaa=1, pc_co2_MPaa=1,
                                        pc_n2_MPaa=1, y_h2s=0, y_co2=0, y_n2=0):
    """
    correlation for pseudocritical properties
    ref 1 Piper, L.D., McCain, W.D., Jr., and Corredor, J.H. “Compressibility Factors for 
    Naturally Occurring Petroleum Gases.” Gas Reservoir Engineering. Reprint Series. Richardson,
    TX: SPE. Vol. 52 (1999) 186–200
    """
    if pc_h2s_MPaa == 0 or pc_co2_MPaa == 0 or pc_n2_MPaa == 0:
        tpc_K = 0
        ppc_MPa = 0
    else:
        tc_h2s_R = 1.8 * (tc_h2s_K - 273.15) + 491.67
        tc_co2_R = 1.8 * (tc_co2_K - 273.15) + 491.67
        tc_n2_R = 1.8 * (tc_n2_K - 273.15) + 491.67
        pc_h2s_psia = pc_h2s_MPaa * 145.03773800721814
        pc_co2_psia = pc_co2_MPaa * 145.03773800721814
        pc_n2_psia = pc_n2_MPaa * 145.03773800721814
        J = 1.1582 * 10 ** (-1) - 4.5820 * 10 ** (-1) * y_h2s * (tc_h2s_R / pc_h2s_psia) - \
            9.0348 * 10 ** (-1) * y_co2 * (tc_co2_R / pc_co2_psia) - 6.6026 * 10 ** (-1) * y_n2 * (
                        tc_n2_R / pc_n2_psia) + \
            7.0729 * 10 ** (-1) * gamma_gas - 9.9397 * 10 ** (-2) * gamma_gas ** 2
        K = 3.8216 - 6.5340 * 10 ** (-2) * y_h2s * (tc_h2s_R / pc_h2s_psia) - \
            4.2113 * 10 ** (-1) * y_co2 * (tc_co2_R / pc_co2_psia) - 9.1249 * 10 ** (-1) * y_n2 * (
                        tc_n2_R / pc_n2_psia) + \
            1.7438 * 10 * gamma_gas - 3.2191 * gamma_gas ** 2
        tpc_R = K ** 2 / J
        ppc_psia = tpc_R / J
        tpc_K = (tpc_R - 491.67) / 1.8 + 273.15
        ppc_MPa = ppc_psia / 145.03773800721814
    return ppc_MPa, tpc_K


def unf_zfactor_DAK(p_MPaa, t_K, ppc_MPa, tpc_K):
    """
    correlation for z-factor
    ref 1 Dranchuk, P.M. and Abou-Kassem, J.H. “Calculation of Z Factors for Natural
    Gases Using Equations of State.” Journal of Canadian Petroleum Technology. (July–September 1975) 34–36.
    """
    ppr = p_MPaa / ppc_MPa
    tpr = t_K / tpc_K
    epsilon = 0.000001
    maxiter = 100
    counter = 0
    z_previous = 1
    ropr = 0.27 * (ppr / (z_previous * tpr))
    z_current = 1 + (0.3265 - 1.0700 / tpr - 0.5339 / tpr ** 3 + 0.01569 / tpr ** 4 - 0.05165 / tpr ** 5) * ropr + \
        (0.5475 - 0.7361 / tpr + 0.1844 / tpr ** 2) * ropr ** 2 - 0.1056 * (-0.7361 / tpr + 0.1844 / tpr ** 2) * \
        ropr ** 5 + 0.6134 * (1 + 0.7210 * ropr ** 2) * (ropr ** 2 / tpr ** 3) * np.exp(-0.7210 / ropr ** 2)
    while abs(z_current - z_previous) > epsilon and counter < maxiter:
        if ppr <= 2:
            z_previous = z_current + (z_current - z_previous)
        elif ppr <= 3:
            z_previous = z_current + (z_current - z_previous) / 2
        elif ppr <= 6:
            z_previous = z_current + (z_current - z_previous) / 3
        else:
            z_previous = z_current + (z_current - z_previous) / 5
        ropr = 0.27 * (ppr / (z_previous * tpr))
        z_current = 1 + (0.3265 - 1.0700 / tpr - 0.5339 / tpr ** 3 + 0.01569 / tpr ** 4 - 0.05165 / tpr ** 5) * ropr + \
            (0.5475 - 0.7361 / tpr + 0.1844 / tpr ** 2) * ropr ** 2 - 0.1056 * (-0.7361 / tpr + 0.1844 / tpr ** 2) * \
            ropr ** 5 + 0.6134 * (1 + 0.7210 * ropr ** 2) * (ropr ** 2 / tpr ** 3) * np.exp(-0.7210 / ropr ** 2)
        counter = counter + 1
    return z_current
