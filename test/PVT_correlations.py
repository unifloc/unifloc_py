# -*- coding: utf-8 -*-
"""
Created on Sat May  5 13:59:06 2018

@author: Rinat Khabibullin
"""
import numpy as np

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
    pb_MPaa = 0.5197 * (rsb_m3m3 / gamma_gas)**0.83 * 10**yg
    # для низких значений газосодержания зададим асимптотику Pb = 1 атма при Rsb = 0
    # для больших значений газосодержания не корректируем то, что дает корреляция
    if rsb_old < min_rsb:
        pb_MPaa = (pb_MPaa - 0.101325) * rsb_old / min_rsb + 0.101325
    return pb_MPaa



def unf_pb_Valko_MPaa(rsb_m3m3, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    bubble point pressure calculation according to Valko McCain (1998) correlation

    ref SPE 51086 "CORRELATION OF BUBBLEPOINT PRESSURES FOR RESERVOIR OILS A COMPARATIVE STUDY"
    W. D. McCain Jr., R. B. Soto, P.P. Valko, and T. A. Blasingame

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
    z2 = 25.537681965 - 57.519938195/gamma_oil + 46.327882495/gamma_oil**2 - 13.485786265/gamma_oil**3
    z3 = 4.51 - 10.84 * gamma_gas + 8.39 * gamma_gas ** 2 - 2.34 * gamma_gas ** 3
    z4 = 6.00696e-8*t_K**3 - 8.554832172e-5*t_K**2 + 0.043155018225018*t_K - 7.22546617091445
    z = z1 + z2 + z3 + z4

    pb_atma = 119.992765886175 * np.exp(0.0075 * z**2 + 0.713 * z)
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

def unf_Rs_Standing_m3m3(p_MPaa, Pb_MPaa=0, Rsb_m3m3=0 , gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    Gas-oil ratio calculation inverse of Standing (1947) correlation for bubble point pressure

    ref1 "A Pressure-Volume-Temperature Correlation for Mixtures of California Oil and Gases",
    M.B. Standing, Drill. & Prod. Prac., API, 1947.

    ref2  "Стандарт компании Юкос. Физические свойства нефти. Методы расчета." Афанасьев В.Ю., Хасанов М.М. и др. 2002 г

    return bubble point pressure abs in MPa
    p_MPaa,         pressure, MPa
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350         temperature, K
    """
    if Pb_MPaa==0 and Rsb_m3m3==0:
        # мольная доля газа
        yg = 1.225 + 0.001648 * t_K - 1.769 / gamma_oil
        Rs_m3m3 = gamma_gas * (1.92*p_MPaa / 10**yg)**1.204
    else:
        Rs_m3m3=Rsb_m3m3*(p_MPaa/Pb_MPaa)**1.204
    return Rs_m3m3

