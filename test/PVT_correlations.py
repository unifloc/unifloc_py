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

    return gas-oil ratio abs in m3/m3
    p_MPaa,         pressure, MPa
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
    Pb_MPaa=0,      buble point pressure, MPa
    Rsb_m3m3=0,     gas-oil ratio at the bublepoint pressure, m3/m3
    """
    if Pb_MPaa==0 or Rsb_m3m3==0:
        # мольная доля газа
        yg = 1.225 + 0.001648 * t_K - 1.769 / gamma_oil
        Rs_m3m3 = gamma_gas * (1.92*p_MPaa / 10**yg)**1.204
    else:
        Rs_m3m3=Rsb_m3m3*(p_MPaa/Pb_MPaa)**1.204
    return Rs_m3m3

def unf_Rs_Velarde_m3m3(p_MPaa, Pb_MPaa=10, Rsb_m3m3=100, gamma_oil=0.86, gamma_gas=0.6, t_K=350):
    """
    Solution Gas-oil ratio calculation according to Velarde McCain (1999) correlation

    ref1 "Correlation of Black Oil Properties at Pressures Below Bubblepoint Pressure—A New Approach",
    J. VELARDE, T.A. BLASINGAME Texas A&M University, W.D. MCCAIN, JR. S.A. Holditch & Associates, Inc 1999

    return gas-oil ratio abs in m3/m3
    p_MPaa,         pressure, MPa
    gamma_oil=0.86, specific gas density (by water)
    gamma_gas=0.6,  specific gas density (by air)
    t_K=350,        temperature, K
    Pb_MPaa=10,      buble point pressure, MPa
    Rsb_m3m3=100,     gas-oil ratio at the bublepoint pressure, m3/m3
    """
    API = 141.5/gamma_oil - 131.5           # в будущем когда разберусь с классами и тд нужно сделать через конвертор 
    T_F = 1.8*(t_K - 273.15) + 32           # в будущем когда разберусь с классами и тд нужно сделать через конвертор
    Pb_psia = 145.03773800721814 * Pb_MPaa  # в будущем когда разберусь с классами и тд нужно сделать через конвертор
    if Pb_psia > 14.7:
        Pr = (p_MPaa*145.03773800721814 - 14.7)/(Pb_psia -14.7)
    else:
        Pr = 0
    if Pr <= 0:
        Rs_m3m3 = 0
    elif Pr<1:
        
        A0 = 9.73*10**(-7)
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
    
        a1 = A0*gamma_gas**A1*API**A2*T_F**A3*(Pb_psia-14.7)**A4
        a2 = B0*gamma_gas**B1*API**B2*T_F**B3*(Pb_psia-14.7)**B4
        a3 = C0*gamma_gas**C1*API**C2*T_F**C3*(Pb_psia-14.7)**C4
        Pr = (p_MPaa*145.03773800721814 - 14.7)/(Pb_psia -14.7)
        Rsr = a1*Pr**a2 + (1-a1)*Pr**a3
        Rs_m3m3 = Rsr*Rsb_m3m3
    else:
        Rs_m3m3 = Rsb_m3m3
    return Rs_m3m3

def unf_Rsb_Mccain_m3m3(Psp_MPaa,Tsp_K,Rsp,gamma_oil):
    """
    Solution Gas-oil ratio at bubble point pressure calculation according to McCain (2002) correlation 
    taking into account the gas losses at separator and stock tank
    
    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002
    """
    if Psp_MPaa>0 and Tsp_K>0:
        API = 141.5/gamma_oil - 131.5
        Psp_psia = 145.03773800721814 * Psp_MPaa
        Tsp_F = 1.8*(Tsp_K - 273.15) + 32
    
        z1 = -8.005 + 2.7*np.log(Psp_psia) - 0.161*np.log(Psp_psia)**2
        z2 = 1.224 - 0.5*np.log(Tsp_F)
        z3 = -1.587 + 0.0441*np.log(API) - 2.29*10**(-5)*np.log(API)**2
        z = z1 + z2 + z3
        Rst = np.exp(3.955+0.83*z-0.024*z**2+0.075*z**3)
        Rsb = Rsp + Rst
    elif Rsp>=0:
        Rsb = 1.1618*Rsp # в случае если неизвестны условия в сепараторе, можно произвести приблизительную оценку
    else:
        Rsb = 0
    Rsb=35.31467/6.289814 *Rsb
    return Rsb

def unf_gamma_gas_Mccain(Psp_MPaa,Rsp_m3m3, Rst_m3m3, gamma_gasSP, Tsp_K,gamma_oil):
    """
    Correlation for separator gas specific gravity
    
    ref1 "Reservoir oil bubblepoint pressures revisited; solution gas–oil ratios and surface gas specific gravities",
    J. VELARDE, W.D. MCCAIN, 2002
    """
    if Psp_MPaa>0 and Rsp_m3m3>0 and Rst_m3m3>0 and gamma_gasSP>0:
        API = 141.5/gamma_oil - 131.5
        Psp_psia = 145.03773800721814 * Psp_MPaa
        Tsp_F = 1.8*(Tsp_K - 273.15) + 32
        Rsp_scfSTB = 6.289814/35.31467 * Rsp_m3m3
        Rst_scfSTB = 6.289814/35.31467 * Rst_m3m3
    
        z1 = -17.275 + 7.9597*np.log(Psp_psia) - 1.1013*np.log(Psp_psia)**2 + 2.7735*10**(-2) * np.log(Psp_psia)**3 + 3.2287*10**(-3)*np.log(Psp_psia)**4
        z2 = -0.3354 - 0.3346*np.log(Rsp_scfSTB) + 0.1956*np.log(Rsp_scfSTB)**2 - 3.4374*10**(-2)*np.log(Rsp_scfSTB)**3 + 2.08*10**(-3)**np.log(Rsp_scfSTB)**4
        z3 = 3.705 - 0.4273*np.log(API) + 1.818*10**(-2)*np.log(API)**2 - 3.459*10**(-4)*np.log(API)**3 +2.505*10**(-6)*np.log(API)**4
        z4 = -155.52 + 629.61*np.log(gamma_gasSP) - 957.38*np.log(gamma_gasSP)**2 + 647.57*np.log(gamma_gasSP)**3-163.26*np.log(gamma_gasSP)**4
        z5 = 2.085 - 7.097*10**(-2)*np.log(Tsp_F) + 9.859*10**(-4)*np.log(Tsp_F)**2 -6.312*10**(-6)**np.log(Tsp_F)**3 + 1.4*10**(-8)*np.log(Tsp_F)**4
        z = z1+z2+z3+z4+z5
        gamma_gasST= 1.219+0.198*z+0.0845*z**2+0.03*z**3+0.003*z**4
    
        gamma_gas = (gamma_gasSP*Rsp_scfSTB + gamma_gasST*Rst_scfSTB)/(Rsp_scfSTB + Rst_scfSTB)
    elif gamma_gasSP>=0:
        gamma_gas = 1.066*gamma_gasSP
    else:
        gamma_gas = 0
    return gamma_gas
