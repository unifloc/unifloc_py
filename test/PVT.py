# -*- coding: utf-8 -*-
"""
Created on Mon Sept 04 2017

@author: rnt

UniflocPy

класс для расчета PVT свойств углеводородных флюидов и воды

"""

from enum import Enum

class PVTcorrelationSet(Enum):
    """
    enumerator for PVT correlation set selection
    """
    StandingBased = 0
    McCainBased = 1
    StrainghLine = 2


class BlackOil:
    """
    класс для описания смеси воды, нефти и газа в соответствии с моделью нелетучей нефти black oil
    позволяет рассчитать свойства флюидов для заданных термобарических условий
    """
    def __init__(self, gamma_oil=0.86, gamma_gas =0.8, gamma_wat = 1, Rsb_m3m3 = 100):
        """
        initialize fluids with default parameters
        :param gamma_oil: optional oil specific gravity
        :param gamma_gas: optional gas specific gravity
        :param gamma_wat: optional water specific gravity
        :param Rsb_m3m3: optional gas solution ratio
        """
        # зададим базовый набор свойств по умолчанию
        self.gammaWater = gamma_wat     # water specific density
        self.gammaOil = gamma_oil       # oil specific gravity
        self.gammaGas = gamma_gas       # gas specific gravity
        self.Rsb_m3m3 = Rsb_m3m3        # gas solution ratio at bubble point
        self.fw = 0                     # water cut, fraction
        # calibration parameters
        self.Pb_calibration_bar = 100               # bubble point pressure
        self.Tb_calibration_C = 80                # temperature Pb_bar taken
        self.Bob_calibration_m3m3 = 1.2             # oil FVF at bubble point condition
        self.Muob_calibration_cP = 0                # oil viscosity at bubble point

        self.Pb_bar = 0                 # oil bubble point pressure
        self.Tb_C = 0                   # oil temperature for Pb_bar calc
        self.Bo_m3m3 = 0
        self.Muo_cP = 0
        self.Mug_cP = 0
        self.Muw_cP = 0
        self.Mudeadoil_cP = 0           # dead oil viscosity calculated value

        self.sigmaOil_Nm = 0            # oil surface tention
        self.sigmaWater_Nm = 0          # water surface tention

        self.correlation = PVTcorrelationSet.StandingBased

    def calcPVT(self, P_bar,T_C):
        """
        caculates all PVT properties according to correlation set selected and initial data given
        :param P_bar: pressure for PVT calculations
        :param T_C: temperature for PVT calculations
        :return: fills PVT properties with correct values
        """
        return 1

