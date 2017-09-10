# -*- coding: utf-8 -*-
"""
Created on Mon Sept 04 2017

@author: rnt

UniflocPy

класс для расчета PVT свойств углеводородных флюидов и воды

"""


class Fluid:
    """
    базовый класс для описания свойств пластовых флюидов -
    смеси воды, нефти и газа в соответствии с моделью нелетучей нефти black oil
    позволяет рассчитать свойства флюидов для заданных термобарических условий
    """
    def __init__(self, gamma_oil=0.86, gamma_gas =0.8, gamma_wat = 1, rsb_m3m3 = 100):
        """
        initialize fluids with default parameters
        :param gamma_oil: optional oil specific gravity
        :param gamma_gas: optional gas specific gravity
        :param gamma_wat: optional water specific gravity
        :param Rsb_m3m3: optional gas solution ratio
        """
        # зададим базовый набор свойств по умолчанию
        self.GammaWater = gamma_wat     # water specific density
        self.GammaOil = gamma_oil       # oil specific gravity
        self.GammaGas = gamma_gas       # gas specific gravity
        self.Rsb_m3m3 = rsb_m3m3        # gas solution ratio at bubble point
        self.Fw = 0                     # water cut, fraction

        # calibration parameters
        self._Pb_calibration_bar = 100    # bubble point calibration pressure
        self._Pb_bar = 0                 # oil bubble point pressure if 0 or negative - calculated

        self._Tb_C = 0                   # oil temperature for Pb_bar calc
        self._Bo_m3m3 = 0
        self._Muo_cP = 0
        self._Mug_cP = 0
        self._Muw_cP = 0
        self._MuDeadOil_cP = 0           # dead oil viscosity calculated value

        self._sigmaOil_Nm = 0            # oil surface tention
        self._sigmaWater_Nm = 0          # water surface tention

    """ bubble point pressure """
    @property
    def Pb_bar(self):
        return self._Pb_bar  # здесь надо проверить если значение есть - вернуть, если нет то рассчитать

    @Pb_bar.setter
    def Pb_bar(self,value):     # set calibration
        if value > 0:
            self._Pb_calibration_bar = value
        else
            self._Pb_calibration_bar = value
        self._Pb_bar = value

    @property
    def Pb_atm(self):
        return self._Pb_bar * 1


    def calcPVT(self, P_bar,T_C):
        """
        caculates all PVT properties according to correlation set selected and initial data given
        :param P_bar: pressure for PVT calculations
        :param T_C: temperature for PVT calculations
        :return: fills PVT properties with correct values
        """
        return 1


class FluidStanding(Fluid):
    """
    class for PVT properties estimation based on Standing correlation set
    """
    pass


class FluidMcCain(FluidStanding):
    """
    class for PVT estimation based on McCain correlations
    """
    pass

