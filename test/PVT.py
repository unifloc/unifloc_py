# -*- coding: utf-8 -*-
"""
Created on Mon Sept 04 2017

@author: rnt

UniflocPy

класс для расчета PVT свойств углеводородных флюидов и воды

"""


class ComponentGeneral:
    """
    Абстрактный класс для описания компонентоы углеводородных флюидов
    """
    def __init__(self):
        self._gamma = 1          # specific gravity of component, dimensionless
        self.rho_kgm3 = 1       # density with dimension
        self.mu_cp = 1          # dynamic viscosity
        """ термобарические условия """
        self._p_bar = 1
        self._t_c = 15

    def calc(self, p_atm, t_c):
        """ recalculate all parameters according to some pressure and termperature"""
        return 1

    @property
    def gamma(self):
        return self._gamma

    @gamma.setter
    def gamma(self, value):
        self._gamma = value


class GasGeneral(ComponentGeneral):
    """
    Класс для описания свойств углеводородных газов
    """
    def __init__(self):
        super().__init__()
        self._z = 0.9               # сверхсжимаемость
        self._pseudo_pressure_mpa = 1
        self._pseudo_temperature_k = 1
        self.gamma = 0.8


    @property
    def z(self):
        return  self._z

    def _calc_z(self):
        pass

    def _calc_bg(self):
        pass

    def _calc_mug(self):
        pass

    @ComponentGeneral.gamma.setter
    def gamma(self, value):
        self._gamma = value
        self._pseudo_pressure_mpa = 4.9 - 0.4 * self._gamma
        self._pseudo_temperature_k = 95 + 171 * self._gamma

    @property
    def pseudo_temperature_k(self):
        return self._pseudo_temperature_k

    @property
    def pseudo_pressure_mpa(self):
        return self._pseudo_pressure_mpa


 class OilGeneral(ComponentGeneral):
    """
    Класс для описания свойств нефти по модели нелетучей нефти
    """
    def __init__(self):
        super().__init__()              # часть базовых свойств наследуется
        self._gas = GasGeneral()        # create gas component
        self.rsb_m3m3 = 100

        self.pb_calibr_bar = 100        # калибровочное значение давления насыщения
        self.tb_calibr_c = 50           # температуры для калибровки по давлению насыщения
        self.bob_calibr_m3m3 = 1.2      # калибровочное значение объемного коэффициента
        self.muob_calibr_cp = 1         # калибровочное значение вязкости при давлении насыщения

        """ расчетные свойства """
        self._rs_m3m3 = 1
        self._bo_m3m3 = 1
        self._mu_cp = 1

    @property
    def gas(self):
        return self._gas

    @property
    def rs_m3m3(self):
        """ газосодержание """
        return self._rs_m3m3

    def _calc_rs_m3m3(self, p_bar, t_c):
        """ тут должна быть реализация расчета газосодержания
        """
        if p_bar < self.pb_calibr_bar:
            return self.rsb_m3m3 / self.pb_calibr_bar * p_bar
        else:
            return self.rsb_m3m3

    def _calc_bo_m3m3(self, p_bar, t_c):
        """
        Расчет объемного коэффициента нефти по корреляции
        :param p_bar:
        :param t_c:
        :return:
        """
        pass

    def calc(self, p_atm, t_c):
        """ реализация расчета свойств нефти """
        self._rs_m3m3 = self._calc_rs_m3m3(p_atm, t_c)


class WaterGeneral(ComponentGeneral):
    """
    класс описывающий свойства воды
    """
    def __init__(self):
        super().__init__()


class Fluid:
    """
    класс описывающий флюид на основе модели нелетучей нефти
    """
    def __init__(self):
        self._oil = OilGeneral()
        self._water = ComponentGeneral()

        self._qliq_m3day = 10   # liquid rate
        self._fw = 0            # water cut, fraction

    @property
    def fw(self):
        return self._fw

    def calc_pvt(self, p_bar, t_c):
        pass
