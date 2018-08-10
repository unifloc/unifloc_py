# -*- coding: utf-8 -*-
"""
Created on Mon Sept 04 2017

@author: rnt

UniflocPy

класс для расчета PVT свойств углеводородных флюидов и воды

"""
authentication token

class ComponentGeneral:
    """
    Абстрактный класс для описания компонентоы углеводородных флюидов
    """
    def __init__(self):
        self._gamma = 1          # specific gravity of component, dimensionless
        self._relative_density_sckgm3 = un.air_density_sckgm3
        self._rho_kgm3 = 1       # density with dimension
        self._mu_cp = 1          # dynamic viscosity
        self._fvf_m3m3 = 1       # component formation volume factor
        self._co_1atm = 1e-5     # component compressibility
        """ термобарические условия """
        self._p_bar = un.psc_bar # thermobaric conditions for all parameters
        self._t_c = un.tsc_c     # can be set up by calc method

    def calc(self, p_bar, t_c):
        """ recalculate all parameters according to some pressure and temperature"""
        self._p_bar = p_bar
        self._t_c = t_c
        return 1

    """ ========= default properties definition ======= """
    """ component density at specific conditions
        read only
    """
    @property
    def rho_kgm3(self):
        return self._rho_kgm3

    """ component density at standard condition 
        read only 
    """
    @property
    def rho_sckgm3(self):
        return self._gamma * self._relative_density_sckgm3

    """ component viscosity at standard condition
        read only
    """
    @property
    def mu_cp(self):
        return self._mu_cp

    """ component specific density at standard condition"""
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
        self._pseudo_pressure_mpa = 1
        self._pseudo_temperature_k = 1
        self.gamma = un.gamma_gas_default
        self._z = self._calc_z()               # сверхсжимаемость

    @property
    def z(self):
        return self._z

    def _calc_z(self, p_bar=un.psc_bar, t_c=un.tsc_c):
        self._z = un.z_default                 # default z constant set here
        return self._z

    def _calc_bg_m3m3(self, p_bar=un.psc_bar, t_c=un.tsc_c):
        self._fvf_m3m3 = un.psc_bar / p_bar
        return self._fvf_m3m3

    def _calc_mug_cp(self, p_bar=un.psc_bar, t_c=un.tsc_c):
        self._mu_cp = 1
        return self._mu_cp

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
    в текущем виде описывает упрощенные зависимости для свойств нефти - прямые линии и константы
    должен быть переопределен для учета более детальных свойств нефти с использованием корреляций
    """
    def __init__(self):
        super().__init__()              # часть базовых свойств наследуется
        self._gas = GasGeneral()        # create gas component
        self.rsb_m3m3 = un.rsb_default_m3m3

        self.pb_calibr_bar = 100        # калибровочное значение давления насыщения
        self.tb_calibr_c = 50           # температуры для калибровки по давлению насыщения
        self.bob_calibr_m3m3 = 1.2      # калибровочное значение объемного коэффициента
        self.muob_calibr_cp = 1         # калибровочное значение вязкости при давлении насыщения
        self.rhob_calibr_kgm3 = 700     # калибровочное значение плотности при давлении насыщения

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

    def _calc_rho_kgm3(self, p_bar, t_c):
        """ тут должна быть реализация расчета плотности нефти
            в упрощенном виде не зависит от температуры
        """
        if p_bar < self.pb_calibr_bar:
            return -self.rhob_calibr_kgm3 / self.pb_calibr_bar * p_bar + 1.8 * self.rhob_calibr_kgm3
        else:
            return self.rhob_calibr_kgm3

    def _calc_bo_m3m3(self, p_bar, t_c):
        """ тут должна быть реализация расчета объемного коэффициента нефти
        """
        if p_bar < self.pb_calibr_bar:
            return self.bob_calibr_m3m3 / self.pb_calibr_bar * p_bar
        else:
            return self.bob_calibr_m3m3

    def _calc_mu_cp(self, p_bar, t_c):
        """ тут должна быть реализация расчета вязкости нефти
        """
        if p_bar < self.pb_calibr_bar:
            return -self.muob_calibr_cp / self.pb_calibr_bar * p_bar + 2 * self.muob_calibr_cp
        else:
            return self.muob_calibr_cp

    def _calc_co_1atm(self, p_bar, t_c):
        """ тут должна быть реализация расчета сжимаемости нефти
        """
        return (28.1 * self.rsb_m3m3 + 30.6 * (t_c + 273) - 1180
                * self._gas.gamma + 1784 / self.gamma - 10910) \
                / (100000 * p_bar)

    def _calc_rs_m3m3(self, p_bar, t_c):
        """ тут должна быть реализация расчета газосодержания
        """
        if p_bar < self.pb_calibr_bar:
            return self.rsb_m3m3 / self.pb_calibr_bar * p_bar
        else:
            return self.rsb_m3m3

    def calc(self, p_atm, t_c):
        """ реализация расчета свойств нефти """
        self._rs_m3m3 = self._calc_rs_m3m3(p_atm, t_c)
        self._rho_kgm3 = self._calc_rho_kgm3(p_atm, t_c)
        self._bo_m3m3 = self._calc_bo_m3m3(p_atm, t_c)
        self._mu_cp = self._calc_mu_cp(p_atm, t_c)
        self._co_1atm = self._calc_co_1atm(p_atm, t_c)


class GasHC(GasGeneral):
    """
    класс реализующий расчет свойств газа на основе z фактора по Дранчуку
    """
    # TODO надо реализовать расчет свойств газа по аналогии с унифлокVBA
    pass

class OilStanding(OilGeneral):
    """
    класс реализующий расчет свойств нефти с использованием корреляции Стендинга (набор корреляций на основе Стендинга)
    """
    # TODO надо реализовать расчет свойств нефти по Стендингу по аналогии с унифлокVBA
    pass


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


if __name__ == "__main__":
    print("Вы запустили модуль напрямую, а не импортировали его.")
    input("\n\nНажмите Enter, чтобы выйти.")