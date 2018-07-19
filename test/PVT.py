# -*- coding: utf-8 -*-
"""
Created on Mon Sept 04 2017

@author: rnt

UniflocPy

класс для расчета PVT свойств углеводородных флюидов и воды

"""
import uconst as uc
import PVT_correlations as PVT


class FluidBlackOil:
    """
    Описание смеси нефти, газа и воды по модели нелетучей нефти
    
    Описание флюида - скважинной продукции
    Класс обеспечивает расчет свойств флюида при заданных термобарических условиях
    по одному набору корреляционных зависимостей
    классы потомки могут менять набор корреляционных зависимостей для уточнения расчета
    """
    _pb_cal: float

    def __init__(self, gamma_oil=0.86, gamma_gas=0.6, gamma_wat=1.0, rsb_m3m3=200.0, 
                 pbcal_bar=-1., tpb_C = 80):
        """
        создает флюид с заданными базовыми свойствами

        калибровочные параметры при необходимости надо задавать отдельно
        :param gamma_oil:
        :param gamma_gas:
        :param gamma_wat:
        :param rsb_m3m3:
        """
        self.gamma_gas = gamma_gas              # specific gravity of gas (by air), dimensionless
        self.gamma_oil = gamma_oil              # specific gravity of oil
        self.gamma_wat = gamma_wat              # specific gravity of water
        self.rsb_m3m3 = rsb_m3m3                # solution gas ratio at bubble point
        # термобарические условия
        self.p_bar = uc.psc_bar                 # thermobaric conditions for all parameters
        self.t_c = uc.tsc_c                     # can be set up by calc method
        # калибровочные параметры
        self.pbcal_bar = pbcal_bar            # давление насыщения, калибровочный параметр
        self.tpb_C = tpb_C
        self.bob_m3m3 = 0.0
        self.muob_cP = 0.0

        # расчетные параметры доступны только для чтения через свойства
        self._pb_bar = 0.0
        self.tpb_C = 0.0
        self._mu_oil_cP = 0.0
        self._mu_gas_cP = 0.0
        self._mu_wat_cP = 0.0
        self._rho_oil_kgm3 = 0.0
        self._rho_gas_kgm3 = 0.0
        self._rho_wat_kgm3 = 0.0
        self._rs_m3m3 = 0.0
        self._bo_m3m3 = 0.0
        self._bg_m3m3 = 0.0
        self._bw_m3m3 = 0.0
        self._z = 0.0
        self._compr_oil_1bar = 0.0
        self._compr_gas_1bar = 0.0
        self._compr_wat_1bar = 0.0
        self._heatcap_oil_ = 0.0
        self._heatcap_gas_ = 0.0
        self._heatcap_wat_ = 0.0
        self._sigma_oil_Nm = 0.0
        self._sigma_wat_Nm = 0.0

    # ========= default properties definition =======
    @property
    def pb_bar(self):
        """ return bubble point pressure an bar absolute"""
        return self._pb_bar

    @property
    def mu_oil_cP(self):
        """ return oil viscosity at working condition"""
        return self._mu_oil_cP

    @property
    def mu_gas_cP(self):
        """ return gas viscosity at working condition"""
        return self._mu_gas_cP

    @property
    def mu_wat_cP(self):
        """ return water viscosity at working condition"""
        return self._mu_wat_cP

    @property
    def rho_oil_kgm3(self):
        """ return oil density at working condition"""
        return self._rho_oil_kgm3

    @property
    def rho_gas_kgm3(self):
        """ return gas density at working condition"""
        return self._rho_gas_kgm3

    @property
    def rho_wat_kgm3(self):
        """ return water density at working condition"""
        return self._rho_wat_kgm3

    @property
    def rs_m3m3(self):
        """ return gas solution ratio at working condition"""
        return self._rs_m3m3

    @property
    def bo_m3m3(self):
        """ return oil formation volume factor at working condition"""
        return self._bo_m3m3

    @property
    def bg_m3m3(self):
        """ return gas formation volume factor at working condition"""
        return self._bg_m3m3

    @property
    def bw_m3m3(self):
        """ return water formation volume factor at working condition"""
        return self._bw_m3m3

    @property
    def z(self):
        """ return gas z factor (compressibility) at working condition"""
        return self._z

    @property
    def compr_oil_1bar(self):
        """ return oil compressibility at working condition"""
        return self._compr_oil_1bar

    @property
    def compr_gas_1bar(self):
        """ return gas compressibility at working condition"""
        return self._compr_gas_1bar

    @property
    def compr_wat_1bar(self):
        """ return water compressibility at working condition"""
        return self._compr_wat_1bar

    @property
    def heatcap_oil_(self):
        """ return oil heat capacity at working condition"""
        return self._heatcap_oil_

    @property
    def heatcap_gas_(self):
        """ return gas heat capacity at working condition"""
        return self._heatcap_gas_

    @property
    def heatcap_wat_(self):
        """ return water heat capacity at working condition"""
        return self._heatcap_wat_

    @property
    def sigma_oil_Nm(self):
        """ return oil surface tension  at working condition"""
        return self._sigma_oil_Nm

    @property
    def sigma_wat_Nm(self):
        """ return water surface tension at working condition"""
        return self._sigma_wat_Nm

    @property
    def rho_gas_sckgm3(self):
        """ component density at standard condition, read only """
        return self.gamma_gas * uc.air_density_sckgm3

    def calc(self, p_bar, t_c):
        """ recalculate all parameters according to some pressure and temperature"""
        self.p_bar = p_bar
        self.t_c = t_c
        return 1


class FluidStanding(FluidBlackOil):
    """
    класс реализующий расчет свойств нефти с использованием
    набора корреляций корреляций на основе Стендинга
    """

    def calc(self, p_bar, t_c):
        """Расчет свойств нефти"""
        return super().calc(p_bar, t_c)


if __name__ == "__main__":
    print("Вы запустили модуль напрямую, а не импортировали его.")
    input("\n\nНажмите Enter, чтобы выйти.")