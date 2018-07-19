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

    def __init__(self, gamma_oil=0.86, gamma_gas=0.6, gamma_wat=1.0, rsb_m3m3=200.0, gamma_gassp=0, y_h2s=0, y_co2=0,
                 y_n2=0, s_ppm=0, par_wat=0, pbcal_bar=-1., tpb_C = 80):
        """
        создает флюид с заданными базовыми свойствами

        калибровочные параметры при необходимости надо задавать отдельно
        :param gamma_oil:
        :param gamma_gas:
        :param gamma_wat:
        :param rsb_m3m3:
        :param gamma_gassp:
        """
        self.gamma_gas = gamma_gas              # specific gravity of gas (by air), dimensionless
        self.gamma_oil = gamma_oil              # specific gravity of oil
        self.gamma_wat = gamma_wat              # specific gravity of water
        self.rsb_m3m3 = rsb_m3m3                # solution gas ratio at bubble point
        self.gamma_gassp = gamma_gassp          # specific gas density in separator(by air)
        self.y_h2s = y_h2s                      # mole fraction of the hydrogen sulfide
        self.y_co2 = y_co2                      # mole fraction of the carbon dioxide
        self.y_n2 = y_n2                        # mole fraction of the nitrogen
        self.s_ppm = s_ppm                      # water salinity
        self.par_wat = par_wat                  # 0 = pure water, 1 = brine , 2 = brine with dissolved methane
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
        p_MPaa = uc.bar2Pa(p_bar) / 10 ** 6
        t_K = uc.c2k(t_c)
        # oil
        pb_MPaa = PVT.unf_pb_Valko_MPaa(self.rsb_m3m3, self.gamma_oil, self.gamma_gas, t_K)
        pbcal_MPaa = uc.bar2Pa(self.pbcal_bar)
        # TODO необходимо будет сделать калибровку при любом P,T любого свойства
        if pbcal_MPaa > 0:
            p_fact = pb_MPaa / pbcal_MPaa
        else:
            p_fact = 1
        p_MPaa = p_fact * p_MPaa
        # внутри rs есть проверка на давление насыщения
        rs_m3m3 = PVT.unf_rs_Velarde_m3m3(p_MPaa, pb_MPaa, self.rsb_m3m3, self.gamma_oil, self.gamma_gas, t_K)
        co_1MPa = PVT.unf_compressibility_oil_VB_1Mpa(rs_m3m3, t_K, self.gamma_oil, p_MPaa, self.gamma_gas)
        # внутри rho есть проверка на давление насыщения
        rho_oil_kgm3 = PVT.unf_density_oil_Mccain(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, self.gamma_gas, t_K, self.gamma_oil
                                                  , self.gamma_gassp)
        density_oilsto_kgm3 = self.gamma_oil * 1000
        bo_m3m3 = PVT.unf_fvf_Mccain_m3m3_below(density_oilsto_kgm3, rs_m3m3, rho_oil_kgm3, self.gamma_gas)
        if p_MPaa > pb_MPaa:
            bo_m3m3 = PVT.unf_fvf_VB_m3m3_above(bo_m3m3, co_1MPa, pb_MPaa, p_MPaa)
        bob_m3m3 = PVT.unf_fvf_Mccain_m3m3_below(density_oilsto_kgm3, self.rsb_m3m3, rho_oil_kgm3, self.gamma_gas)
        if self.bob_m3m3 > 0:
            b_fact = (bob_m3m3 - 1) / (self.bob_m3m3 -1)
        else:
            b_fact = 1
        bo_m3m3 = b_fact * bo_m3m3
        deadoilviscosity_cP = PVT.unf_deadoilviscosity_Beggs_cP(self.gamma_oil, t_K)
        saturatedoilviscosity_cP = PVT.unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, self.rsb_m3m3)
        if self.muob_cP > 0:
            mu_fact = self.muob_cP / saturatedoilviscosity_cP
        else:
            mu_fact = 1
        mu_oil_cP = mu_fact * PVT.unf_oil_viscosity_Beggs_VB_cP(deadoilviscosity_cP, rs_m3m3, p_MPaa, pb_MPaa)
        # gas
        tpc_K = PVT.unf_pseudocritical_temperature_K(self.gamma_gas, self.y_h2s, self.y_co2, self.y_n2)
        ppc_MPa = PVT.unf_pseudocritical_pressure_MPa(self.gamma_gas, self.y_h2s, self.y_co2, self.y_n2)
        z = PVT.unf_zfactor_DAK(p_MPaa, t_K, ppc_MPa, tpc_K)
        mu_gas_cP = PVT.unf_gasviscosity_Lee_cP(t_K, p_MPaa, z, self.gamma_gas)
        bg_m3m3 = PVT.unf_gas_fvf_m3m3(t_K, p_MPaa, z)
        cg_1MPa = PVT.unf_compressibility_gas_Mattar_1MPa(p_MPaa, t_K, ppc_MPa, tpc_K)
        # water
        rho_wat_kgm3 = PVT.unf_density_brine_Spivey_kgm3(t_K, p_MPaa, self.s_ppm, self.par_wat)
        cw_1MPa = PVT.unf_compressibility_brine_Spivey_1MPa(t_K, p_MPaa, self.s_ppm, z, self.par_wat)
        bw_m3m3 = PVT.unf_fvf_brine_Spivey_m3m3(t_K, p_MPaa, self.s_ppm)
        mu_wat_cP = PVT.unf_viscosity_brine_MaoDuan_cP(t_K, p_MPaa, self.s_ppm)
        # output (для понимания отдельно внизу вынесено)
        # oil
        self._pb_bar = uc.Pa2bar(pb_MPaa / p_fact * 10 ** 6)
        self._rs_m3m3 = rs_m3m3
        self._mu_oil_cP = mu_oil_cP
        self._bo_m3m3 = bo_m3m3
        self._rho_oil_kgm3 = rho_oil_kgm3
        self._compr_oil_1bar = uc.compr_1pa_2_1bar(co_1MPa / 10 ** 6)
        # gas
        self._mu_gas_cP = mu_gas_cP
        self._z = z
        self._compr_gas_1bar = uc.compr_1pa_2_1bar(cg_1MPa / 10 ** 6)
        self._bg_m3m3 = bg_m3m3
        # brine
        self._rho_wat_kgm3 = rho_wat_kgm3
        self._compr_wat_1bar = uc.compr_1pa_2_1bar(cw_1MPa / 10 ** 6)
        self._mu_wat_cP = mu_wat_cP
        self._bw_m3m3 = bw_m3m3
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
