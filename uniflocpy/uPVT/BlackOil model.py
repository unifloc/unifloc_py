import uniflocpy.uTools.uconst as uc
import uniflocpy.uPVT.PVT_correlations as PVT
import math

class BlackOil_option():
    def __init__(self):
        self.pb_cor_number = 0
        self.rs_cor_number = 0
        self.compr_oil_cor_number = 0
        self.b_oil_in_pb_cor_number = 0
        self.b_oil_below_pb_cor_number = 0
        self.b_oil_above_pb_cor_number = 0
        self.mu_dead_oil_cor_number = 0
        self.mu_oil_pb_cor_number = 0
        self.mu_oil_any_p_cor_number = 0
        self.heatcap_oil_cor_number = 0
        self.thermal_conduct_oil_cor_number = 0

        self.pseudocritical_temperature_cor_number = 0
        self.pseudocritical_pressure_cor_number = 0
        self.z_cor_number = 0
        self.mu_gas_cor_number = 0
        self.b_gas_cor_number = 0
        self.compr_gas_cor_number = 0
        self.rho_gas_cor_number = 0
        self.heatcap_gas_cor_number = 0
        self.thermal_conduct_gas_cor_number = 0

        self.rho_wat_cor_number = 0
        self.compr_wat_cor_number = 0
        self.b_wat_cor_number = 0
        self.mu_wat_cor_number = 0
        self.rs_wat_cor_number = 0
        self.heatcap_wat_cor_number = 0
        self.thermal_conduct_wat_cor_number = 0
        self.thermal_expansion_wat_cor_number = 0
        self.sigma_oil_gas_cor_number = 0
        self.sigma_wat_gas_cor_number = 0






class Fluid:
    def __init__(self, gamma_oil=0.86, gamma_gas=0.6, gamma_wat=1.0, rsb_m3m3=200.0, gamma_gassp=0, y_h2s=0, y_co2=0,
                 y_n2=0, s_ppm=0, par_wat=0, pbcal_bar=-1., tpb_C=80, bobcal_m3m3=1.2, muobcal_cP=0.5 ):
        """
        Cоздает флюид с заданными базовыми свойствами

        калибровочные параметры при необходимости надо задавать отдельно

        :param gamma_oil:
        :param gamma_gas:
        :param gamma_wat:
        :param rsb_m3m3:
        :param gamma_gassp:
        :param y_h2s:
        :param y_co2:
        :param y_n2:
        :param s_ppm:
        :param par_wat:
        :param pbcal_bar:
        :param tpb_C:
        :param bobcal_m3m3:
        :param muobcal_cP:
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
        self._tpb_C = tpb_C
        self.bobcal_m3m3 = bobcal_m3m3
        self.muobcal_cP = muobcal_cP
        self.rho_oil_stkgm3 = gamma_oil * 1000  # TODO check?

        # расчетные параметры
        self._p_mpa = 0.0
        self._t_k = 0.0
        self._tpc_k = 0.0
        self._ppc_mpa = 0.0
        self._pb_bar = 0.0
        self._pb_mpa = 0.0
        self._bob_m3m3 = 0.0
        self._muob_cP = 0.0
        self._tpb_C = 0.0
        self._mu_oil_cP = 0.0        # TODO хорошо бы везде сделать единообразные индексы для нефти или o или oil
        self._mu_gas_cP = 0.0
        self._mu_wat_cP = 0.0
        self._mu_deadoil_cP = 0.0    # dead oil
        self._rho_oil_kgm3 = 0.0
        self._rho_gas_kgm3 = 0.0
        self._rho_wat_kgm3 = 0.0
        self._rs_m3m3 = 0.0
        self._rsw_m3m3 = 0.0   # TODO проверить - равен GWR?
        self._bo_m3m3 = 0.0
        self._bg_m3m3 = 0.0
        self._bw_m3m3 = 0.0
        self._z = 0.0
        self._compr_oil_1bar = 0.0
        self._compr_oil_1mpa = 0.0
        self._compr_gas_1bar = 0.0
        self._compr_wat_1bar = 0.0
        self._heatcap_oil_jkgc = 0.0
        self._heatcap_gas_jkgc = 0.0
        self._heatcap_wat_jkgc = 0.0
        self._sigma_oil_gas_Nm = 0.0
        self._sigma_wat_gas_Nm = 0.0
        self._thermal_conduct_oil_wmk = 0.0
        self._thermal_conduct_gas_wmk = 0.0
        self._thermal_conduct_wat_wmk = 0.0
        self._thermal_expansion_wat_1c = 0.0

        self.option = BlackOil_option()

    def calc_pb_MPaa(self, number_cor):
        if number_cor == 0:
            return PVT.unf_pb_Standing_MPaa(self.rsb_m3m3, self.gamma_oil, self.gamma_gas, self._t_k)
        if number_cor == 1:
            return PVT.unf_pb_Valko_MPaa(self.rsb_m3m3, self.gamma_oil, self.gamma_gas, self._t_k)
        if number_cor == 2:  # TODO check cor
            return PVT.unf_pb_Glaso_MPaa(self.rsb_m3m3, self._t_k, self.gamma_oil, self.gamma_gas)

    def calc_rs_m3m3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_rs_Standing_m3m3(self._p_mpa, self._pb_mpa, self.gamma_oil,
                                            self.gamma_gas, self._t_k)
        if number_cor == 1:
            return PVT.unf_rs_Velarde_m3m3(self._p_mpa, self._pb_mpa, self.gamma_oil,
                                           self.gamma_gas, self._t_k)

    def calc_compr_oil_1mpa(self, number_cor):
        if number_cor == 0:
            return PVT.unf_compressibility_oil_VB_1Mpa(self._rs_m3m3, self._t_k, self.gamma_oil,
                                                       self._p_mpa, self.gamma_gas)

    def calc_b_oil_in_pb_m3m3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_fvf_Standing_m3m3_saturated(self.rsb_m3m3, self.gamma_gas, self.gamma_oil, self._t_k)

    def calc_b_oil_above_pb_m3m3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_fvf_VB_m3m3_above(self._bob_m3m3, self._compr_oil_1mpa,
                                             self._pb_mpa, self._p_mpa)

    def calc_b_oil_below_pb_m3m3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_fvf_Standing_m3m3_saturated(self._rs_m3m3, self.gamma_gas, self.gamma_oil, self._t_k)
        if number_cor == 1:
            return PVT.unf_fvf_Glaso_m3m3_below(self._rs_m3m3, self._t_k, self.gamma_gas, self.gamma_oil, self._p_mpa)
        if number_cor == 2:
            return PVT.unf_fvf_Mccain_m3m3_below(self.rho_oil_stkgm3, self._rs_m3m3, self._rho_oil_kgm3,
                                                 self.gamma_gas)

    def calc_mu_dead_oil_cp(self, number_cor):
        if number_cor == 0:
            return PVT.unf_deadoilviscosity_Beggs_cP(self.gamma_oil, self._t_k)

    def calc_mu_oil_in_pb_cp(self, number_cor):
        if number_cor == 0:
            return PVT.unf_saturatedoilviscosity_Beggs_cP(self._mu_deadoil_cP, self.rsb_m3m3)

    def calc_mu_oil_any_p_cp(self, number_cor):
        if number_cor == 0:
            return PVT.unf_oil_viscosity_Beggs_VB_cP(self._mu_deadoil_cP, self._rs_m3m3,
                                                     self._p_mpa, self._pb_mpa)

    def calc_heatcap_oil_jkgc(self, number_cor):
        if number_cor == 0:
            return PVT.unf_heat_capacity_oil_Gambill_JkgC(self.gamma_oil, self.t_c)
        if number_cor == 1:
            return PVT.unf_heat_capacity_oil_Wes_Wright_JkgC(self.gamma_oil, self.t_c)

    def calc_thermal_conduct_oil_wmk(self, number_cor):
        if number_cor == 0:
            return PVT.unf_thermal_conductivity_oil_Cragoe_WmK(self.gamma_oil, self.t_c)
        if number_cor == 1:
            return PVT.unf_thermal_conductivity_oil_Abdul_Seoud_Moharam_WmK(self.gamma_oil, self.t_c)
        if number_cor == 2:
            return PVT.unf_thermal_conductivity_oil_Smith_WmK(self.gamma_oil, self.t_c)

    def calc_pseudocritical_temperature_k(self, number_cor):
        if number_cor == 0:
            return PVT.unf_pseudocritical_temperature_K(self.gamma_gas, self.y_h2s, self.y_co2, self.y_n2)

    def calc_pseudocritical_pressure_mpa(self, number_cor):
        if number_cor == 0:
            return PVT.unf_pseudocritical_pressure_MPa(self.gamma_gas, self.y_h2s, self.y_co2, self.y_n2)

    def calc_zfactor(self, number_cor):
        if number_cor == 0:
            return PVT.unf_zfactor_DAK(self._p_mpa, self._t_k, self._ppc_mpa, self._tpc_k)

    def calc_mu_gas_cp(self, number_cor):
        if number_cor == 0:
            return PVT.unf_gasviscosity_Lee_cP(self._t_k, self._p_mpa, self._z, self.gamma_gas)

    def calc_b_gas_m3m3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_gas_fvf_m3m3(self._t_k, self._p_mpa, self._z)

    def calc_compr_gas_1bar(self, number_cor):
        if number_cor == 0:
            return uc.compr_1mpa_2_1bar(PVT.unf_compressibility_gas_Mattar_1MPa(self._p_mpa, self._t_k,
                                                                                self._ppc_mpa, self._tpc_k))

    def calc_rho_gas_kgm3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_gas_density_kgm3(self._t_k, self._p_mpa, self.gamma_gas, self._z)

    def calc_heatcap_gas_jkgc(self, number_cor):
        if number_cor == 0:
            return PVT.unf_heat_capacity_gas_Mahmood_Moshfeghian_JkgC(self._p_mpa, self._t_k,
                                                                      self.gamma_gas)

    def calc_thermal_conduct_gas_wmk(self, number_cor):
        if number_cor == 0:
            return PVT.unf_thermal_conductivity_gas_methane_WmK(self.t_c)

    def calc_rho_wat_kgm3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_density_brine_Spivey_kgm3(self._t_k, self._p_mpa, self.s_ppm, self.par_wat)

    def calc_compr_wat_1bar(self, number_cor):
        if number_cor == 0:
            return uc.compr_1mpa_2_1bar(PVT.unf_compressibility_brine_Spivey_1MPa(self._t_k, self._p_mpa, self.s_ppm,
                                                                                  self._z, self.par_wat))

    def calc_b_wat_m3m3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_fvf_brine_Spivey_m3m3(self._t_k, self._p_mpa, self.s_ppm)

    def calc_mu_wat_cp(self, number_cor):
        if number_cor == 0:
            return PVT.unf_viscosity_brine_MaoDuan_cP(self._t_k, self._p_mpa, self.s_ppm)

    def calc_rs_wat_m3m3(self, number_cor):
        if number_cor == 0:
            return PVT.unf_gwr_brine_Spivey_m3m3(self.s_ppm, self._z)

    def calc_heatcap_wat_jkgc(self, number_cor):
        if number_cor == 0:
            return PVT.unf_heat_capacity_water_IAPWS_JkgC(self.t_c)

    def calc_thermal_conduct_wat_wmk(self, number_cor):
        if number_cor == 0:
            return PVT.unf_thermal_conductivity_water_IAPWS_WmC(self.t_c)

    def calc_thermal_expansion_wat_1c(self, number_cor):
        if number_cor == 0:
            return PVT.unf_thermal_expansion_coefficient_water_IAPWS_1C(self.t_c)

    def calc_sigma_oil_gas_Nm(self, number_cor):
        if number_cor == 0:
            return PVT.unf_surface_tension_go_Baker_Swerdloff_Nm(self._t_k, self.gamma_oil, self._p_mpa)
        if number_cor == 1:
            return PVT.unf_surface_tension_go_Abdul_Majeed_Nm(self._t_k, self.gamma_oil, self._rs_m3m3)
    def calc_sigma_wat_gas_Nm(self, number_cor):
        if number_cor == 0:
            return PVT.unf_surface_tension_gw_Sutton_Nm(self._rho_wat_kgm3, self._rho_gas_kgm3, self.t_c)


    def calc(self, p_bar, t_c):
        self.p_bar = p_bar
        self.t_c = t_c
        self._t_k = uc.c2k(self.t_c)
        self._p_mpa = uc.bar2MPa(self.p_bar)
        # oil
        self._pb_mpa = self.calc_pb_MPaa(self.option.pb_cor_number)
        self._pb_bar = uc.MPa2bar(self._pb_mpa)
        self._rs_m3m3 = self.calc_rs_m3m3(self.option.rs_cor_number)
        self._compr_oil_1mpa = self.calc_compr_oil_1mpa(self.option.compr_oil_cor_number)
        self._compr_oil_1bar = uc.compr_1mpa_2_1bar(self._compr_oil_1mpa)
        self._bob_m3m3 = self.calc_b_oil_in_pb_m3m3(self.option.b_oil_in_pb_cor_number)
        if self._p_mpa > self._pb_mpa:
            self._bo_m3m3 = self.calc_b_oil_above_pb_m3m3(self.option.b_oil_above_pb_cor_number)
        else:
            self._bo_m3m3 = self.calc_b_oil_below_pb_m3m3(self.option.b_oil_below_pb_cor_number)
        self._mu_deadoil_cP = self.calc_mu_dead_oil_cp(self.option.mu_dead_oil_cor_number)
        self._muob_cP = self.calc_mu_oil_in_pb_cp(self.option.mu_oil_pb_cor_number)
        self._mu_oil_cP = self.calc_mu_oil_any_p_cp(self.option.mu_oil_any_p_cor_number)
        self._heatcap_oil_jkgc = self.calc_heatcap_oil_jkgc(self.option.heatcap_oil_cor_number)
        self._thermal_conduct_oil_wmk = self.calc_thermal_conduct_oil_wmk(self.option.thermal_conduct_oil_cor_number)
        # gas
        self._tpc_k = self.calc_pseudocritical_temperature_k(self.option.pseudocritical_temperature_cor_number)
        self._ppc_mpa = self.calc_pseudocritical_pressure_mpa(self.option.pseudocritical_pressure_cor_number)
        self._z = self.calc_zfactor(self.option.z_cor_number)
        self._mu_gas_cP = self.calc_mu_gas_cp(self.option.mu_gas_cor_number)
        self._bg_m3m3 = self.calc_b_gas_m3m3(self.option.b_gas_cor_number)
        self._compr_gas_1bar = self.calc_compr_gas_1bar(self.option.compr_gas_cor_number)
        self._rho_gas_kgm3 = self.calc_rho_gas_kgm3(self.option.rho_gas_cor_number)
        self._heatcap_gas_jkgc = self.calc_heatcap_gas_jkgc(self.option.heatcap_gas_cor_number)
        self._thermal_conduct_gas_wmk = self.calc_thermal_conduct_gas_wmk(self.option.thermal_conduct_gas_cor_number)
        # water
        self._rho_wat_kgm3 = self.calc_rho_wat_kgm3(self.option.rho_wat_cor_number)
        self._compr_wat_1bar = self.calc_compr_gas_1bar(self.option.compr_gas_cor_number)
        self._bw_m3m3 = self.calc_b_wat_m3m3(self.option.b_wat_cor_number)
        self._mu_wat_cP = self.calc_mu_wat_cp(self.option.mu_wat_cor_number)
        self._rsw_m3m3 = self.calc_rs_wat_m3m3(self.option.rs_wat_cor_number)
        self._heatcap_wat_jkgc = self.calc_heatcap_wat_jkgc(self.option.heatcap_wat_cor_number)
        self._thermal_conduct_wat_wmk = self.calc_thermal_conduct_wat_wmk(self.option.thermal_conduct_wat_cor_number)
        self._thermal_expansion_wat_1c = self.calc_thermal_expansion_wat_1c(self.option.thermal_expansion_wat_cor_number)
        # some system properties
        self._sigma_oil_gas_Nm = self.calc_sigma_oil_gas_Nm(self.option.sigma_oil_gas_cor_number)
        self._sigma_wat_gas_Nm = self.calc_sigma_wat_gas_Nm(self.option.sigma_wat_gas_cor_number)

black_oil_model = Fluid()

pb_mpaa = black_oil_model.calc(30,30)
print(pb_mpaa)
print(black_oil_model.__dict__)
