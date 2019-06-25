"""
Кобзарь О.С. Хабибуллин Р.А.
Температурная корреляция в форме градиента
"""

import math
from scipy.optimize import fsolve
import uniflocpy.uTools.uconst as uc

const_g_m2sec = uc.g
pi = math.pi


class Hasan_Kabir_cor():
    def __init__(self):

        self.rho_earth_kgm3 = 2504
        self.cp_earth_jkgc = 1256
        self.thermal_conduct_earth_wmc = 2.422

        self.thermal_conduct_tube_wmk = 25
        self.thermal_conduct_cas_wmk = self.thermal_conduct_tube_wmk
        self.thermal_conduct_cem_wmc = 0.779

        self.delta_t_an_init_c = 3

        self.section_casing = None  # if Tube: self.section_casing = False

        self.time_sec = None

        self.p_bar = None
        self.t_c = None

        self.t_earth_init_c = None

        # конструкция скважины
        self.angle_rad = None
        self.d_m = None

        self.r_tube_in_m = None
        self.r_tube_out_m = None
        self.r_cas_in_m = None
        self.r_cas_out_m = None
        self.r_well_m = None

        # свойства многофазного потока
        self.liquid_content = None
        self.mass_flowraten_kgsec = None
        self.vm_msec = None

        self.sigma_liq_Nm = None
        self.rhon_kgm3 = None
        self.mun_cP = None
        self.heatcapn_jkgc = None
        self.thermal_conductn_wmk = None

        self.number_re_n = None
        self.number_pr_n = None

        # свойства флюида, находящегося в затрубном пространстве
        self.mu_an_cP = None
        self.heatcap_an_jkgc = None
        self.thermal_conduct_an_wmk = None
        self.rho_an_kgm3 = None
        self.heat_expansion_an_1c = None

        # параметры, входящие в градиент
        self.Joule_Thompson_coef_cpa = None
        self.grad_p_pam = None
        self.grad_v_msecm = None

        # рассчитанные параметры
        self.number_nu = None

        self.number_pr_an = None
        self.number_gr_an = None

        self.time_dimensionless = None
        self.heat_dissipation_to_earth = None

        self.hf_wm2c = None
        self.han_wm2c = None

        self.delta_t_an_c = None

        self.thermal_resistance_conv_flow = None
        self.thermal_resistance_cond_tube = None
        self.thermal_resistance_conv_an = None
        self.thermal_resistance_cond_cas = None
        self.thermal_resistance_cond_cem = None
        self.thermal_resistance_cond_earth = None

        self.overall_heat_transfer_coef_wm2c = None

        self.relaxation_parametr = None

        self.heat_flowrate = None

        self.part_grad_JT = None
        self.part_grad_flow = None
        self.part_grad_kinetic = None
        self.part_grad_potential = None
        self.grad_t_cm = None

        self.heat_transfer_rate = None

    def calc_annulus(self, delta_t_an_c):
        """
        Расчет затрубного пространства
        :param delta_t_an_c: начальное приближения для перепада температур в затрубном пространстве, С
        :return: разница между начальным приближением и вычисленным результатом - для fsolve
        """
        self.delta_t_an_c = delta_t_an_c
        self.number_pr_an = uc.cP2pasec(self.mu_an_cP) * self.heatcap_an_jkgc / self.thermal_conduct_an_wmk
        self.number_gr_an = (self.r_cas_in_m - self.r_tube_out_m) ** 3 * self.rho_an_kgm3 ** 2 * \
                            self.heat_expansion_an_1c * uc.g * self.delta_t_an_c / uc.cP2pasec(self.mu_an_cP) ** 2

        self.han_wm2c = (0.049 * (self.number_gr_an * self.number_pr_an) ** (1 / 3) *
                         self.number_pr_an ** 0.074 * self.thermal_conduct_an_wmk) / \
                        (self.r_tube_out_m * math.log(self.r_cas_in_m / self.r_tube_out_m))
        self.han_wm2c = self.han_wm2c * 0.25

        self.thermal_resistance_conv_an = 1 / (self.r_cas_in_m * self.han_wm2c)
        self.overall_heat_transfer_coef_wm2c = 1 / self.r_tube_out_m * (1 /
                                                                        (self.thermal_resistance_conv_flow +
                                                                         self.thermal_resistance_cond_tube +
                                                                         self.thermal_resistance_conv_an +
                                                                         self.thermal_resistance_cond_cas +
                                                                         self.thermal_resistance_cond_cem +
                                                                         self.thermal_resistance_cond_earth))
        self.heat_transfer_rate = 2 * uc.pi * self.r_tube_out_m * self.overall_heat_transfer_coef_wm2c * \
                                  (self.t_c - self.t_earth_init_c)

        self.delta_t_an_c = self.heat_transfer_rate / (2 * uc.pi * self.r_tube_in_m * self.han_wm2c)
        return  float(self.delta_t_an_c - delta_t_an_c)

    def calc_grad_t_cm(self, p_bar, t_c):
        self.p_bar = p_bar
        self.t_c = t_c

        self.number_nu = 0.023 * self.number_re_n ** 0.8 * self.number_pr_n ** 0.3
        self.hf_wm2c = self.number_nu * self.thermal_conductn_wmk / self.d_m

        self.time_dimensionless = (self.thermal_conduct_earth_wmc * self.time_sec /
                                  (self.rho_earth_kgm3 * self.cp_earth_jkgc * self.r_well_m ** 2))
        if self.time_dimensionless <= 1.5:
            self.heat_dissipation_to_earth = 1.1281 * math.sqrt(self.time_dimensionless) * \
                                             (1 - 0.3 * math.sqrt(self.time_dimensionless))
        elif self.time_dimensionless > 1.5:
            self.heat_dissipation_to_earth = (0.4063 + 1 / 2 * math.log(self.time_dimensionless)) * \
                                             (1 + 0.6 / self.time_dimensionless)

        self.thermal_resistance_conv_flow = 1 / (self.d_m / 2 * self.hf_wm2c)

        self.thermal_resistance_cond_cas = math.log(self.r_cas_out_m / self.r_cas_in_m) / \
                                            self.thermal_conduct_cas_wmk
        self.thermal_resistance_cond_cem = math.log(self.r_well_m / self.r_cas_out_m) / \
                                            self.thermal_conduct_cem_wmc

        self.thermal_resistance_cond_earth = self.heat_dissipation_to_earth / self.thermal_conduct_earth_wmc

        if self.section_casing == True:  # TODO подумать, как правильно перейти
            self.overall_heat_transfer_coef_wm2c = 1 / self.r_cas_out_m * (1 /
                                                                        (self.thermal_resistance_conv_flow +
                                                                         self.thermal_resistance_cond_cas +
                                                                         self.thermal_resistance_cond_cem +
                                                                         self.thermal_resistance_cond_earth))
        else:
            self.thermal_resistance_cond_tube = math.log(self.r_tube_out_m / self.r_tube_in_m) / \
                                                self.thermal_conduct_tube_wmk
            self.result_an_fsolve = fsolve(self.calc_annulus, self.delta_t_an_init_c)

        self.relaxation_parametr = 2 * uc.pi / self.heatcapn_jkgc / self.mass_flowraten_kgsec * \
                                   (self.r_tube_out_m * self.overall_heat_transfer_coef_wm2c * self.thermal_conduct_earth_wmc/
                                    (self.thermal_conduct_earth_wmc +
                                     self.r_tube_out_m * self.overall_heat_transfer_coef_wm2c * self.heat_dissipation_to_earth))

        #self.heat_flowrate = - self.relaxation_parametr * self.mass_flowraten_kgsec * \
        #                     (self.t_c - self.t_earth_init_c)

        self.heat_flowrate = - self.relaxation_parametr * self.mass_flowraten_kgsec * \
                             self.heatcapn_jkgc * (self.t_c - self.t_earth_init_c)

        self.part_grad_JT = - self.Joule_Thompson_coef_cpa * self.grad_p_pam
        #self.part_grad_flow = 1 / self.heatcapn_jkgc * self.heat_flowrate / self.mass_flowraten_kgsec
        #self.part_grad_flow = (self.t_c - self.t_earth_init_c) * self.relaxation_parametr
        self.part_grad_flow = - 1 / self.heatcapn_jkgc * self.heat_flowrate / self.mass_flowraten_kgsec
        self.part_grad_potential = 1 / self.heatcapn_jkgc * uc.g * math.sin(self.angle_rad)
        self.part_grad_kinetic = 1 / self.heatcapn_jkgc * self.vm_msec * self.grad_v_msecm

        """self.grad_t_cm = self.Joule_Thompson_coef_cpa * self.grad_p_pam + 1 / self.heatcapn_jkgc * \
                         (self.heat_flowrate / self.mass_flowraten_kgsec +
                          uc.g * math.sin(self.angle_rad) -
                          self.vm_msec * self.grad_v_msecm)"""
        self.grad_t_cm = self.part_grad_JT + self.part_grad_flow + self.part_grad_potential + self.part_grad_kinetic

        return float(self.grad_t_cm)



