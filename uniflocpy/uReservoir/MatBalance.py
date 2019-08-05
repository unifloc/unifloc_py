"""
Водопьян А.О. Кобзарь О.С. Хабибуллин Р.А. 03.08.2019.

Модуль для расчета материального баланса приминительно к нефтяным и газовым залежам
"""

import uniflocpy.uTools.data_workflow as data_workflow
import uniflocpy.uPVT.PVT_fluids as PVT
import uniflocpy.uTools.uconst as uc
import numpy as np
from scipy.optimize import fsolve


class MatBalance():

    def __init__(self, fluid=PVT.FluidStanding(), p_reservoir_init_bar=250, t_reservoir_init_c=80):

        self.fluid = fluid

        self.p_reservoir_init_bar = p_reservoir_init_bar
        self.t_reservoir_init_c = t_reservoir_init_c

        self.p_reservoir_bar = None
        self.t_reservoir_c = None

        self.N_cum_oil_recovery_m3 = None
        self.N_cum_gas_recovery_m3 = None
        self.N_cum_wat_recovery_m3 = None
        self.rp_m3m3 = None

        self.STOIIP_by_MB_m3 = None
        self.STOIIP_by_VOL_m3 = None

        self.porosity_m = None

        self.S_wat_connate_d = None
        self.S_oil_d = None

        self.c_oil_1bar = None
        self.c_wat_1bar = None # TODO при расчете отрицательное (нефтяным методом), проверить почему
        self.c_res_1bar = None
        self.c_eff_1bar = None

        self.b_oil_init_m3m3 = None
        self.b_wat_init_m3m3 = None
        self.b_gas_init_m3m3 = None
        self.rs_init_m3m3 = None

        self.b_oil_m3m3 = None
        self.b_wat_m3m3 = None
        self.b_gas_m3m3 = None
        self.rs_m3m3 = None

        self.p_drop_bar = None
        self.oil_recovery_perc = None

        self.material_balance_m3 = None

    def calc_depletion_above_pb(self, p_reservoir_bar): # TODO придумать формульный конструктор
        """
        Функция рассчитывает КИН при истощении выше давления насыщения и добыче чистой нефти
        Функция служит учебным примером.

        :param p_reservoir_bar: конечное давление пласта при разработке (давление насыщения), бар
        :return: None
        """
        self.p_reservoir_bar = p_reservoir_bar
        self.p_drop_bar = self.p_reservoir_init_bar - self.p_reservoir_bar

        self.fluid.calc(self.p_reservoir_init_bar, self.t_reservoir_init_c)
        self.b_oil_init_m3m3 = self.fluid.bo_m3m3
        self.b_wat_init_m3m3 = self.fluid.bw_m3m3

        self.fluid.calc(self.p_reservoir_bar, self.t_reservoir_c)  # TODO разграничить флюиды на 2 экземпляра
        self.b_oil_m3m3 = self.fluid.bo_m3m3
        self.b_wat_m3m3 = self.fluid.bw_m3m3

        self.S_oil_d = 1 - self.S_wat_connate_d

        self.c_oil_1bar = (self.b_oil_m3m3 - self.b_oil_init_m3m3) / self.b_oil_init_m3m3 / self.p_drop_bar
        self.c_eff_1bar = ((self.c_oil_1bar * self.S_oil_d + self.c_wat_1bar * self.S_wat_connate_d + self.c_res_1bar) /
                          (1 - self.S_wat_connate_d))

        self.oil_recovery_perc = self.b_oil_init_m3m3 * self.p_drop_bar * self.c_eff_1bar / self.b_oil_m3m3 * 100



    def __material_balance_for_fsolve__(self, p_reservoir_bar):
        """
        Внутренняя функция для fsolve, позволяющая решать уравнение итерациями

        TODO можно ли сделать возможность учета изменения температуры пласта?

        :param p_reservoir_bar: пластовое давление, бар
        :return: material_balance_m3, разницу левой и правой части уравнения мат. баланса, должно быть равно нулю
        """

        self.p_reservoir_bar = float(p_reservoir_bar)
        self.fluid.calc(self.p_reservoir_bar, self.t_reservoir_init_c)
        self.b_oil_m3m3 = self.fluid.bo_m3m3
        self.b_wat_m3m3 = self.fluid.bw_m3m3
        self.b_gas_m3m3 = self.fluid.bg_m3m3
        self.rs_m3m3 = self.fluid.rs_m3m3

        self.p_drop_bar = self.p_reservoir_init_bar - self.p_reservoir_bar

        self.left_part_oil_m3 = self.N_cum_oil_recovery_m3 * self.b_oil_m3m3
        self.left_part_free_gas_m3 = self.N_cum_oil_recovery_m3 * (self.rp_m3m3 - self.rs_m3m3) * self.b_gas_m3m3
        self.left_side_underground_withdrawal_m3 = self.left_part_oil_m3 + self.left_part_free_gas_m3

        self.right_part_oil_expansion_m3 = self.STOIIP_by_VOL_m3 * ((self.b_oil_m3m3 - self.b_oil_init_m3m3) +
                                                                    (self.rs_init_m3m3 - self.rs_m3m3) * self.b_gas_m3m3)
        self.right_part_con_wat_expansion_m3 = (self.STOIIP_by_VOL_m3 * self.b_oil_init_m3m3 *
                                                (self.c_wat_1bar * self.S_wat_connate_d + self.c_res_1bar) *
                                                self.p_drop_bar) / (1 - self.S_wat_connate_d)
        self.right_side_system_expansion_m3 = self.right_part_oil_expansion_m3 + self.right_part_con_wat_expansion_m3

        self.material_balance_m3 = self.right_side_system_expansion_m3 - self.left_side_underground_withdrawal_m3

        return self.material_balance_m3

    def calc_depletion_above_and_below_pb(self, N_cum_oil_recovery_m3):

        self.N_cum_oil_recovery_m3 = N_cum_oil_recovery_m3

        self.fluid.calc(self.p_reservoir_init_bar, self.t_reservoir_init_c)
        self.b_oil_init_m3m3 = self.fluid.bo_m3m3
        self.b_wat_init_m3m3 = self.fluid.bw_m3m3
        self.b_gas_init_m3m3 = self.fluid.bg_m3m3
        self.rs_init_m3m3 = self.fluid.rs_m3m3

        #p_0_reservoir_bar = self.p_reservoir_init_bar - 248
        p_0_reservoir_bar = 1.1
        self.p_reservoir_bar = fsolve(self.__material_balance_for_fsolve__, p_0_reservoir_bar)


p_res_init_bar = 250
t_res_init_c = 80
r_drainage_m = 250
porosity_d = 0.25
q_oil_surf_m3day = 50
h_eff_res_m = 8
c_system_1bar = 7.5 * 10 ** (-5)
t_end_year = 1
t_step_days = 30.33
S_wat_connate_d = 0.25

fluid = PVT.FluidStanding()
fluid.pbcal_bar = 100
fluid.rsb_m3m3 = 100
fluid.calc(p_res_init_bar, t_res_init_c)


STOIIP_by_VOL_m3 = uc.pi * r_drainage_m ** 2 * h_eff_res_m * porosity_d * (1 - S_wat_connate_d) / fluid.bo_m3m3

N_cum_oil_recovery_m3 = q_oil_surf_m3day * t_step_days * 15

MB = MatBalance()

MB.fluid = fluid
MB.rp_m3m3 = MB.fluid.rs_m3m3
MB.STOIIP_by_VOL_m3 = STOIIP_by_VOL_m3
c_wat_1bar = 4.35 * 10 ** (-5)
c_res_1bar = 7.25 * 10 ** (-5)

MB.c_wat_1bar = c_wat_1bar
MB.c_res_1bar = c_res_1bar

MB.t_reservoir_init_c = t_res_init_c
MB.p_reservoir_init_bar = p_res_init_bar
MB.S_wat_connate_d = S_wat_connate_d

MB.calc_depletion_above_and_below_pb(N_cum_oil_recovery_m3)
print(MB.fluid.pb_bar)
print(MB.material_balance_m3)
print(MB.__dict__)
