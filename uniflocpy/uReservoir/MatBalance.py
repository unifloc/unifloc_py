import uniflocpy.uTools.data_workflow as data_workflow
import uniflocpy.uPVT.PVT_fluids as PVT
import uniflocpy.uTools.uconst as uc
import numpy as np
import scipy as sp


class MatBalance():

    def __init__(self, fluid=PVT.FluidStanding()):

        self.fluid = fluid

        self.p_reservoir_init_bar = None
        self.t_reservoir_init_c = None

        self.p_reservoir_bar = None
        self.t_reservoir_c = None

        self.N_cum_oil_recovery_m3 = None
        self.N_cum_gas_recovery_m3 = None
        self.N_cum_wat_recovery_m3 = None

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
        self.b_oil_m3m3 = None
        self.b_wat_init_m3m3 = None
        self.b_wat_m3m3 = None

        self.p_drop_bar = None
        self.oil_recovery_perc = None

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
