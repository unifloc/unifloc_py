import uniflocpy.uTools.data_workflow as data_workflow
import uniflocpy.uPVT.PVT_fluids as PVT
import uniflocpy.uTools.uconst as uc
import numpy as np
import scipy as sp


class MatBalans():

    def __init__(self, fluid=PVT.FluidStanding()):

        self.fluid = fluid

        self.p_reservoir_init_bar = None
        self.t_reservoir_init_c = None

        self.p_reservoir_bar = None
        self.t_reservoir_c = None

        self.N_cum_oil_recovery_m3 = None
        self.N_cum_gas_recovery_m3 = None
        self.N_cum_wat_recovery_m3 = None

        self.STOIIP_m3 = None

        self.porosity_m = None

        self.S_wat_connate_d = None

        self.c_oil_1bar = None
        self.c_gas_1bar = None
        self.c_wat_1bar = None
        self.c_res_1bar = None
        self.c_eff_1bar = None

        self.b_oil_init_m3m3 = None
        self.b_oil_m3m3 = None
        self.b_wat_init_m3m3 = None
        self.b_wat_m3m3 = None

        self.p_drop_bar = None

    def calc_depletion_above_pb(self, p_reservoir_bar, N_cum_oil_recovery_m3, N_cum_wat_recovery_m3):
        self.p_reservoir_bar = p_reservoir_bar
        self.p_drop_bar = self.p_reservoir_init_bar - self.p_reservoir_bar

        self.N_cum_oil_recovery_m3 = N_cum_oil_recovery_m3
        self.N_cum_wat_recovery_m3 = N_cum_wat_recovery_m3

        self.fluid.calc(self.p_reservoir_init_bar, self.t_reservoir_init_c)
        self.b_oil_init_m3m3 = self.fluid.bo_m3m3
        self.b_wat_init_m3m3 = self.fluid.bw_m3m3

        self.fluid.calc(self.p_reservoir_bar, self.t_reservoir_c)  # TODO разграничить флюиды на 2 экземпляра
        self.b_oil_m3m3 = self.fluid.bo_m3m3
        self.b_wat_m3m3 = self.fluid.bw_m3m3

        self.c_oil_1bar = (self.b_oil_m3m3 - self.b_oil_init_m3m3) / self.b_oil_init_m3m3 / self.p_drop_bar
        self.c_wat_1bar = (self.b_wat_m3m3 - self.b_wat_init_m3m3) / self.b_wat_init_m3m3 / self.p_drop_bar
        self.c_eff_1bar = (self.c_oil_1bar * (1 - self.S_wat_connate_d) + self.c_wat_1bar * self.S_wat_connate_d + self.c_res_1bar)

        self.STOIIP_m3 = ((self.N_cum_oil_recovery_m3 * self.b_oil_m3m3 + self.N_cum_wat_recovery_m3 * self.b_wat_m3m3) /
                          self.b_oil_init_m3m3 * self.p_drop_bar * self.c_eff_1bar)







