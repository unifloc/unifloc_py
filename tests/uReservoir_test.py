"""
Модуль для тестирования пакета uReservoir
"""
import unittest
import uniflocpy.uReservoir.MatBalance as MatBalance
import uniflocpy.uPVT.PVT_fluids as PVT
import uniflocpy.uTools.uconst as uc

class TestMatBalance(unittest.TestCase):
    def test_calc_depletion_above_pb(self):
        c_oil_1bar = 14.5 * 10 ** (-5)
        c_wat_1bar = 4.35 * 10 ** (-5)
        c_res_1bar = 7.25 * 10 ** (-5)

        p_res_init_bar = 250
        p_drop_bar = 103
        t_res_init_c = 80
        t_res_c = 80
        S_wat_connate_d = 0.25
        p_reservoir_bar = pbcal_bar = p_res_init_bar - p_drop_bar
        MB_ex1 = MatBalance.MatBalance()
        MB_ex1.c_oil_1bar = c_oil_1bar
        MB_ex1.c_wat_1bar = c_wat_1bar
        MB_ex1.c_res_1bar = c_res_1bar

        MB_ex1.t_reservoir_c = t_res_c
        MB_ex1.t_reservoir_init_c = t_res_init_c
        MB_ex1.p_reservoir_init_bar = p_res_init_bar
        MB_ex1.S_wat_connate_d = S_wat_connate_d
        MB_ex1.fluid.pbcal_bar = pbcal_bar

        MB_ex1.calc_depletion_above_pb(p_reservoir_bar)
        self.assertAlmostEqual(MB_ex1.oil_recovery_perc, 3.922566415892517,
                               delta=0.0001)

    def test_calc_depletion_above_and_below_pb(self):
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

        N_cum_oil_recovery_m3 = q_oil_surf_m3day * t_step_days * 45

        MB = MatBalance.MatBalance()

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
        self.assertAlmostEqual(MB.p_reservoir_bar, 51.77749,
                               delta=0.00001)