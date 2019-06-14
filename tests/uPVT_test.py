import unittest
import uniflocpy.uPVT.PVT_fluids as PVT_fluids
import uniflocpy.uPVT.PVT_correlations as PVT_correlations


# Проверка флюидов через условную "хеш" сумму параметров после calc(P,T)
class TestFluid(unittest.TestCase):
    def test_FluidStanding(self):
        pressure_bar = 100
        temp_c = 80
        fluid = PVT_fluids.FluidStanding()
        fluid.calc(pressure_bar, temp_c)
        sum = 0
        for i in fluid.__dict__.items():
            sum += i[-1]
        self.assertAlmostEqual(sum, 2118.8391250450486,
                               delta=0.0001)

    def test_FluidMcCain(self):
        pressure_bar = 100
        temp_c = 80
        fluid = PVT_fluids.FluidMcCain()
        fluid.calc(pressure_bar, temp_c)
        sum = 0
        for i in fluid.__dict__.items():
            sum += i[-1]
        self.assertAlmostEqual(sum, 2624.679427463553,
                               delta=0.0001)  # TODO cлишком большая разница со Стендингом, проверить (плотность и др.)


class TestPVT(unittest.TestCase):
    def test_unf_pb_Standing_MPaa(self):
        rsb_m3m3 = 100
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(PVT_correlations.unf_pb_Standing_MPaa(rsb_m3m3, gamma_oil, gamma_gas, t_K), 20.170210695316566,
                               delta=0.0001)

    def test_unf_pb_Valko_MPaa(self):
        rsb_m3m3 = 100
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(PVT_correlations.unf_pb_Valko_MPaa(rsb_m3m3, gamma_oil, gamma_gas, t_K), 22.694051278736964,
                               delta=0.0001)

    def test_unf_rs_Standing_m3m3(self):
        p_MPaa = 10
        pb_MPaa = 15
        rsb = 200
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(PVT_correlations.unf_rs_Standing_m3m3(p_MPaa, pb_MPaa, rsb, gamma_oil, gamma_gas, t_K),
                               122.74847910146916, delta=0.0001)

    def test_unf_rs_Velarde_m3m3(self):
        p_MPaa = 10
        pb_MPaa = 15
        rsb = 250
        gamma_oil = 0.86
        gamma_gas = 0.6
        t_K = 350
        self.assertAlmostEqual(PVT_correlations.unf_rs_Velarde_m3m3(p_MPaa, pb_MPaa, rsb, gamma_oil, gamma_gas, t_K),
                               170.25302712587356, delta=0.0001)

    def test_unf_rsb_Mccain_m3m3(self):
        rsp_m3m3 = 150
        gamma_oil = 0.86
        psp_MPaa = 5
        tsp_K = 320
        self.assertAlmostEqual(PVT_correlations.unf_rsb_Mccain_m3m3(rsp_m3m3, gamma_oil, psp_MPaa, tsp_K),
                               161.03286985548442, delta=0.0001)

    def test_unf_gamma_gas_Mccain(self):
        rsp_m3m3 = 30
        rst_m3m3 = 20
        gamma_gassp = 0.65
        gamma_oil = 0.86
        psp_MPaa = 5
        tsp_K = 350
        self.assertAlmostEqual(PVT_correlations.unf_gamma_gas_Mccain(rsp_m3m3, rst_m3m3, gamma_gassp, gamma_oil, psp_MPaa, tsp_K),
                               0.7932830162938984, delta=0.0001)

    def test_unf_fvf_Mccain_m3m3_below(self):
        density_oilsto_kgm3 = 800
        rs_m3m3 = 200
        density_oil_kgm3 = 820
        gamma_gas = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_fvf_Mccain_m3m3_below(density_oilsto_kgm3, rs_m3m3, density_oil_kgm3, gamma_gas),
                               1.1542114715227887, delta=0.0001)

    def test_unf_fvf_VB_m3m3_above(self):
        bob = 1.3
        cofb_1MPa = 3 * 10**(-3)
        pb_MPaa = 12
        p_MPaa = 15
        self.assertAlmostEqual(PVT_correlations.unf_fvf_VB_m3m3_above(bob, cofb_1MPa, pb_MPaa, p_MPaa), 1.2883524924047487, delta=0.0001)

    def test_unf_compressibility_oil_VB_1Mpa(self):
        rs_m3m3 = 200
        t_K = 350
        gamma_oil = 0.86
        p_MPaa = 15
        gamma_gas = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_compressibility_oil_VB_1Mpa(rs_m3m3, t_K, gamma_oil, p_MPaa, gamma_gas),
                               0.004546552811369566, delta=0.0001)

    def test_unf_fvf_Standing_m3m3_saturated(self):
        rs_m3m3 = 200
        gamma_gas = 0.6
        gamma_oil = 0.86
        t_K = 350
        self.assertAlmostEqual(PVT_correlations.unf_fvf_Standing_m3m3_saturated(rs_m3m3, gamma_gas, gamma_oil, t_K),
                               1.5527836202040448, delta=0.0001)

    def test_unf_density_oil_Mccain(self):
        p_MPaa = 10
        pb_MPaa = 12
        co_1MPa = 3 * 10**(-3)
        rs_m3m3 = 250
        gamma_gas = 0.6
        t_K = 350
        gamma_oil = 0.86
        gamma_gassp = 0
        self.assertAlmostEqual(PVT_correlations.unf_density_oil_Mccain(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, gamma_gas, t_K, gamma_oil,
                               gamma_gassp), 630.0536681794456, delta=0.0001)

    def test_unf_density_oil_Standing(self):
        p_MPaa = 10
        pb_MPaa = 12
        co_1MPa = 3 * 10**(-3)
        rs_m3m3 = 250
        bo_m3m3 = 1.1
        gamma_gas = 0.6
        gamma_oil = 0.86
        self.assertAlmostEqual(PVT_correlations.unf_density_oil_Standing(p_MPaa, pb_MPaa, co_1MPa, rs_m3m3, bo_m3m3, gamma_gas, gamma_oil
                                                        ), 948.7272727272725, delta=0.0001)

    def test_unf_deadoilviscosity_Beggs_cP(self):
        gamma_oil = 0.86
        t_K = 350
        self.assertAlmostEqual(PVT_correlations.unf_deadoilviscosity_Beggs_cP(gamma_oil, t_K), 2.86938394460968, delta=0.0001)

    def test_unf_saturatedoilviscosity_Beggs_cP(self):
        deadoilviscosity_cP = 2.87
        rs_m3m3 = 150
        self.assertAlmostEqual(PVT_correlations.unf_saturatedoilviscosity_Beggs_cP(deadoilviscosity_cP, rs_m3m3), 0.5497153091178292,
                               delta=0.0001)

    def test_unf_undersaturatedoilviscosity_VB_cP(self):
        p_MPaa = 10
        pb_MPaa = 12
        bubblepointviscosity_cP = 1
        self.assertAlmostEqual(PVT_correlations.unf_undersaturatedoilviscosity_VB_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP),
                               0.9767303348551418, delta=0.0001)

    def test_unf_undersaturatedoilviscosity_Petrovsky_cP(self):
        p_MPaa = 10
        pb_MPaa = 12
        bubblepointviscosity_cP = 1
        self.assertAlmostEqual(PVT_correlations.unf_undersaturatedoilviscosity_Petrovsky_cP(p_MPaa, pb_MPaa, bubblepointviscosity_cP),
                               0.9622774530985722, delta=0.0001)

    def test_unf_oil_viscosity_Beggs_VB_cP(self):
        deadoilviscosity_cP = 2.87
        rs_m3m3 = 150
        p_MPaa = 10
        pb_MPaa = 12
        self.assertAlmostEqual(PVT_correlations.unf_oil_viscosity_Beggs_VB_cP(deadoilviscosity_cP, rs_m3m3, p_MPaa, pb_MPaa),
                               0.5497153091178292, delta=0.0001)

    def test_unf_pseudocritical_temperature_K(self):
        gamma_gas = 0.6
        y_h2s = 0.01
        y_co2 = 0.03
        y_n2 = 0.02
        self.assertAlmostEqual(PVT_correlations.unf_pseudocritical_temperature_K(gamma_gas, y_h2s, y_co2, y_n2), 198.0708725589674,
                               delta=0.0001)

    def test_unf_pseudocritical_pressure_MPa(self):
        gamma_gas = 0.6
        y_h2s = 0.01
        y_co2 = 0.03
        y_n2 = 0.02
        self.assertAlmostEqual(PVT_correlations.unf_pseudocritical_pressure_MPa(gamma_gas, y_h2s, y_co2, y_n2), 5.09893164741181,
                               delta=0.0001)

    def test_unf_zfactor_DAK(self):
        p_MPaa = 10
        t_K = 350
        ppc_MPa = 7.477307083789863
        tpc_K = 239.186917147216
        self.assertAlmostEqual(PVT_correlations.unf_zfactor_DAK(p_MPaa, t_K, ppc_MPa, tpc_K), 0.8607752185760458, delta=0.0001)

    def test_unf_gasviscosity_Lee_cP(self):
        t_K = 350
        p_MPaa = 10
        z = 0.84
        gamma_gas = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_gasviscosity_Lee_cP(t_K, p_MPaa, z, gamma_gas), 0.015423237238038448, delta=0.0001)

    def test_unf_gas_fvf_m3m3(self):
        t_K = 350
        p_MPaa = 10
        z = 0.84
        self.assertAlmostEqual(PVT_correlations.unf_gas_fvf_m3m3(t_K, p_MPaa, z), 0.010162381033600544, delta=0.0001)

    def test_unf_density_brine_Spivey_kgm3(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        par = 1
        self.assertAlmostEqual(PVT_correlations.unf_density_brine_Spivey_kgm3(t_K, p_MPaa, s_ppm, par), 987.685677686006, delta=0.0001)

    def test_unf_compressibility_brine_Spivey_1MPa(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        z = 1
        par = 0
        self.assertAlmostEqual(PVT_correlations.unf_compressibility_brine_Spivey_1MPa(t_K, p_MPaa, s_ppm, z, par), 0.0004241522548512511,
                               delta=0.0001)

    def test_unf_fvf_brine_Spivey_m3m3(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        self.assertAlmostEqual(PVT_correlations.unf_fvf_brine_Spivey_m3m3(t_K, p_MPaa, s_ppm), 1.0279011434122953, delta=0.0001)

    def test_unf_viscosity_brine_MaoDuan_cP(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        self.assertAlmostEqual(PVT_correlations.unf_viscosity_brine_MaoDuan_cP(t_K, p_MPaa, s_ppm), 0.3745199364964906, delta=0.0001)

    def test_unf_pb_Glaso_MPaa(self):
        rs_m3m3 = 100
        t_K = 350
        gamma_oil = 0.86
        gamma_gas = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_pb_Glaso_MPaa(rs_m3m3, t_K, gamma_oil, gamma_gas), 23.365669948236604, delta=0.0001)

    def test_unf_fvf_Glaso_m3m3_saturated(self):
        rs_m3m3 = 100
        t_K = 350
        gamma_oil = 0.86
        gamma_gas = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_fvf_Glaso_m3m3_saturated(rs_m3m3, t_K, gamma_oil, gamma_gas), 1.2514004319480372,
                               delta=0.0001)

    def test_unf_fvf_Glaso_m3m3_below(self):
        rs_m3m3 = 100
        t_K = 350
        gamma_oil = 0.86
        gamma_gas = 0.6
        p_MPaa = 10
        self.assertAlmostEqual(PVT_correlations.unf_fvf_Glaso_m3m3_below(rs_m3m3, t_K, gamma_oil, gamma_gas, p_MPaa), 1.7091714311161692,
                               delta=0.0001)

    def test_unf_compressibility_gas_Mattar_1MPa(self):
        p_MPaa = 10
        t_K = 350
        ppc_MPa = 7.477307083789863
        tpc_K = 239.186917147216
        self.assertAlmostEqual(PVT_correlations.unf_compressibility_gas_Mattar_1MPa(p_MPaa, t_K, ppc_MPa, tpc_K), 0.4814932416304309,
                               delta=0.0001)


    def test_unf_McCain_specificgravity(self):
        p_MPaa = 10
        rsb_m3m3 = 100
        t_K = 350
        gamma_oil = 0.8
        gamma_gassp = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_McCain_specificgravity(p_MPaa, rsb_m3m3, t_K, gamma_oil, gamma_gassp), 0.6004849666507259,
                               delta=0.0001)

    def test_unf_gwr_brine_Spivey_m3m3(self):
        s_ppm = 10000
        z = 1
        self.assertAlmostEqual(PVT_correlations.unf_gwr_brine_Spivey_m3m3(s_ppm, z), 0.0013095456419714546, delta=0.0001)

    def test_unf_zfactor_BrillBeggs(self):
        ppr = 2
        tpr = 2
        self.assertAlmostEqual(PVT_correlations.unf_zfactor_BrillBeggs(ppr, tpr), 0.9540692750239955, delta=0.0001)

    def test_unf_gas_density_kgm3(self):
        t_K = 350
        p_MPaa = 0.1
        gamma_gas = 0.6
        z = 1
        self.assertAlmostEqual(PVT_correlations.unf_gas_density_kgm3(t_K, p_MPaa, gamma_gas, z), 0.5982465188241361, delta=0.0001)

# лучше запустите все тесты в run_all_tests.py
# но, для тестирования только данного модуля воспользуйтесь следующими функциями
# calcTestSuite = unittest.TestSuite()
# calcTestSuite.addTest(unittest.makeSuite(TestFluid))
# calcTestSuite.addTest(unittest.makeSuite(TestPVT))
# runner = unittest.TextTestRunner(verbosity=2)
# runner.run(calcTestSuite)