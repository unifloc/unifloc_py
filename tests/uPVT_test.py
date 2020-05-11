"""
Модуль для тестирования пакета uPVT
"""
import unittest
import uniflocpy.uPVT.PVT_fluids as PVT_fluids
import uniflocpy.uPVT.PVT_correlations as PVT_correlations
import uniflocpy.uPVT.BlackOil_model as BlackOil_model

#TODO не хватает теста для одной функции - найти ее
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
        self.assertAlmostEqual(sum, 11690.655158519261,
                               delta=0.0001)

    def test_FluidMcCain(self):
        pressure_bar = 100
        temp_c = 80
        fluid = PVT_fluids.FluidMcCain()
        fluid.calc(pressure_bar, temp_c)
        sum = 0
        for i in fluid.__dict__.items():
            sum += i[-1]
        self.assertAlmostEqual(sum, 2632.7063404381806,
                               delta=0.0001)  # TODO cлишком большая разница со Стендингом, проверить (плотность и др.)

    def test_FluidFlow(self):
        pressure_bar = 100
        temp_c = 80
        fluid_flow = PVT_fluids.FluidFlow()
        fluid_flow.calc(pressure_bar, temp_c)
        sum = 0
        for i in fluid_flow.__dict__.items():
            if type(i[-1]) != type(PVT_fluids.FluidStanding()):
                sum += i[-1]
        self.assertAlmostEqual(sum,  53422.725353030706,
                               delta=0.0001)

    def test_BlackOil_model(self):
        pressure_bar = 100
        temp_c = 80
        fluid_model = BlackOil_model.Fluid()
        fluid_model.calc(pressure_bar, temp_c)
        sum = 0
        for i in fluid_model.__dict__.items():
            if type(i[-1]) != type((BlackOil_model.BlackOil_option())):
                sum += i[-1]
        self.assertAlmostEqual(sum, 13330.58601414152,
                               delta=0.0001)

    def test_BlackOil_model_vba_preset(self):
        pressure_bar = 20
        temp_c = 80
        blackoil_option_vba = BlackOil_model.BlackOil_option()
        blackoil_option_vba.set_vba_preset()
        fluid_model = BlackOil_model.Fluid(option=blackoil_option_vba)
        fluid_model.calc(pressure_bar, temp_c)
        sum = 0
        for i in fluid_model.__dict__.items():
            if type(i[-1]) != type((BlackOil_model.BlackOil_option())):
                sum += i[-1]
        self.assertAlmostEqual(sum, 12781.243876576445,
                               delta=0.0001)

    def test_BlackOil_option(self):
        option = BlackOil_model.BlackOil_option()
        sum = 0
        for i in option.__dict__.items():
            sum += i[-1]
        self.assertAlmostEqual(sum, 0,
                               delta=0.0001)


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
        self.assertAlmostEqual(PVT_correlations.unf_pb_Valko_MPaa(rsb_m3m3, gamma_oil, gamma_gas, t_K), 23.29991481380937,
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

    def test_unf_compressibility_saturated_oil_VB_1Mpa(self):
        rsb_m3m3 = 200
        t_k = 350
        gamma_oil = 0.86
        p_MPaa = 10
        pb_mpa = 15
        self.assertAlmostEqual(PVT_correlations.unf_compressibility_saturated_oil_McCain_1Mpa(p_MPaa, pb_mpa, t_k, gamma_oil, rsb_m3m3),
                               0.004934802450463976, delta=0.0001)

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

    def test_unf_zfactor_DAK_ppr(self):
        ppr = 4
        tpr = 2

        self.assertAlmostEqual(PVT_correlations.unf_zfactor_DAK_ppr(ppr, tpr), 0.9426402059431057,
                               delta=0.0001)

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

    def test_unf_density_brine_uniflocvba_kgm3(self):
        gamma_w = 1
        bw_m3m3 = 1.1
        self.assertAlmostEqual(PVT_correlations.unf_density_brine_uniflocvba_kgm3(gamma_w, bw_m3m3), 909.090909090909, delta=0.0001)


    def test_unf_fvf_brine_McCain_m3m3(self):
        t_K = 300
        p_MPaa =20
        self.assertAlmostEqual(PVT_correlations.unf_fvf_brine_McCain_m3m3(t_K, p_MPaa), 1.0007434853666817,
                               delta=0.0001)

    def test_unf_fvf_brine_Spivey_m3m3(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        self.assertAlmostEqual(PVT_correlations.unf_fvf_brine_Spivey_m3m3(t_K, p_MPaa, s_ppm), 1.0279011434122953, delta=0.0001)

    def test_unf_viscosity_brine_McCain_cp(self):
        t_K = 350
        p_MPaa = 20
        s_ppm = 10000
        self.assertAlmostEqual(PVT_correlations.unf_viscosity_brine_McCain_cp(t_K, p_MPaa, s_ppm), 0.4165673950441691, delta=0.0001)

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

    def test_unf_surface_tension_go_Abdul_Majeed_Nm(self):
        t_K = 350
        rs_m3m3 = 50
        gamma_oil = 0.6
        z = 1
        self.assertAlmostEqual(PVT_correlations.unf_surface_tension_go_Abdul_Majeed_Nm(t_K, gamma_oil, rs_m3m3),
                               0.003673109943227455, delta=0.0001)

    def test_unf_surface_tension_go_Baker_Swerdloff_Nm(self):
        t_K = 350
        p_MPaa = 0.1
        gamma_oil = 0.6
        z = 1
        self.assertAlmostEqual(PVT_correlations.unf_surface_tension_go_Baker_Swerdloff_Nm(t_K, gamma_oil, p_MPaa),
                               0.01054309596387229, delta=0.0001)

    def test_unf_heat_capacity_oil_Gambill_JkgC(self):
        gamma_oil = 0.8
        t_c = 60
        self.assertAlmostEqual(PVT_correlations.unf_heat_capacity_oil_Gambill_JkgC(gamma_oil, t_c),
                               2110.7207148850844,
                               delta=0.0001)

    def test_unf_heat_capacity_oil_Wes_Wright_JkgC(self):
        gamma_oil = 0.8
        t_c = 60
        self.assertAlmostEqual(PVT_correlations.unf_heat_capacity_oil_Wes_Wright_JkgC(gamma_oil, t_c),
                               2162.0, delta=0.0001)

    def test_unf_thermal_conductivity_oil_Abdul_Seoud_Moharam_WmK(self):
        gamma_oil = 0.8
        t_c = 60
        self.assertAlmostEqual(PVT_correlations.unf_thermal_conductivity_oil_Abdul_Seoud_Moharam_WmK(gamma_oil, t_c),
                               0.10999860144810972, delta=0.0001)

    def test_unf_thermal_conductivity_oil_Smith_WmK(self):
        gamma_oil = 0.8
        t_c = 60
        self.assertAlmostEqual(PVT_correlations.unf_thermal_conductivity_oil_Smith_WmK(gamma_oil, t_c),
                               0.16568762875, delta=0.0001)

    def test_unf_thermal_conductivity_oil_Cragoe_WmK(self):
        gamma_oil = 0.8
        t_c = 60
        self.assertAlmostEqual(PVT_correlations.unf_thermal_conductivity_oil_Cragoe_WmK(gamma_oil, t_c),
                               0.1427090525,
                               delta=0.0001)

    def test_unf_heat_capacity_gas_Mahmood_Moshfeghian_JkgC(self):
        p_MPaa = 3
        gamma_gas = 0.8
        t_K = 300
        self.assertAlmostEqual(PVT_correlations.unf_heat_capacity_gas_Mahmood_Moshfeghian_JkgC(p_MPaa, t_K, gamma_gas),
                               2471.4603282835255,
                               delta=0.0001)

    def test_unf_thermal_conductivity_gas_methane_WmK(self):
        t_c = 20
        self.assertAlmostEqual(PVT_correlations.unf_thermal_conductivity_gas_methane_WmK(t_c),
                               0.033390322580645164,
                               delta=0.0001)

    def test_unf_heat_capacity_water_IAPWS_JkgC(self):
        t_c = 20
        self.assertAlmostEqual(PVT_correlations.unf_heat_capacity_water_IAPWS_JkgC(t_c),
                               4184.92592,
                               delta=0.0001)

    def test_unf_thermal_conductivity_water_IAPWS_WmC(self):
        t_c = 20
        self.assertAlmostEqual(PVT_correlations.unf_thermal_conductivity_water_IAPWS_WmC(t_c),
                               0.5992595999999999,
                               delta=0.0001)

    def test_unf_thermal_expansion_coefficient_water_IAPWS_1C(self):
        t_c = 20
        self.assertAlmostEqual(PVT_correlations.unf_thermal_expansion_coefficient_water_IAPWS_1C(t_c),
                               0.00022587,
                               delta=0.0001)

    def test_unf_surface_tension_gw_Sutton_Nm(self):
        rho_water_kgm3 = 1000
        rho_gas_kgm3 = 50
        t_c = 60
        self.assertAlmostEqual(PVT_correlations.unf_surface_tension_gw_Sutton_Nm(rho_water_kgm3, rho_gas_kgm3, t_c),
                               0.06256845320633196,
                               delta=0.0001)

    def test_unf_z_factor_Kareem(self):
        Tpr = 1.2
        Ppr = 1.2
        self.assertAlmostEqual(PVT_correlations.unf_z_factor_Kareem(Tpr, Ppr),
                               0.71245963496651,
                               delta=0.0001)

    def test_unf_pseudocritical_temperature_Standing_K(self):
        gamma_gas = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_pseudocritical_temperature_Standing_K(gamma_gas),
                               198.8016, #TODO сheck - маловато
                               delta=0.0001)

    def test_unf_pseudocritical_pressure_Standing_MPa(self):
        gamma_gas = 0.6
        self.assertAlmostEqual(PVT_correlations.unf_pseudocritical_pressure_Standing_MPa(gamma_gas),
                               4.567119999999999,
                               delta=0.0001)

    def test_unf_gas_density_VBA_kgm3(self):
        gamma_gas = 0.6
        b_gas_m3m3 = 0.005
        self.assertAlmostEqual(PVT_correlations.unf_gas_density_VBA_kgm3(gamma_gas, b_gas_m3m3),
                               147.0,
                               delta=0.0001)

    def test_unf_fvf_gas_vba_m3m3(self):
        T_K = 300
        z = 1.1
        P_MPa = 0.3
        self.assertAlmostEqual(PVT_correlations.unf_fvf_gas_vba_m3m3(T_K, z, P_MPa),
                               0.38194200000000006,
                               delta=0.0001)

    def test_unf_deadoilviscosity_BeggsRobinson_VBA_cP(self):
        gamma_oil = 0.8
        t_K = 300
        self.assertAlmostEqual(PVT_correlations.unf_deadoilviscosity_BeggsRobinson_VBA_cP(gamma_oil, t_K),
                               5.264455765058494,
                               delta=0.0001)

    def test_unf_surface_tension_Baker_Sverdloff_vba_nm(self):
        p_atma = 10
        t_C = 20
        gamma_o_ = 40
        self.assertAlmostEqual(sum(PVT_correlations.unf_surface_tension_Baker_Sverdloff_vba_nm(p_atma, t_C, gamma_o_)),
                               0.12299551951661537,
                               delta=0.0001)




# лучше запустите все тесты в run_all_tests.py
# но, для тестирования только данного модуля воспользуйтесь следующими функциями
# calcTestSuite = unittest.TestSuite()
# calcTestSuite.addTest(unittest.makeSuite(TestFluid))
# calcTestSuite.addTest(unittest.makeSuite(TestPVT))
# runner = unittest.TextTestRunner(verbosity=2)
# runner.run(calcTestSuite)