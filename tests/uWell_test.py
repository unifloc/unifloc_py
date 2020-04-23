"""
Модуль для тестирования пакета uWell
"""
import unittest
import uniflocpy.uWell.uPipe as Pipe
import uniflocpy.uWell.deviation_survey as dev_surv
import sys
import uniflocpy.uWell.Self_flow_well as self_flow_well_module
import uniflocpy.uPVT.BlackOil_model as BlackOil_model
sys.path.append('../')


class TestWell(unittest.TestCase):
    def test_Pipe_calc_p_grad_pam(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        self.assertAlmostEqual(pipe.calc_p_grad_pam(p_bar, t_c), 7913.910837767923,
                               delta=0.0001)

    def test_Pipe_calc_t_grad_cm_in_tube(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        pipe.section_casing = False
        self.assertAlmostEqual(pipe.calc_t_grad_cm(p_bar, t_c), 0.020721113953251068,
                               delta=0.0001)

    def test_Pipe_calc_t_grad_cm_in_casing(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        pipe.section_casing = True
        self.assertAlmostEqual(pipe.calc_t_grad_cm(p_bar, t_c), 0.0184349739137646,
                               delta=0.0001)
# TODO температура последний рассчитанный параметр, но может быть поменять на сумму для более точной проверки?


class TestSelfFlowWell(unittest.TestCase):
    def test_calc_all_from_down_to_up(self):
        self_flow_well_object = self_flow_well_module.self_flow_well()
        self_flow_well_object.calc_all_from_down_to_up()
        t_wellhead_c = self_flow_well_object.t_wellhead_c
        p_wellhead_bar =  self_flow_well_object.p_wellhead_bar
        sum_p_t = t_wellhead_c + p_wellhead_bar
        self.assertAlmostEquals(sum_p_t, 187.76503060230502, delta=0.00000001)

    def test_calc_all_from_down_to_up_vba_fluid_simple_line_for_t(self):
        bo_option = BlackOil_model.BlackOil_option()
        bo_option.set_vba_preset()
        self_flow_well_object = self_flow_well_module.self_flow_well(fluid=1, temp_corr=1)
        self_flow_well_object.pipe.fluid_flow.fl.option = bo_option
        self_flow_well_object.calc_all_from_down_to_up()
        t_wellhead_c = self_flow_well_object.t_wellhead_c
        p_wellhead_bar =  self_flow_well_object.p_wellhead_bar
        sum_p_t = t_wellhead_c + p_wellhead_bar
        self.assertAlmostEquals(sum_p_t, 127.22040318153236, delta=0.00000001)

    def test_calc_all_from_up_to_down(self):
        self_flow_well_object = self_flow_well_module.self_flow_well(temp_corr=1)
        self_flow_well_object.calc_all_from_up_to_down()
        p_bottomhole_bar = self_flow_well_object.p_bottomhole_bar
        t_bottomhole_c = self_flow_well_object.t_bottomhole_c
        sum_p_t = p_bottomhole_bar + t_bottomhole_c
        self.assertAlmostEquals(sum_p_t, 179.91262201328874, delta=0.00000001)

    def test_calc_all_from_up_to_down_vba_fluid_simple_line_for_t(self):
        bo_option = BlackOil_model.BlackOil_option()
        bo_option.set_vba_preset()
        self_flow_well_object = self_flow_well_module.self_flow_well(fluid=1, temp_corr=1)
        self_flow_well_object.pipe.fluid_flow.fl.option = bo_option
        self_flow_well_object.calc_all_from_up_to_down()
        p_bottomhole_bar = self_flow_well_object.p_bottomhole_bar
        t_bottomhole_c = self_flow_well_object.t_bottomhole_c
        sum_p_t = p_bottomhole_bar + t_bottomhole_c
        self.assertAlmostEquals(sum_p_t, 180.69492387992076, delta=0.00000001)


class TestDeviationSurvey(unittest.TestCase):
    def test_well_deviation_survey(self):
        wds = dev_surv.well_deviation_survey()
        path_to_file =  './test_data/Инклинометрия 2196 нп.xls'
        wds.load_deviation_survey(path_to_file)
        wds.calc_all()
        h_mes_m = 2002.22
        vert_angle_grad = wds.get_vert_angle_grad(h_mes_m)
        h_vert_m = wds.get_h_vert_m(h_mes_m)
        curvature_rate_grad10m = wds.get_curvature_rate_grad10m(h_mes_m)
        sum = vert_angle_grad + h_vert_m + curvature_rate_grad10m
        self.assertAlmostEqual(sum, 1826.6854610931557,
                               delta=0.0000001)

    def test_simple_well_deviation_survey(self):
        swds = dev_surv.simple_well_deviation_survey()
        swds.calc_all()
        h_mes_m = 1587.566
        curvature_rate_grad10m = swds.get_curvature_rate_grad10m(h_mes_m)
        vert_angle_grad = swds.get_angle_to_horizontal_grad(h_mes_m)
        borehole_extension_m = swds.get_borehole_extension_m(h_mes_m)
        h_vert_m = swds.get_h_vert_m(h_mes_m)
        x_displacement_m = swds.get_x_displacement_m(h_mes_m)
        sum = curvature_rate_grad10m + vert_angle_grad + borehole_extension_m + h_vert_m + x_displacement_m
        self.assertAlmostEqual(sum, 2429.763334309198,
                               delta=0.00001)