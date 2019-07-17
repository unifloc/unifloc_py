"""
Модуль для тестирования пакета uWell
"""
import unittest
import uniflocpy.uWell.uPipe as Pipe
import uniflocpy.uWell.deviation_survey as dev_surv
import sys
sys.path.append('../')

class TestWell(unittest.TestCase):
    def test_Pipe_calc_p_grad_pam(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        self.assertAlmostEqual(pipe.calc_p_grad_pam(p_bar, t_c), 10247.413809815911,
                               delta=0.0001)
    def test_Pipe_calc_t_grad_cm_in_tube(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        pipe.section_casing = False
        self.assertAlmostEqual(pipe.calc_t_grad_cm(p_bar, t_c), 0.020113527637494258,
                               delta=0.0001)

    def test_Pipe_calc_t_grad_cm_in_casing(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        pipe.section_casing = True
        self.assertAlmostEqual(pipe.calc_t_grad_cm(p_bar, t_c), 0.017798062343446185,
                               delta=0.0001)


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
        vert_angle_grad = swds.get_vert_angle_grad(h_mes_m)
        borehole_extension_m = swds.get_borehole_extension_m(h_mes_m)
        h_vert_m = swds.get_h_vert_m(h_mes_m)
        x_displacement_m = swds.get_x_displacement_m(h_mes_m)
        sum = curvature_rate_grad10m + vert_angle_grad + borehole_extension_m + h_vert_m + x_displacement_m
        self.assertAlmostEqual(sum, 3131.965243588372,
                               delta=0.0000001)