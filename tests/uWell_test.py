"""
Модуль для тестирования пакета uWell
"""
import unittest
import uniflocpy.uWell.uPipe as Pipe

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
        self.assertAlmostEqual(pipe.calc_t_grad_cm(p_bar, t_c), 0.0276669369708872,
                               delta=0.0001)

    def test_Pipe_calc_t_grad_cm_in_casing(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        pipe.section_casing = True
        self.assertAlmostEqual(pipe.calc_t_grad_cm(p_bar, t_c), 0.02535709180351646,
                               delta=0.0001)


