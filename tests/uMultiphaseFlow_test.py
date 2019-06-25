"""
Молуль для тестрования пакета uMultiphaseFlow
"""
import unittest
import uniflocpy.uMultiphaseFlow.hydr_cor_Beggs_Brill as hydr_cor_Beggs_Brill
import uniflocpy.uMultiphaseFlow.friction_Bratland as friction_Bratland
import uniflocpy.uWell.uPipe as Pipe

class TestFriction_Bratland(unittest.TestCase):
    def test_first_part(self):
        number_re = 2000
        relative_roughness = 0.04
        d_m = 144 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction()
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.032,
                               delta=0.00000001)

    def test_second_part(self):
        number_re = 2500
        relative_roughness = 0.03
        d_m = 180 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction()
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.030869565217391304,
                               delta=0.00000001)

    def test_third_part(self):
        number_re = 15000
        relative_roughness = 0.002
        d_m = 159 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction()
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.037014744989689666,
                               delta=0.00000001)

    def test_fourth_part(self):
        number_re = 10**7
        relative_roughness = 0.02
        d_m = 159 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction()
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.04865207639535838,
                               delta=0.00000001)


class TestBB(unittest.TestCase):
    def test_Beggs_Brill_hyrd_cor(self):
        pipe = Pipe.Pipe(hydr_cor = hydr_cor_Beggs_Brill.Beggs_Brill_cor())

        p_bar = 11.713 * 10
        t_c = 82
        self.assertAlmostEqual(pipe.calc_p_grad_pam(p_bar, t_c), 10511.938363972778,
                               delta=0.00000001)


# лучше запустите все тесты в run_all_tests.py
# но, для тестирования только данного модуля воспользуйтесь следующими функциями
# calcTestSuite = unittest.TestSuite()
# calcTestSuite.addTest(unittest.makeSuite(TestFriction_Bratland))
# calcTestSuite.addTest(unittest.makeSuite(TestBB))
# runner = unittest.TextTestRunner(verbosity=2)
# runner.run(calcTestSuite)
