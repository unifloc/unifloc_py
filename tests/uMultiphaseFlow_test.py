import unittest
import uniflocpy.uMultiphaseFlow.hydr_cor_Beggs_Brill as hydr_cor_Beggs_Brill
import uniflocpy.uMultiphaseFlow.friction_Bratland as friction_Bratland


class TestFriction_Bratland(unittest.TestCase):
    def test_first_part(self):
        number_re = 2000
        relative_roughness = 0.04
        d_m = 144 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction(number_re, epsilon, d_m)
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.032,
                               delta=0.00000001)

    def test_second_part(self):
        number_re = 2500
        relative_roughness = 0.03
        d_m = 180 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction(number_re, epsilon, d_m)
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.030869565217391304,
                               delta=0.00000001)

    def test_third_part(self):
        number_re = 15000
        relative_roughness = 0.002
        d_m = 159 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction(number_re, epsilon, d_m)
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.037014744989689666,
                               delta=0.00000001)

    def test_fourth_part(self):
        number_re = 10**7
        relative_roughness = 0.02
        d_m = 159 / 1000
        epsilon = relative_roughness * d_m
        friction = friction_Bratland.Friction(number_re, epsilon, d_m)
        self.assertAlmostEqual(friction.calc_f(number_re, epsilon, d_m), 0.04865207639535838,
                               delta=0.00000001)


class TestBB(unittest.TestCase):
    def test_Beggs_Brill_calc_grad(self):
        hydr_cor = hydr_cor_Beggs_Brill.Beggs_Brill_cor()
        pressure_mpa = 11.713
        temperature_c = 82
        PT_test = hydr_cor_Beggs_Brill.PT(pressure_mpa, temperature_c)
        self.assertAlmostEqual(hydr_cor.calc_grad(PT_test), 4602.417176719269,
                               delta=0.00000001)


# лучше запустите все тесты в run_all_tests.py
# но, для тестирования только данного модуля воспользуйтесь следующими функциями
# calcTestSuite = unittest.TestSuite()
# calcTestSuite.addTest(unittest.makeSuite(TestFriction_Bratland))
# calcTestSuite.addTest(unittest.makeSuite(TestBB))
# runner = unittest.TextTestRunner(verbosity=2)
# runner.run(calcTestSuite)
