import unittest
import uniflocpy.uTemperature.temp_cor_Hasan_Kabir_integrated as temp_cor_Hasan_Kabir_integrated
import uniflocpy.uTemperature.temp_cable_NS as temp_cable_NS


class TestTempCable(unittest.TestCase):
    def test_calc_i_max_a(self):
        cable = temp_cable_NS.Cable()
        cable.cabel_type = 'Round'
        t_max_c = 120
        t_env_c = 90
        self.assertAlmostEqual(cable.calc_i_max_a(t_max_c, t_env_c), 65.0923992910633,
                               delta=0.0001)

    def test_calc_t_max_cable_c(self):
        cable = temp_cable_NS.Cable()
        cable.cabel_type = 'Flat'
        t_f_c = 120
        i_a = 30
        self.assertAlmostEqual(cable.calc_t_max_cable_c(t_f_c, i_a), 126.81955779,
                               delta=0.0001)


class TestTempCor(unittest.TestCase):
    def test_calc_t_c_fluid(self):
        distance_from_bh = 10
        pressure_pa = 20*10**6
        temp_cor = temp_cor_Hasan_Kabir_integrated.Hasan_Kabir_cor()
        self.assertAlmostEqual(temp_cor.calc_t_c_fluid(distance_from_bh, pressure_pa), 93.23420470524496,
                               delta=0.00000001)


# лучше запустите все тесты в run_all_tests.py
# но, для тестирования только данного модуля воспользуйтесь следующими функциями
# calcTestSuite = unittest.TestSuite()
# calcTestSuite.addTest(unittest.makeSuite(TestTempCor))
# calcTestSuite.addTest(unittest.makeSuite(TestTempCable))
# runner = unittest.TextTestRunner(verbosity=2)
# runner.run(calcTestSuite)
