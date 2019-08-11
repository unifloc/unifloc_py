"""
Модуль для тестирования пакета uMotor
"""
import unittest
import uniflocpy.uMotor.equivalent_circuit as equivalent_circuit


class TestMotor(unittest.TestCase):
    def test_calc_resistances(self):
        # Параметры АД для теста(ПЭДН32-117-1000 Новомет)
        nom_power__kW = 32
        nom_voltage__V = 1000
        nom_efficient = 0.835
        nom_slip = 0.05
        # Оптимизация коэффициентов
        motor_lamda = 3
        alfa_0 = 0.61
        moments_division = 1.13
        self.assertAlmostEqual(sum(equivalent_circuit.calc_resistances(nom_power__W=nom_power__kW*1e3,
                                 nom_voltage__V=nom_voltage__V,
                                 nom_slip=nom_slip,
                                 nom_efficient=nom_efficient,
                                 motor_lamda=motor_lamda,
                                 alfa_0=alfa_0,
                                 moments_division=moments_division)), 17.10194928209109,
                               delta=0.0001)

    def test_calc_idle(self):
        # Параметры АД для теста(ПЭДН32-117-1000 Новомет)
        nom_voltage__V = 1000
        nom_efficient = 0.835
        nom_cos = 0.84
        nom_slip = 0.05
        r_1__Om = 3.594324220578044
        r_2__Om = 3.500644734028088
        x_k__Om = 10.006980327484957
        self.assertAlmostEqual(sum(equivalent_circuit.calc_idle(nom_voltage__V, nom_slip, nom_efficient, nom_cos, r_1__Om,
              r_2__Om, x_k__Om)), 8.59153995887971,
                               delta=0.0001)
    def test_calc_g_circuit(self):
        # Параметры АД для теста(ПЭДН32-117-1000 Новомет)
        nom_power__kW = 32
        nom_voltage__V = 1000
        nom_efficient = 0.835
        nom_cos = 0.84
        nom_slip = 0.05
        r_1__Om = 3.594324220578044
        r_2__Om = 3.500644734028088
        x_k__Om = 10.006980327484957
        work_voltage__V = 1000
        frequency__Hz = 50
        moments_division = 1.13
        self.assertAlmostEqual(sum(equivalent_circuit.calc_g_circuit(slip=nom_slip,
                                                        nom_power__W=nom_power__kW,
                                                        nom_voltage__V=nom_voltage__V,
                                                        nom_slip=nom_slip,
                                                        nom_efficient=nom_efficient,
                                                        nom_cos=nom_cos,
                                                        frequency__Hz=frequency__Hz,
                                                        voltage__V=work_voltage__V,
                                                        r_1__Om=r_1__Om,
                                                        r_2__Om=r_2__Om,
                                                        x_k__Om=x_k__Om,
                                                        moments_division=moments_division)), 177.3327837108102,
                               delta=0.0001)
    def test_motor_data_loading(self):
        # Параметры АД для теста(ПЭДН32-117-1000 Новомет)
        nom_power__kW = 32
        motor_power__kW = 30
        nom_voltage__V = 1000
        nom_efficient = 0.835
        nom_cos = 0.84
        nom_slip = 0.05
        r_1__Om = 3.594324220578044
        r_2__Om = 3.500644734028088
        x_k__Om = 10.006980327484957
        work_voltage__V = 1000
        frequency__Hz = 50
        moments_division = 1.13
        self.assertAlmostEqual(sum(equivalent_circuit.motor_data_loading(motor_power__kW*1e3,
                                                        nom_power__W=nom_power__kW*1e3,
                                                        nom_voltage__V=nom_voltage__V,
                                                        nom_slip=nom_slip,
                                                        nom_efficient=nom_efficient,
                                                        nom_cos=nom_cos,
                                                        frequency__Hz=frequency__Hz,
                                                        voltage__V=work_voltage__V,
                                                        r_1__Om=r_1__Om,
                                                        r_2__Om=r_2__Om,
                                                        x_k__Om=x_k__Om,
                                                        moments_division=moments_division)), 142.20960712,
                               delta=0.0001)


# лучше запустите все тесты в run_all_tests.py
# но, для тестирования только данного модуля воспользуйтесь следующими функциями
# calcTestSuite = unittest.TestSuite()
# calcTestSuite.addTest(unittest.makeSuite(TestMotor))
# runner = unittest.TextTestRunner(verbosity=2)
# runner.run(calcTestSuite)