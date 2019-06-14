import unittest
import uniflocpy.uTools.uconst as uconst

class TestConverters(unittest.TestCase):  # TODO продолжать ли тестирование?
    def test_convert_pressure(self):
        ar = 1
        self.assertAlmostEqual(uconst.convert_pressure(ar, 'atm', 'psi'), 14.69594878, 5)


# лучше запустите все тесты в run_all_tests.py
# но, для тестирования только данного модуля воспользуйтесь следующими функциями
# calcTestSuite = unittest.TestSuite()
# calcTestSuite.addTest(unittest.makeSuite(TestConverters))
# runner = unittest.TextTestRunner(verbosity=2)
# runner.run(calcTestSuite)