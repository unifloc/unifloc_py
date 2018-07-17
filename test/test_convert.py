from unittest import TestCase
import uconst


class TestConverters(TestCase):
    def test_convert_pressure(self):
        ar = 1
        self.assertAlmostEqual(uconst.convert_pressure(ar, 'atm', 'psi'), 14.69594878, 5)
