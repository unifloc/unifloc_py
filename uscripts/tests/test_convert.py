from unittest import TestCase
import uscripts


class TestConverters(TestCase):
    def test_convert_pressure(self):
        ar = 1
        self.assertAlmostEqual(uscripts.convert_pressure(ar, 'atm', 'psi'), 14.69594878, 5)
