import unittest
import uniflocpy.uWell.uPipe as Pipe

class TestWell(unittest.TestCase):
    def test_Pipe(self):
        p_bar = 100
        t_c = 80
        pipe = Pipe.Pipe()
        self.assertAlmostEqual(pipe.calc_p_grad_pam(p_bar, t_c), 10247.413809815911,
                               delta=0.0001)

