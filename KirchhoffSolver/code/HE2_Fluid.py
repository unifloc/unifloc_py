from uniflocpy.uPVT.BlackOil_model import Fluid
from HE2_ABC import HE2_ABC_Fluid

class HE2_DummyWater(HE2_ABC_Fluid):
    def __init__(self):
        self.rho_wat_kgm3 = 1000
        self.mu_wat_cp = 1

    def calc(self, P_bar, T_C):
        pass


if __name__ == '__main__':
    fl = HE2_DummyWater()
    fl.calc(100, 100)
    print(fl.rho_wat_kgm3)