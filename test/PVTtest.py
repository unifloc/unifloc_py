import PVT
import matplotlib.pyplot as plt
import numpy as np

def test2():
    g = PVT.GasGeneral()
    print(g.pseudo_pressure_mpa)

def test1():
    a = PVT.OilGeneral()
    a.calc(230,59)
    print(a.rs_m3m3)

    pp = np.arange(1,300,20)
    rs=[]
    for p in pp:
        a.calc(p,20)
        rs.append(a.rs_m3m3)
    plt.plot(pp,rs)
    plt.show()

test2()