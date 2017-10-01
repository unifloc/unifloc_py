import PVT
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
mpl.rcParams['font.family'] = 'fantasy'
mpl.rcParams['font.fantasy'] = 'Times New Roman'

def test4():
    # TODO тут надо сделать тест для расчета z фактора газа, чтобы в итоге выводился график Стендинга

    pass

def test3():
    """
    test from ruslan

    :return:
    """
    """Временно сюда построитель графиков"""


    def pvt_plot(oil, p_0=1, p_n=300, dp=20, t_c=20):
        # TODO надо поправить горизонтальный и вертикальный интервал в выводе графиков, чтобы не было наползаний
        """Газосодержание"""
        plt.subplot(321)
        rs = []
        pp = np.arange(p_0, p_n, dp)
        for p in pp:
            oil.calc(p, t_c)
            rs.append(oil.rs_m3m3)
        plt.ylim(0, np.max(rs) + 10)
        plt.xlim(0, p_n)
        plt.grid(True)
        plt.title('Газосодержание', color = 'black', family = 'fantasy')
        plt.ylabel('Rs, м3/м3', color = 'black', family = 'fantasy')
        plt.xlabel('Давление, атм', color = 'black', family = 'fantasy')
        plt.plot(pp, rs, 'b', linewidth=3)

        """Плотность"""
        plt.subplot(322)
        rho = []
        pp = np.arange(p_0, p_n, dp)
        for p in pp:
            oil.calc(p, t_c)
            rho.append(oil.rho_kgm3)
        plt.ylim(0, np.max(rho) + 10)
        plt.xlim(0, p_n)
        plt.grid(True)
        plt.title('Плотность', color = 'black', family = 'fantasy')
        plt.ylabel('r, кг/м3', color = 'black', family = 'fantasy')
        plt.xlabel('Давление, атм', color = 'black', family = 'fantasy')
        plt.plot(pp, rho, 'g', linewidth=3)

        """Объемный коэффициент нефти"""
        plt.subplot(323)
        Bo = []
        pp = np.arange(p_0, p_n, dp)
        for p in pp:
            oil.calc(p, t_c)
            Bo.append(oil._bo_m3m3)
        plt.ylim(0, np.max(Bo) + 0.3)
        plt.xlim(0, p_n)
        plt.grid(True)
        plt.title('Объемный коэффициент нефти', color = 'black', family = 'fantasy')
        plt.ylabel('Bo, м3/м3', color = 'black', family = 'fantasy')
        plt.xlabel('Давление, атм', color = 'black', family = 'fantasy')
        plt.plot(pp, Bo, 'r', linewidth=3)

        """Вязкость нефти"""
        plt.subplot(324)
        mu = []
        pp = np.arange(p_0, p_n, dp)
        for p in pp:
            oil.calc(p, t_c)
            mu.append(oil._mu_cp)
        plt.ylim(0, np.max(mu) + 0.3)
        plt.xlim(0, p_n)
        plt.grid(True)
        plt.title('Вязкость нефти', color = 'black', family = 'fantasy')
        plt.ylabel('m, сПз', color = 'black', family = 'fantasy')
        plt.xlabel('Давление, атм', color = 'black', family = 'fantasy')
        plt.plot(pp, mu, 'b', linewidth=3)

        """Сжимаемость нефти"""
        plt.subplot(325)
        co = []
        pp = np.arange(p_0, p_n, dp)
        for p in pp:
            oil.calc(p, t_c)
            co.append(oil._co_1atm)
        plt.ylim(0, np.max(co))
        plt.xlim(0, p_n)
        plt.grid(True)
        plt.title('Сжимаемость нефти', color = 'black', family = 'fantasy')
        plt.ylabel('Со, 1/атм', color = 'black', family = 'fantasy')
        plt.xlabel('Давление, атм', color = 'black', family = 'fantasy')
        plt.plot(pp, co, 'm', linewidth=3)
        plt.show()

        #input("\nНажмите Enter, чтобы продолжить")

    a = PVT.OilGeneral()
    pvt_plot(a)


def test2():
    g = PVT.GasGeneral()
    print(g.pseudo_pressure_mpa)


def test1():
    a = PVT.OilGeneral()
    a.calc(230, 59)
    print(a.rs_m3m3)

    pp = np.arange(1, 300, 20)
    rs = []
    for p in pp:
        a.calc(p, 20)
        rs.append(a.rs_m3m3)
    plt.plot(pp, rs)
    plt.show()

test3()
