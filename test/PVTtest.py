import PVT
import PVT_correlations
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


def test4():
    fl = PVT.FluidMcCain()
    p_bar = 100
    t_C = 20
    fl.calc(p_bar, t_C)
    print('P, bar = ', p_bar)
    print('T, C = ', t_C)
    print('Oil PVT properties ')
    print('Bubble point pressure, bar = ', fl.pb_bar)
    print('Gas-oil ratio, m3/m3 = ', fl.rs_m3m3)
    print('Viscosity, cP = ', fl.mu_oil_cP)
    print('Compressibility, 1/bar = ', fl.compr_oil_1bar)
    print('Formation Volume Factor, m3/m3 = ', fl.bo_m3m3)
    print('Density, kg/m3 = ', fl.rho_oil_kgm3)
    print('Gas PVT properties ')
    print('Z-factor = ', fl.z)
    print('Viscosity, cP = ', fl.mu_gas_cP)
    print('Compressibility, 1/bar = ', fl.compr_gas_1bar)
    print('Formation Volume Factor, m3/m3 = ', fl.bg_m3m3)
    print('Water PVT properties ')
    print('Density, kg/m3 = ', fl.rho_wat_kgm3)
    print('Viscosity, cP = ', fl.mu_wat_cP)
    print('Compressibility, 1/bar = ', fl.compr_wat_1bar)
    print('Formation Volume Factor, m3/m3 = ', fl.bw_m3m3)
    # Графички
    p_0 = 10
    p_n = 400
    dp = 5
    p_bar = np.arange(p_0, p_n, dp)
    t_C = 150
    pb = []
    rs = []
    mu_oil = []
    compr_oil = []
    bo = []
    rho_oil = []
    z = []
    mu_gas = []
    compr_gas = []
    bg = []
    rho_wat = []
    mu_wat = []
    compr_wat = []
    bw = []
    for p in p_bar:
        fl.calc(p, t_C)
        pb.append(fl.pb_bar)
        rs.append(fl.rs_m3m3)
        mu_oil.append(fl.mu_oil_cP)
        compr_oil.append(fl.compr_oil_1bar)
        bo.append(fl.bo_m3m3)
        rho_oil.append(fl.rho_oil_kgm3)
        z.append(fl.z)
        mu_gas.append(fl.mu_gas_cP)
        compr_gas.append(fl.compr_gas_1bar)
        bg.append(fl.bg_m3m3)
        rho_wat.append(fl.rho_wat_kgm3)
        mu_wat.append(fl.mu_wat_cP)
        compr_wat.append(fl.compr_wat_1bar)
        bw.append(fl.bw_m3m3)

    plt.subplots_adjust(left=0.04, right=0.999, top=0.98, bottom=0.04)  # чтобы пошире и было меньше наплывания

    """Давление насыщения"""
    plt.subplot(351)
    plt.ylim(np.min(pb) - 10, np.max(pb) + 10)
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Давление насыщения', color='black', family='fantasy')
    plt.ylabel('Pb, бар', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, pb, 'b', linewidth=3)

    """Газссодержание нефти"""
    plt.subplot(352)
    plt.ylim(0, np.max(rs) + 10)
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Газосодержание', color='black', family='fantasy')
    plt.ylabel('Rs, м3/м3', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, rs, 'b', linewidth=3)

    """Вязкость нефти"""
    plt.subplot(353)
    plt.ylim(0, np.max(mu_oil) + 5)
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Вязкость', color='black', family='fantasy')
    plt.ylabel('Mu, сП', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, mu_oil, 'b', linewidth=3)

    """Сжимаемость нефти"""
    plt.subplot(354)
    plt.ylim(0, 1.1 * np.max(compr_oil))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Сжимаемость', color='black', family='fantasy')
    plt.ylabel('co, 1/бар', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, compr_oil, 'b', linewidth=3)

    """Объемный коэффициент нефти """
    plt.subplot(355)
    plt.ylim(0.9 * np.min(bo), 1.1 * np.max(bo))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Объемный коэффициент нефти', color='black', family='fantasy')
    plt.ylabel('bo,м3/м3', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, bo, 'b', linewidth=3)

    """Плотность нефти """
    plt.subplot(356)
    plt.ylim(0.9 * np.min(rho_oil), np.max(rho_oil) + 10)
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Плотность нефти', color='black', family='fantasy')
    plt.ylabel('rho, кг/м3', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, rho_oil, 'b', linewidth=3)

    """z-фактор """
    plt.subplot(357)
    plt.ylim(0.9 * np.min(z), 1.1 * np.max(z))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('z - фактор', color='black', family='fantasy')
    plt.ylabel('z', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, z, 'b', linewidth=3)

    """Вязкость газа """
    plt.subplot(358)
    plt.ylim(0, 1.1 * np.max(mu_gas))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Вязкость газа', color='black', family='fantasy')
    plt.ylabel('mu_gas, сП', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, mu_gas, 'b', linewidth=3)

    """Сжимаемость газа """
    plt.subplot(359)
    plt.ylim(0, 1.1 * np.max(compr_gas))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Сжимаемость газа', color='black', family='fantasy')
    plt.ylabel('compr_gas, 1/бар', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, compr_gas, 'b', linewidth=3)

    """Объемный коэффициент газа """
    plt.subplot(3, 5, 10)
    plt.ylim(0, 1.1 * np.max(bg))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Объемный коэффициент газа', color='black', family='fantasy')
    plt.ylabel('bg, м3/м3', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, bg, 'b', linewidth=3)

    """Плотность воды """
    plt.subplot(3, 5, 11)
    plt.ylim(0.9 * np.min(rho_wat), 1.1 * np.max(rho_wat))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Плотность воды', color='black', family='fantasy')
    plt.ylabel('rho_wat, кг/м3', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, rho_wat, 'b', linewidth=3)

    """Вязкость воды """
    plt.subplot(3, 5, 12)
    plt.ylim(0.9 * np.min(mu_wat), 1.1 * np.max(mu_wat))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Вязкость воды', color='black', family='fantasy')
    plt.ylabel('mu_wat, сП', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, mu_wat, 'b', linewidth=3)

    """Сжимаемость воды """
    plt.subplot(3, 5, 13)
    plt.ylim(0.9 * np.min(compr_wat), 1.1 * np.max(compr_wat))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Сжимаемость воды', color='black', family='fantasy')
    plt.ylabel('compr_wat, 1/бар', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, compr_wat, 'b', linewidth=3)

    """Объемный коэф """
    plt.subplot(3, 5, 14)
    plt.ylim(0.9 * np.min(bw), 1.1 * np.max(bw))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Объемный коэффициент воды', color='black', family='fantasy')
    plt.ylabel('bw, м3/м3', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, bw, 'b', linewidth=3)
    plt.show()



test4()