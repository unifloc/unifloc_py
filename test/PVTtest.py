import PVT
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
import pylab
import PVT_correlations

mpl.rcParams['font.family'] = 'fantasy'
mpl.rcParams['font.fantasy'] = 'Times New Roman'


def get_z_curve_StandingKatz(tpr):
    """
    Функция позволяет считать данные из нужного файла в зависимости от входного tpr и построить график
    Допустимые значения tpr = 1.05, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.2, 2.4, 2.6, 2.8, 3
    :param tpr: температура приведенная
    :return: данные из графика Cтендинга для этой температуры
    """
    data = pd.read_csv('Standing-Katz Chart Data\sk_tpr_{}.txt'.format(int(tpr*100)), sep=';')
    ppr = np.array(pd.DataFrame(data)['x'])
    z = np.array(pd.DataFrame(data)['y'])
    return ppr, z


# Сравним расчетный график с графиком Стендинга
tpr = 2
ppr, z = get_z_curve_StandingKatz(tpr)
z_calc = []
pogr = []
i = 0
for p in ppr:
    z_calc.append(PVT_correlations.unf_zfactor_DAK_ppr(p, tpr))
    pogr.append((z[i]-z_calc[i])/z[i] * 100)
    i += 1
pylab.subplot(211)
pylab.plot(ppr, z, label='По графикам Стендинга-Катца')
pylab.plot(ppr, z_calc, label='расчетный')
pylab.title('Сравнение графиков для tpr={}'.format(tpr))
pylab.grid()
pylab.legend()
pylab.subplot(212)
pylab.plot(ppr, pogr, label='погрешность в %')
pylab.grid()
pylab.legend()
pylab.show()

# построим все графики Стендинга сразу
tpr = [1.05, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.2, 2.4, 2.6, 2.8, 3]

for t in tpr:
    ppr_standing, z_standing = get_z_curve_StandingKatz(t)
    pylab.plot(ppr_standing, z_standing, label='tpr = {}'.format(t))
pylab.grid()
pylab.title('Графики Стендинга-Катца для различных значений tpr')
pylab.legend()
pylab.xlabel('ppr')
pylab.ylabel('z')
pylab.show()


def test4():
    fl = PVT.FluidStanding(rsb_m3m3=50)
    p_0 = 5
    p_n = 300
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

    """Объемный коэффициент """
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