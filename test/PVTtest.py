import PVT
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
import pylab
import PVT_correlations

mpl.rcParams['font.family'] = 'fantasy'
mpl.rcParams['font.fantasy'] = 'Times New Roman'

# не понимаю как считывать с файла в зависимости от входного параметра и потом строить график
# поэтому файл можно менять только с руки
# TODO разобраться как сделать динамическое считывание файлов в зависимости от входного tpr
data = pd.read_csv(r"Standing-Katz Chart Data\sk_tpr_280.txt", sep=';')
df = pd.DataFrame(data)
ppr = df['x']
z = df['y']
"""
plt.ylim(0.9 * z.min, 1.1 * z.max)
plt.xlim(0, z.max)
"""
tpr = 2.8  # менять это значение и в названии файла число в там без точек
pylab.grid(True)
pylab.title('z - фактор', color='black', family='fantasy')
pylab.ylabel('z', color='black', family='fantasy')
pylab.xlabel('Давление, бар', color='black', family='fantasy')
pppr = []
zz = []
for p in ppr:
    pppr.append(p)
    zz.append(PVT_correlations.unf_zfactor_DAK_ppr(p, tpr))
pylab.plot(ppr, z, label='По графикам Стендинга-Катца')
pylab.plot(pppr, zz, label='расчетный')
pylab.legend()
pylab.show()


def test4():
    fl = PVT.FluidMcCain()
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

    """Объемный коэффициент """
    plt.subplot(3, 5, 14)
    plt.ylim(0.9 * np.min(bw), 1.1 * np.max(bw))
    plt.xlim(0, p_n)
    plt.grid(True)
    plt.title('Объемный коэффициент воды', color='black', family='fantasy')
    plt.ylabel('bw, м3/м3', color='black', family='fantasy')
    plt.xlabel('Давление, бар', color='black', family='fantasy')
    plt.plot(p_bar, bw, 'b', linewidth=3)
    # plt.show()


test4()