# -*- coding: utf-8 -*-
"""
Created on Mon Sept 04 2017

@author: rnt

UniflocPy

класс для расчета PVT свойств углеводородных флюидов и воды

"""
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
mpl.rcParams['font.family'] = 'fantasy'
mpl.rcParams['font.fantasy'] = 'Times New Roman'

class ComponentGeneral:
    """
    Абстрактный класс для описания компонентоы углеводородных флюидов
    """

    def __init__(self):
        self.gamma = 1  # specific gravity of component, dimensionless
        self.rho_kgm3 = 800  # density with dimension
        self.mu_cp = 1  # dynamic viscosity
        """ термобарические условия """
        self._p_bar = 1
        self._t_c = 15

    def calc(self, p_atm, t_c):
        """ recalculate all parameters according to some pressure and temperature"""
        return 1

    """Временно сюда построитель графиков"""
    def pvt_plot(self, p_0=1, p_n=300, dp=20, t_c=20):
        """Газосодержание"""
        plt.subplot(321)
        rs = []
        pp = np.arange(p_0, p_n, dp)
        for p in pp:
            self.calc(p, t_c)
            rs.append(self.rs_m3m3)
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
            self.calc(p, t_c)
            rho.append(self.rho_kgm3)
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
            self.calc(p, t_c)
            Bo.append(self._bo_m3m3)
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
            self.calc(p, t_c)
            mu.append(self._mu_cp)
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
            self.calc(p, t_c)
            co.append(self._co_1atm)
        plt.ylim(0, np.max(co))
        plt.xlim(0, p_n)
        plt.grid(True)
        plt.title('Сжимаемость нефти', color = 'black', family = 'fantasy')
        plt.ylabel('Со, 1/атм', color = 'black', family = 'fantasy')
        plt.xlabel('Давление, атм', color = 'black', family = 'fantasy')
        plt.plot(pp, co, 'm', linewidth=3)
        plt.show()

        input("\nНажмите Enter, чтобы продолжить")

class GasGeneral(ComponentGeneral):
    """
    Класс для описания свойств углеводородных газов
    """

    def __init__(self):
        super().__init__()
        self._z = 0.9  # сверхсжимаемость

    @property
    def z(self):
        return self._z


class OilGeneral(ComponentGeneral):
    """
    Класс для описания свойств нефти по модели нелетучей нефти
    """

    def __init__(self):
        super().__init__()  # часть базовых свойств наследуется
        self._gas = GasGeneral()  # create gas component
        self.rsb_m3m3 = 100

        self.pb_calibr_bar = 100  # калибровочное значение давления насыщения
        self.tb_calibr_c = 50  # температуры для калибровки по давлению насыщения
        self.bob_calibr_m3m3 = 1.2  # калибровочное значение объемного коэффициента
        self.muob_calibr_cp = 1  # калибровочное значение вязкости при давлении насыщения
        self.rhob_calibr_kgm3 = 700  # калибровочное значение плотности при давлении насыщения

        """ расчетные свойства """
        self._rs_m3m3 = 1
        self._bo_m3m3 = 1
        self._mu_cp = 1
        self._co_1atm = 1

    @property
    def gas(self):
        return self._gas

    @property
    def rs_m3m3(self):
        """ газосодержание """
        return self._rs_m3m3

    def _calc_rs_m3m3(self, p_bar, t_c):
        """ тут должна быть реализация расчета газосодержания
        """
        if p_bar < self.pb_calibr_bar:
            return self.rsb_m3m3 / self.pb_calibr_bar * p_bar
        else:
            return self.rsb_m3m3

    def _calc_rho_kgm3(self, p_bar, t_c):
        """ тут должна быть реализация расчета плотности нефти
        """
        if p_bar < self.pb_calibr_bar:
            return -self.rhob_calibr_kgm3 / self.pb_calibr_bar * p_bar + 1.8 * self.rhob_calibr_kgm3
        else:
            return self.rhob_calibr_kgm3

    def _calc_bo_m3m3(self, p_bar, t_c):
        """ тут должна быть реализация расчета объемного коэффициента нефти
        """
        if p_bar < self.pb_calibr_bar:
            return self.bob_calibr_m3m3 / self.pb_calibr_bar * p_bar
        else:
            return self.bob_calibr_m3m3

    def _calc_mu_cp(self, p_bar, t_c):
        """ тут должна быть реализация расчета вязкости нефти
        """
        if p_bar < self.pb_calibr_bar:
            return -self.muob_calibr_cp / self.pb_calibr_bar * p_bar + 2 * self.muob_calibr_cp
        else:
            return self.muob_calibr_cp

    def _calc_co_1atm(self, p_bar, t_c):
        """ тут должна быть реализация расчета сжимаемости нефти
        """
        return (28.1 * self.rsb_m3m3 + 30.6 * (t_c + 273) - 1180
                * self._gas.gamma + 1784 / self.gamma - 10910) \
                / (100000 * p_bar)

    def calc(self, p_atm, t_c):
        """ реализация расчета свойств нефти """
        self._rs_m3m3 = self._calc_rs_m3m3(p_atm, t_c)
        self.rho_kgm3 = self._calc_rho_kgm3(p_atm, t_c)
        self._bo_m3m3 = self._calc_bo_m3m3(p_atm, t_c)
        self._mu_cp = self._calc_mu_cp(p_atm, t_c)
        self._co_1atm = self._calc_co_1atm(p_atm, t_c)


if __name__ == "__main__":
    print("Вы запустили модуль напрямую, а не импортировали его.")
    input("\n\nНажмите Enter, чтобы выйти.")
