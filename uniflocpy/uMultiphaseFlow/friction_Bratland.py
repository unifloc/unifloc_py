"""
Модуль для рассчета коэффициента трения
по данным книги:
Bratland O. Pipe flow 1: single-phase flow assurance
//Fonte: http://www.drbratland.com/download-two-free-books-on-flow-assurance. – 2009.
"""
import scipy.optimize as sp  # модуль для решения уравения
import scipy.optimize._minimize as minimize
import math
import numpy as np

class Friction():
    """
    Модуль-класс для расчета коэффициента трения f
    в зависимости от числа Рейнольдса, абс. шероховатости и диаметра трубы.
    """
    def __init__(self):
        """
        Консруктор класса

        :param number_re: безразмерное число Рейнольдса
        :param epsilon_m: абсолюная шероховатость
        :param d_m: внутренний диаметр трубы
        """
        self.number_re = None
        self.absolute_roughness_m = None
        self.d_m = None
        self.relative_roughness = None  # относительная шероховатость
        self.f = None  # итоговый коэффициент трения
        self.u_s = 1  # подстроечный параметр для сбивки к экспериментальным исследованиям

    def __correlation__(self, f):
        """
        Основная корреляция Ove Bartland для расчета коэффициента трения

        При решении уравнения с помощью fsolve результат сохраняется в атрибуте

        :param f: коэффициент трения
        :return: разница между расчитанным f и приближенным f для примения fsolve
        """
        self.f = f
        in_log10_part_first = ((1.547 / self.number_re / self.f ** (1/2))**(0.9445 * self.u_s))
        in_log10_part_second = ((self.absolute_roughness_m / 3.7 / self.d_m) ** self.u_s)
        result = (1 / ( - 2 / self.u_s * np.log10(in_log10_part_first + in_log10_part_second)))**2
        return result - self.f

    def calc_f(self, number_re, epsilon_m, d_m):
        """
        Метод для расчета коэффициента трения

        :param number_re: Число Рейнольдса
        :param epsilon_m: Абсолютная шероховатость
        :param d_m: Внутрениний диаметр трубы
        :return: коэффициент трения f
        """
        self.number_re = number_re
        self.absolute_roughness_m = epsilon_m
        self.d_m = d_m
        self.relative_roughness = self.absolute_roughness_m / self.d_m
        method = 'excitingmixing'
        if self.number_re <= 2300:  # Laminar
            self.f = 64 / self.number_re
        if 2300 < self.number_re <= 3100:  # Turbulent
            p1 = 64 / 2300
            p2 = 0.04
            self.f = p1 + (p2 - p1) / (3100 - 2300) * (self.number_re - 2300)  # TODO должна быть прямая линия
        if 3100 < self.number_re <= 20000:
            mistake = sp.root(self.__correlation__, 0.02, method=method).x  # TODO разобраться с выбором начального приближения
            p3 = float(self.f)
            p2 = 0.04
            self.f = p2 + (p3 - p2) / (20000 - 3100) * (self.number_re - 3100)  # TODO должна быть прямая линия
        if self.number_re > 20000:
            mistake = sp.root(self.__correlation__, 0.02, method=method).x
        self.f = float(self.f)  # fsolve выдает numpy array, перевод в float
        return self.f

