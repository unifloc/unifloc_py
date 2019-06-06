
import math
from scipy.optimize import fsolve

# TODO реализовать нормально ГОСТ, отрефакторить, учитывать разные формы кабеля

class Cable():
    def __init__(self):
        # T - длительно допустимая температура нагрева жил кабеля, C
        # Tср - температура окружающей среды

        self.sigma_isolation = 400  # удельное сопротивелении изоляции на 1000 м
        self.sigma_liquid = 104
        self.sigma_oil = 425
        self.sigma_gas = 1100
        self.sigma_o = 750  # материала оболочки
        self.sigma_b = 1000  # материала бандажа поверх оболочки
        self.sigma_p = 3  # материала подушки под броней
        self.t_max_c = 120  # длительно допустимая температура нагрева жилы
        self.R = 1.15  # электрическое сопротивление токопроводящей жилы
        self.d = 4.5  # номинальный диаметр токопроводящей жилы, мм
        self.d1 = 7.5  # номинальный наружный диаметр первого слоя изоляции жилы, мм
        self.d2 = 7.5  # номинальный наружный диаметр второго слоя изоляции жилы, мм
        self.do = 10  # номинальный диаметр оболочки жилы, мм
        self.db = 11  # номинальный наружный диаметр бандажа поверх оболочки жилы
        self.Ds = 20  # номинальный диаметр по скрутке жил, мм
        self.Dp = 12  # номинальный наружный диаметр подушки под броней
        self.D = 30  # максимальный наружный диаметр круглого кабеля
        # максимальные наружные размеры плоского кабеля
        self.Dtr = 120  # внутренний диаметр обсадной трубы скважины
        self.H = 12.5  # толщина
        self.B = 36  # ширина
        self.alpha = 0.0038  # температурный коэффициент электрического сопротивления материала
        # токопроводящей жилы, С-1

    def s_c(self, sigmai, dn, Ds, d):
        # dn - наружный диаметр основной жилы под подушкой, мм, численно равный
        # d1, d2, do или db в зависимости от конструкции кабеля
        result = sigmai / 6 / 3.14 * math.log(Ds ** 3 / 6.8 / dn ** 2 / d)
        return result  # тепловое сопротивление кабеля

    def s_env(self, D, Dtr, sigma_water, sigma_oil):
        # Тепловое сопротивление по Б.2.2.1 в скважинной жидкости нефтяной скважины
        result = 1 / 2 / 3.14 * 10 * (sigma_oil * (1 / D + 1 / Dtr) + sigma_water / D)
        return result

    def rt(self, R, t, alpha):
        # электрическое сопротивление токопроводящей жилы, Ом
        result = R * (1 + alpha * (t - 20))
        return result

    def calc_i(self, t, t_env, s_c, s_env, rt):
        # длительно допустимый ток I, A
        result = ((t - t_env) * 10 ** 5 / 3 / (s_c + s_env) / rt) ** (1 / 2)
        return result

    def tmax(self, s_c, s_env, rt, i, t):
        result = t + 3 * (s_c + s_env) * rt * i ** 2 / 10 ** 5
        return result

    def t_cabel(self, tf_c, i, rt, s_cable, s_env):
        result = (i ** 2) * (s_cable + s_env) * rt * 3 / 10 ** 5 + tf_c
        return result

    def calc_t_max_cable_c(self, tf_c, i):

        delta0 = tf_c * 0 + 10  # начальное приближение

        def calc_temp_cable(val_t_cabel1):
            s_c_val = self.s_c(self.sigma_isolation, self.d1, self.Ds, self.d)
            s_env_val = self.s_env(self.D, self.Dtr, self.sigma_gas, self.sigma_oil)
            rt_val = self.rt(self.R, val_t_cabel1, self.alpha)
            val_t_cabel2 = self.t_cabel(tf_c, i, rt_val, s_c_val, s_env_val)
            return val_t_cabel2 - val_t_cabel1

        result = fsolve(calc_temp_cable, delta0)  # находит такое val_t_cabel1, при котором calc_temp_cable = 0
        return result

    def calc_i_max_a(self, t_max_c, t_env_c):
        self.t_max_c = t_max_c
        self.t_env_c = t_env_c
        s_c_val = self.s_c(self.sigma_isolation, self.d1, self.Ds, self.d)
        s_env_val = self.s_env(self.D, self.Dtr, self.sigma_gas, self.sigma_oil)
        rt_val = self.rt(self.R, self.t_max_c, self.alpha)
        return self.calc_i(self.t_max_c, self.t_env_c, s_c_val, s_env_val, rt_val)
