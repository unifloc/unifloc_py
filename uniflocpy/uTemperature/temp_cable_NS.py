
"""ГОСТ Р 51777-2001 Кабели для установок погружных электронасосов.

 Общие технические условия (с Поправкой) """
import math
from scipy.optimize import fsolve


# TODO реализовать нормально ГОСТ, отрефакторить, учитывать разные формы кабеля
# TODO толщины слоев сделать
# TODO рисунок кабеля при инициализации


class Cable():
    def __init__(self):
        # T - длительно допустимая температура нагрева жил кабеля, C
        # Tср - температура окружающей среды
        # У.T.С.Т. - удельное тепловое сопротивление теплоперехода

        self.sigma_liquid__Ccm_V = 104  # У.Т.С.Т. от поверхности кабеля в воду и от воды к ОТ, C * см^ 2 /Вт
        self.sigma_oil = 425  # У.Т.С.Т. от поверхности кабеля в скважинную жидкость (нефть) и от нефти к ОТ, C*см^ 2/Вт
        self.sigma_gas = 1100  # У.Т.С. теплоизлучению от поверхности кабеля в воздушную среду

        self.sigma_polyethylene_Ccm_V = 400  # У.Т.С. полиэтилена, композиции полипропилена и сополимеров пропилена
        self.sigma_thermoplastic_elastomers_Ccm_V = 600  # У.Т.С. термоэластопласта
        self.sigma_rubber_Ccm_V = 750  # У.Т.С. резины
        self.sigma_fluorocopolymers_Ccm_V = 1000  # У.Т.С. фторсополимеров
        self.sigma_braids_ribbons_Ccm_V = 650  # У.Т.С. материалов оплеток и лент для наложения бандажей и подушек
        self.sigma_plumbum_Ccm_V = 3  # У.Т.С. свинца и его сплавов

        self.sigma_1isolation_Ccm_V = self.sigma_polyethylene_Ccm_V  # удельное сопротивеление изоляции на C*см/Вт
        self.sigma_2isolation_Ccm_V = self.sigma_polyethylene_Ccm_V
        self.sigma_shell_Ccm_V = self.sigma_polyethylene_Ccm_V
        self.sigma_bandage_Ccm_V = self.sigma_braids_ribbons_Ccm_V
        self.sigma_pillow_Ccm_V = self.sigma_braids_ribbons_Ccm_V

        self.sigma_o = 750  # материала оболочки
        self.sigma_b = 1000  # материала бандажа поверх оболочки
        self.sigma_p = 3  # материала подушки под броней

        self.t_permanently_permissible_c = 120  # длительно допустимая температура нагрева жилы
        self.R = 1.15  # электрическое сопротивление токопроводящей жилы

        self.d_mm = 4.5  # номинальный диаметр токопроводящей жилы, мм
        self.d1_first_isolation_mm = 7.5  # номинальный наружный диаметр первого слоя изоляции жилы, мм
        self.d2_second_isolation_mm = 7.5  # номинальный наружный диаметр второго слоя изоляции жилы, мм
        self.do_shell_mm = 10  # номинальный диаметр оболочки жилы, мм
        self.db_bandage_mm = 11  # номинальный наружный диаметр бандажа поверх оболочки жилы
        self.Dc_twist_mm = 20  # номинальный диаметр по скрутке жил, мм
        self.Dp_pillow_mm = 12  # номинальный наружный диаметр подушки под броней
        self.D_round_cable_mm = 30  # максимальный наружный диаметр круглого кабеля

        # максимальные наружные размеры плоского кабеля
        self.H_flat_cable_mm = 12.5  # толщина
        self.B_flat_cable_mm = 36  # ширина

        self.di_casing_mm = 120  # внутренний диаметр обсадной трубы скважины
        self.alpha_1C = 0.0038  # температурный коэффициент электрического сопротивления материала
        # токопроводящей жилы, С-1

        self.cabel_type = 'Round'  # Или 'Flat'
        self.environment_type = 'Oil'  # в нефти , 'Water' - в воде


    def __thermal_resistance_cable__(self):
        """Расчет теплового сопротивления кабеля"""
        result = (1 / 6 / math.pi *(self.sigma_1isolation_Ccm_V * math.log(self.d1_first_isolation_mm / self.d_mm) +
                                   self.sigma_shell_Ccm_V * math.log(self.do_shell_mm / self.d1_first_isolation_mm  ) +
                                   self.sigma_bandage_Ccm_V * math.log(self.db_bandage_mm / self.do_shell_mm)) +
                  self.sigma_pillow_Ccm_V / 2 / math.pi * math.log( self.D_round_cable_mm/ self.Dc_twist_mm))  # TODO проверить диаметр подушки
        return result

    def __thermal_resistance_environment__(self):
        """Расчет теплового сопротивления окружающей среды"""
        # Тепловое сопротивление по Б.2.2.1 в скважинной жидкости нефтяной скважины
        if self.cabel_type == 'Round' and self.environment_type == 'Oil':
            return (1 / 2 / math.pi * 10 * (self.sigma_oil *
                                            (1 / self.D_round_cable_mm + 1 / self.di_casing_mm) +
                                            self.sigma_gas / self.D_round_cable_mm))
        if self.cabel_type == 'Flat' and self.environment_type == 'Oil':
            return (1 / 2 * 10 * ( self.sigma_oil * (1 / (1.14 * self.H_flat_cable_mm + 2 * self.B_flat_cable_mm ) +
                                                     1 / math.pi / self.di_casing_mm) +
                                   self.sigma_gas / (1.14 * self.H_flat_cable_mm + 2 * self.B_flat_cable_mm ) ))


    def __electricial_resistance_cable_core__(self, R, t, alpha):
        """Расчет электрического сопротивления жилы кабеля"""
        # электрическое сопротивление токопроводящей жилы, Ом
        result = R * (1 + alpha * (t - 20))
        return result

    def __calc_i_a__(self, t, t_env, s_c, s_env, rt):
        """Расчет длительно допустимого тока"""
        # длительно допустимый ток I, A
        result = ((t - t_env) * 10 ** 5 / 3 / (s_c + s_env) / rt) ** (1 / 2)
        return result

    def __t_cabel_c__(self, tf_c, i, rt, s_cable, s_env):
        """Температура кабеля"""
        result = (i ** 2) * (s_cable + s_env) * rt * 3 / 10 ** 5 + tf_c
        return result

    def calc_t_max_cable_c(self, tf_c, i_a):
        """
        Расчет температуры кабеля

        :param tf_c: Температура среды, С
        :param i_a: Ток жилы кабеля, А
        :return: температуры кабеля, С
        """
        delta0 = tf_c * 0 + 10  # начальное приближение

        def calc_temp_cable(val_t_cabel1):
            s_c_val = self.__thermal_resistance_cable__()
            s_env_val = self.__thermal_resistance_environment__()
            rt_val = self.__electricial_resistance_cable_core__(self.R, val_t_cabel1, self.alpha_1C)
            val_t_cabel2 = self.__t_cabel_c__(tf_c, i_a, rt_val, s_c_val, s_env_val)
            return val_t_cabel2 - val_t_cabel1
        result = fsolve(calc_temp_cable, delta0)  # находит такое val_t_cabel1, при котором calc_temp_cable = 0
        return result

    def calc_i_max_a(self, t_max_c, t_env_c):
        """
        Расчета максимально допустимой длительной силы тока кабеля

        :param t_max_c: температурный индекс кабеля, максимальная температуры нагрева жил кабеля, С
        :param t_env_c: температура среды
        :return: длительно допустимый ток, А
        """
        self.t_permanently_permissible_c = t_max_c
        self.t_env_c = t_env_c
        s_c_val = self.__thermal_resistance_cable__()
        s_env_val = self.__thermal_resistance_environment__()
        rt_val = self.__electricial_resistance_cable_core__(self.R, self.t_permanently_permissible_c, self.alpha_1C)
        return self.__calc_i_a__(self.t_permanently_permissible_c, self.t_env_c, s_c_val, s_env_val, rt_val)


