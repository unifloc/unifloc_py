import math
#import matplotlib as plt
#from scipy.optimize import fsolve
#mport numpy as np

g = 9.81
pi = math.pi


class Save_temp_fluid_c(object):
    """
    Класс для сохранения значений температуры внутри функции
    temp_fluid, в которой решения выполняются итеративно
    с помощью fsolve()
    """

    def __init__(self, init):
        """
        Начальное значение температуры
        """
        self.temp_fluid_c = init

    def save(self, value):
        """
        Сохранение вычисленных значений T в класс
        """
        self.temp_fluid_c = value


class system_properties(object):
    """
    Класс для задания всех необходимых свойств для расчета КРТ
    """
    def __init__(self):
        # convective heat transfer in an Oil Well
        self.ql_m3sec = 79.5 / 86400
        self.qg_m3sec = 283 / 86400
        self.kl_wmc = 0.138
        self.kg_wmc = 1.73 * 10 ** (-4)
        self.rhol_kgm3 = 882.9
        self.rhog_kgm3 = 80.3
        self.cpl_jkgc = 2512
        self.cpg_jkgc = 2093
        self.mul_pas = 0.015
        self.mug_pas = 1.5 * 10 ** (-4)
        self.rti_m = 0.0259

        # natural convection in well annulus
        self.rto_m = 0.0561
        self.rci_m = 0.0797
        # parametres are medium
        self.mu_an_pas = 0.0001
        self.cp_an_jkgc = 1004.81
        self.rho_an_kgm3 = 36.92
        self.k_an_wmc = 0.865
        self.betta_1c = 0.004824
        self.delta_temp_an_c = 3

        # overall heat transfer coefficient
        self.rco_m = 0.0889
        self.rwb_m = 0.1079
        self.kcem_wmc = 0.779
        self.kt_wmc = 25
        self.ke_wmc = 2.422

        self.time_sec = 100 * 7 * 24 * 3600
        self.rhoe_kgm3 = 2504
        self.cpe_jkgc = 1256

        self.tei_c = 93.3
        self.distance_m = 1000
        self.gg_cm = 0.027
        self.gamma_gas = 0.65
        self.gamma_api = 29

        self.p_pa = 792.9 * 10 ** 3

        #переменные, необходимые для реализации примеров
        '''self.pwh = 115
        self.mt_kgs = 1.07
        self.rp_sm3sm3 = 3.56'''


def_prop = system_properties()  # создание класса стандартный свойств


def q_motor_v(a_motor, d_motor_m, l_motor_m, t_wall_motor_c, tf_c):
    """
    Тепловой поток, выделяемый электродвигателем, передаваемый теплоотдачей обтекаемой скважинной продукции
    :param a_motor: коэффициент теплоотдачи от поверхности стенки э/д,
    :param d_motor_m: диаметр э/д, м
    :param l_motor_m: длина э/д, м
    :param t_wall_motor_c: температура стенок э/д, С
    :param tf_c: температура скважинной продукции, С
    :return: тепловой поток, Ватт
    """
    return a_motor * pi * d_motor_m * l_motor_m * (t_wall_motor_c - tf_c)


def t_wall_motor_c(tf_c, n_motor_v, efficiency_perc, a_motor, d_motor_m, l_motor_m):
    efficiency = efficiency_perc / 100
    return tf_c + n_motor_v * (1 - efficiency) / a_motor / pi / d_motor_m / l_motor_m


def de_m(dwb_m, ae_m2c, time_sec):
    """
    Диаметр окружающей ГП, соответствующей естесственной температуры
    :param dwb_m: диаметр скважины (цементного кольца), м
    :param ae_m2c: коэффициент температуропроводности ГП, м2 / сек
    :param time_sec: время прогрева, сек
    :return: диаметр возмущенной зоны, м
    """
    return dwb_m + 4 * math.sqrt(ae_m2c + time_sec)


def a_complex(dti_m, k_vm3c, gf_kgsec, cpf_jkgc):
    """
    Размерный комплекс А, учитывающий особенности теплообмена
    :param dti_m: внутренний диаметр трубы, где поднимается флюид, м ???
    :param k_vm3c: коэффициент теплопередачи от добываемого флюида в ГП, Ватт / м2 / С
    :param gf_kgsec: массовый расход флюида, кг / сек
    :param cpf_jkgc: теплоемкость флюида, Дж / кг / С
    :return: А, 1/м
    """
    return pi * dti_m * k_vm3c / gf_kgsec / cpf_jkgc


def m_complex(ql_vm, gf_kgsec, cpf_jkgc):
    """
    Размерный комплекс м, учитывающий особенности теплообмена
    :param ql_vm: линейная плотность внутренних источников (+) и стоков (-) теплоты, Вт / м
    :param gf_kgsec: массовый расход флюида, кг / сек
    :param cpf_jkgc: теплоемкость флюида, Дж / кг / С
    :return: m, К / м
    """
    return ql_vm / gf_kgsec / cpf_jkgc


