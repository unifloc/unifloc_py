import math

const_g_m2sec = 9.8


def number_Fr(vm_msec, diametr_m):
    """
    (4.109)
    :param vm_msec: общая приведенная скорость смеси (3.12)
    :param diametr_m: внутренний диаметр трубы
    :return:  безразмерное число Фруда
    """
    return vm_msec ** 2 / const_g_m2sec / diametr_m


def CL(q_liquid, q_gas):
    """
    (3.8)
    CL - input liquid content (no slip liquid holdup CL)
    :param q_liquid: дебит жидкости при данных условиях
    :param q_gas: дебит газа при данных условиях
    :return: Объемное содержание жидкости в потоке
    """
    return q_liquid / (q_liquid + q_gas)


def define_flow_regime(CL, number_Fr):
    """
    (4.110 - 4.113)
    :param CL:  Объемное содержание жидкости в потоке
    :param number_Fr: безразмерное число Фруда
    :return: номер режима
        0 - Segregated Flow - разделенный режим
        1 - Intermittent Flow - прерывистый режим
        2 - Distributed Flow - распределенный режим
        3 - Transition flow - переходный режим
    """
    L1 = 316 * CL ** 0.302
    L2 = 0.0009252 * CL ** (-2.4684)
    L3 = 0.1 * CL ** (-1.4516)
    L4 = 0.5 * CL ** (-6.738)
    if (CL < 0.01 and number_Fr < L1) or (CL >= 0.01 and number_Fr < L2):
        # Segregated Flow
        return 0
    if (0.01 <= CL < 0.4 and L3 < number_Fr <= L1) or (CL >= 0.4 and L3 < number_Fr <= L4):
        # Intermittent Flow
        return 1
    if (CL < 0.4 and number_Fr >= L1) or (CL >= 0.4 and number_Fr > L4):
        # Distributed Flow
        return 2
    if L2 <= number_Fr < L3 and CL >= 0.01:
        # Transition Flow
        return 3


def EL(CL, number_Fr, flow_regime):
    """
    (4.114)
    :param CL: Объемное содержание жидкости в потоке
    :param number_Fr: безразмерное число Фруда
    :param flow_regime: номер режима
    :return: liquid holdup for horizontal flow EL(0) = HL0 - объемное газосодерждание для горизонтальной трубы
    """
    if flow_regime == 0:
        # Segregated Flow
        a = 0.98
        b = 0.4846
        c = 0.0868
    if flow_regime == 1:
        # Intermittent Flow
        a = 0.845
        b = 0.5351
        c = 0.0173
    if flow_regime == 2:
        # Distributed Flow
        a = 1.065
        b = 0.5824
        c = 0.0609
    return a * CL ** b / number_Fr ** c


def correction_factor_B(betta, angle):
    """
    (4.116)
    :param betta: корректирующий фактор
    :param angle: угол от горизонтали, град
    :return: Корректирующий фактор B для учета угла наклона трубы
    """
    return 1 + betta * (math.sin(1.8 * angle) - (1 / 3) * (math.sin(1.8 * angle)) ** 3)


def correction_factor_betta(CL, number_Fr, Nlv, flow_regime):
    """
    (4.117)
    :param CL: Объемное содержание жидкости в потоке
    :param number_Fr: число Фруда
    :param Nlv: Liquid velocity number - безразмерное число Данса и Роса - показатель скорости жидкости (4.3)
    :param flow_regime: номер режима потока
    :return:  C - коэффициент для поправки на угол наклона трубы
    """

    # coefficienta for uphill
    if flow_regime == 0:
        d = 0.011
        e = -3.768
        f = 3.539
        g = - 1.614
    if flow_regime == 1:
        d = 2.96
        e = 0.305
        f = -0.4473
        g = 0.0978
    if flow_regime == 2:
        return 0
    result = (1 - CL) * math.log(d * CL ** e * Nlv ** f * number_Fr ** g)
    if result <= 0:
        return 0
    else:
        return result


def Nlv(vsl_m3sec, rho_liquid_kgm3, sigma):
    """
    (4.3)
    :param vsl_m3sec: no slip liquid velocity -  приведенная скорость жидкости (3.10)
    :param rho_liquid_kgm3: плотность жидкости без проскальзывания (аддитивно нефть и вода) (3.16)
    :param sigma: поверхностное натяжения на границе газ-жидкость
    :return: Liquid velocity number - безразмерное число Данса и Роса - показатель скорости жидкости
    """
    return vsl_m3sec * (rho_liquid_kgm3 / (const_g_m2sec * sigma)) ** (1 / 4)


def EL_for_transition_flow(L2, L3, number_Fr, EL_segregated, EL_intermittent):
    """
    (4.118)
    :param L2: граница режима (4.111)
    :param L3: граница режима течения (4.112)
    :param number_Fr: число Фруда
    :param EL_segregated:  объемное содержание жидкости с поправкой на угол наклона для расслоенного режима
    :param EL_intermittent: объемное содержание жидкости с поправкой на угод наклона для прерывистого режима
    :return: объемное содержание жидкости с поправкой на угод наклона для переходного режима
    """
    A = (L3 - number_Fr) / (L3 - L2)
    B = 1 - A
    return A * EL_segregated + B * EL_intermittent


def mixture_density(rho_liquid_kgm3, rho_gas_kgm3, liquid_content):
    """
    (3.22 - 3.23)
    :param rho_liquid_kgm3: плотность жидкости
    :param rho_gas_kgm3: плотность газа
    :param liquid_content: объемное содержание жидкости в потоке
    :return: плотность смеси
    """
    return rho_liquid_kgm3 * liquid_content + rho_gas_kgm3 * (1 - liquid_content)


def viscosity_mixture(mu_liquid_kgm3, mu_gas_kgm3, liquid_content):
    """
    (3.21)
    :param mu_liquid_kgm3: вязкость жидкости
    :param mu_gas_kgm3: вязкость газа
    :param liquid_content: Объемное содержание жидкости в потоке
    :return: вязкость смеси
    """
    return mu_liquid_kgm3 * liquid_content + mu_gas_kgm3 * (1 - liquid_content)


def number_Re(rho, v, d, mu):
    return rho * v * d / mu


def friction_coefficient(Re):
    # TODO исправить коэффициент
    """
    (2.12 и 2.13)
    :param Re:
    :return: коэффициент трения
    """
    if Re < 3000:
        return 64 / Re
    if 3000 < Re:
        return 0.0056 + 0.5 * Re ** (-0.32)


def Pain_cor_for_volume_liquid_content(volume_liquid_content, angle_grad):
    """
    (4.126 и 4.127)
    :param volume_liquid_content: объемное содежание жидкости с поправкой на угол наклона
    :param angle_grad: угол наклона от горизонтали
    :return: объемное содежание жидкости с поправкой на угол наклона и поправкой Пэйна и др.
    """
    if angle_grad > 0:
        return 0.924 * volume_liquid_content
    else:
        return 0.685 * volume_liquid_content


def calc_s(y):
    """
    (4.123 и 4.125)
    :param y:
    :return:
    """
    if 1 < y < 1.2:
        return math.log(2.2 * y - 1.2)
    else:
        lny = math.log(y)
        return lny / (-0.0523 + 3.182 * lny - 0.8725 * lny ** 2 + 0.01853 * lny ** 4)


def true_friction_coefficient(fn, s):
    """
    (4.122)
    :param fn: нормирующий коэффициент трения
    :param s:
    :return: коэффициент трения двухфазного потока
    """
    return fn * math.exp(s)


def Ek(vm, vsg, rhon, p):
    """
    (4.53)
    :param vm:
    :param vsg:
    :param rhon:
    :param p: давление
    :return: обезразмеренная кинетическая энергия
    """
    return vm * vsg * rhon / p


def calc_result(f, rhon, vm, d, rhos, angle, ek):
    """
    (4.107)
    :param f: коэффициент трения двухфазного потока
    :param rhon:
    :param vm:
    :param d: внутренний диаметр трубы
    :param rhos: (4.108)
    :param angle: угол наклона от горизонтали
    :param ek: обезразмеренная кинетическая энергия
    :return:
    """
    return (f * rhon * vm ** 2 / 2 / d + rhos * const_g_m2sec * math.sin(angle)) / (1 - ek)


def out(a, b):
    print(str(a) + " =  " + str(b) + '\n')


class data():
    def __init__(self):
        self.mu_oil_pasec = 0.97 * 10 ** (-3)
        self.sigma_kgsec2 = 8.41 * 10 ** (-3)
        self.mu_gas_pasec = 0.016 * 10 ** (-3)

        self.epsilon_friction_m = 18.288 * 10 ** (-6)
        self.diametr_inner_m = 0.152

        self.velosity_mix_msec = 2.39

        self.oil_rate_on_surface_m3day = 1590
        self.gas_rate_on_surface_m3day = 283 * 10 ** 3
        self.water_rate_on_surface_m3day = 0

        # self.CL = 0.507 # заглушка, содержание жидкости в потоке
        # self.vsl_msec = 1.21 # заглушка, no slip liquid4 velocity

        self.Rp_m3m3 = 178  # газовый фактор
        self.Rs_m3m3 = 50.6
        self.Rsw_m3m3 = 0
        self.rho_liquid_kgm3 = 762.64
        self.rho_gas_kgm3 = 94.19
        self.oil_formation_volume_factor_m3m3 = 1.197
        self.gas_formation_volume_factor_m3m3 = 0.0091

        data.angle_grad = 90  # угол наклона ствола скважины от горизонтали

        data.Ap = None  # поперечная площадь трубы

        self.volume_oil_rate_in_condition_m3sec = 0  # объемный дебит нефти при данных условиях (P,T)
        self.volume_water_rate_in_condition_m3sec = 0  # объемный дебит воды при данных условиях (P,T)
        self.volume_liquid_rate_in_condition_m3sec = 0  # объемный дебит жидкости при данных условиях (P,T)
        self.volume_gas_rate_in_condition_m3sec = 0  # объемный дебит газа при данных условиях (P,T)

        self.liquid_content = None

        self.pressure_bar = 117.13
        self.pressure_pa = self.pressure_bar * 10 ** 5
        self.temperature_c = 82

        self.correction_factor_betta = None
        self.angle_correction_factor = None
        self.volume_liquid_content_with_angle = None

        self.vsl_msec = 0  # приведенная скорость жидкости (3.10)
        self.vsg_msec = 0  # приведенная скорость газа (3.11)
        self.vsm_msec = 0  # приведенная (общая) скорость смеси (3.12)

        self.liquid_content = 0  # объемное содержание жидкости при отсутствии проскальзывания (3.8)

class Option():
    def __init__(self):
        self.print_all = False

def calc_hltetta(data):
    """
    Для расчета объемного содержания жидкости
    :param data: набор всех данных
    :return: ничего, последний расчет - объемное соодержание жидкости с поправкой на угол и поправкой Пэйна
    """

    data.EL = EL(data.liquid_content, data.val_number_Fr,
                 data.flow_regime)  # объемное содержания жидкости в потоке для горизонтальной трубы

    data.Nlv = Nlv(data.vsl_msec, data.rho_liquid_kgm3, data.sigma_kgsec2)

    data.correction_factor_betta = correction_factor_betta(data.liquid_content, data.val_number_Fr,
                                                           data.Nlv, data.flow_regime)

    data.angle_correction_factor = correction_factor_B(data.correction_factor_betta, data.angle_grad)

    data.volume_liquid_content_with_angle = data.EL * data.angle_correction_factor

    data.volume_liquid_content_with_Pains_cor = Pain_cor_for_volume_liquid_content(
        data.volume_liquid_content_with_angle, data.angle_grad)


def calc_grad_BeggsBrill(data, option=None):
    data.Ap = math.pi * data.diametr_inner_m ** 2 / 4  # площадь поперечного сечения трубы, м2

    data.volume_oil_rate_in_condition_m3sec = data.oil_rate_on_surface_m3day * data.oil_formation_volume_factor_m3m3 * 0.9998 / 86400  # (3.1)

    data.volume_liquid_rate_in_condition_m3sec = data.volume_oil_rate_in_condition_m3sec + data.volume_water_rate_in_condition_m3sec

    data.vsl_msec = data.volume_liquid_rate_in_condition_m3sec / data.Ap  # приведенная скорость жидкости (3.10)

    data.volume_gas_rate_in_condition_m3sec = (data.gas_rate_on_surface_m3day -
                                               data.oil_rate_on_surface_m3day * data.Rs_m3m3 -
                                               data.water_rate_on_surface_m3day * data.Rsw_m3m3) * data.gas_formation_volume_factor_m3m3 / 86400  # (3.3)

    data.vsg_msec = data.volume_gas_rate_in_condition_m3sec / data.Ap  # приведенная скорость газа (3.11)

    data.vsm_msec = data.vsl_msec + data.vsg_msec  # приведенная скорость смеси

    data.liquid_content = data.volume_liquid_rate_in_condition_m3sec / (
                data.volume_liquid_rate_in_condition_m3sec + data.volume_gas_rate_in_condition_m3sec)

    data.val_number_Fr = number_Fr(data.vsm_msec, data.diametr_inner_m)

    data.flow_regime = define_flow_regime(data.liquid_content, data.val_number_Fr)

    if data.flow_regime != 3:
        calc_hltetta(data)
    else:
        data.flow_regime = 0
        calc_hltetta(data)
        hltetta_segr = data.volume_liquid_content_with_Pains_cor
        data.flow_regime = 1
        calc_hltetta(data)
        hltetta_inter = data.volume_liquid_content_with_Pains_cor
        L2 = 0.0009252 * data.liquid_content ** (-2.4684)
        L3 = 0.1 * data.liquid_content ** (-1.4516)
        data.volume_liquid_content_with_Pains_cor = EL_for_transition_flow(L2, L3, data.val_number_Fr,
                                                                           hltetta_segr, hltetta_inter)


    data.mu_mix_noslip_pas = viscosity_mixture(data.mu_oil_pasec, data.mu_gas_pasec, data.liquid_content)

    data.rhon_kgm3 = mixture_density(data.rho_liquid_kgm3, data.rho_gas_kgm3, data.liquid_content)

    data.number_Re = number_Re(data.rhon_kgm3, data.vsm_msec, data.diametr_inner_m, data.mu_mix_noslip_pas)

    data.friction_coefficient = friction_coefficient(data.number_Re)

    data.y = data.liquid_content / data.volume_liquid_content_with_Pains_cor ** 2

    data.s = calc_s(data.y)

    data.result_friction = true_friction_coefficient(data.friction_coefficient, data.s)

    data.Ek = Ek(data.vsm_msec, data.vsg_msec, data.rhon_kgm3, data.pressure_pa)

    data.rhos_kgm3 = mixture_density(data.rho_liquid_kgm3, data.rho_gas_kgm3, data.volume_liquid_content_with_Pains_cor)

    data.grad = calc_result(data.result_friction, data.rhon_kgm3, data.vsm_msec,
                            data.diametr_inner_m, data.rhos_kgm3, data.angle_grad, data.Ek)
    if option.print_all:
        out('Ap', data.Ap)
        out('volume_oil_rate_in_condition_m3sec', data.volume_oil_rate_in_condition_m3sec)
        out('volume_liquid_rate_in_condition_m3sec', data.volume_liquid_rate_in_condition_m3sec)
        out('vsl_msec', data.vsl_msec)
        out('volume_gas_rate_in_condition_m3sec', data.volume_gas_rate_in_condition_m3sec)
        out('vsg_msec', data.vsg_msec)
        out('vsm_msec', data.vsm_msec)
        out('liquid_content', data.liquid_content)
        out('val_number_Fr', data.val_number_Fr)
        out('flow_regime', data.flow_regime)
        out('EL', data.EL)
        out('Nlv', data.Nlv)
        out('correction_factor_betta', data.correction_factor_betta)
        out('angle_correction_factor', data.angle_correction_factor)
        out('volume_liquid_content_with_angle', data.volume_liquid_content_with_angle)
        out('volume_liquid_content_with_Pains_cor', data.volume_liquid_content_with_Pains_cor)
        out('mu_mix_noslip_pas', data.mu_mix_noslip_pas)
        out('rhon_kgm3', data.rhon_kgm3)
        out('number_Re', data.number_Re)
        out('friction_coefficient', data.friction_coefficient)
        out('y', data.y)
        out('s', data.s)
        out('result_friction', data.result_friction)
        out('Ek', data.Ek)
        out('rhos_kgm3', data.rhos_kgm3)
        out('grad_barm', data.grad / 10 ** 5)


data_example = data()
option_example = Option()
option_example.print_all = True
calc_grad_BeggsBrill(data_example, option_example)