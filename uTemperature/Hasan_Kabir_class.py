import math
from scipy.optimize import fsolve
import numpy as np

const_g_m2sec = 9.81
pi = math.pi





def __fc_var2__(p_pa, mt_kgs, rp_sm3sm3, gamma_api, gamma_gas, gg_cm):
    """
    Корреляция Sagar et al. (1991) для расчетра Fc
    физически - совокупность влияния эффекта Джоуля-Томпсона и
    изменения кинетической энергии на теплоперенос
    p_pa - давление, Па
    mt_kgs - массовый расход, кг/с
    rp_sm3sm3 - газовый фактор, м3/м3 ???
    gamma_api - плотность нефти, АPI
    gamma_gas - относительная плотность газа (по воздуху???)
    gg_cm - геотермический градиент, градус Цельсия на м 
	out -  градус Цельсия на м 
	Расчет в несистемных размерностях, выход в СИ
	Пример:
	import uTemperature.uTemp as temp
	sys_prop=temp.system_properties()
	result=temp.fc_var2(sys_prop.p_pa,sys_prop.mt_kgs,sys_prop.rp_sm3sm3,
           sys_prop.gamma_api,sys_prop.gamma_gas,sys_prop.gg_cm)
	"""
    # Перевод размерностей
    p_psi = p_pa / 6894.757293178
    gg_fft = gg_cm / 1.82268883056
    rp_scfstb = rp_sm3sm3 / 0.17810760667903497
    mt_lbms = mt_kgs * 2.2046226218487757

    result_fft = ((-2.978) * 10 ** (-3) + 1.006 * 10 ** (-6) * p_psi + 1.906 * 10 ** (-4) * mt_lbms - 1.047 * 10 ** (
        -6) * rp_scfstb +
                  3.229 * 10 ** (-5) * gamma_api + 4.009 * 10 ** (-3) * gamma_gas - 0.3551 * gg_fft)
    return result_fft * 1.82268883056




def __number_pr__(cp, mu, k):
    """
    Рассчет числа Прандтля
    :param cp: Теплоемкость,  Дж / кг / С
    :param mu: Вязкость, Па * с
    :param k: Коэффициент теплопроводности, Ватт / м / С
    :return: Число Прандтля, безразмерное
    """
    return cp*mu/k


def han_var2(mu_an_pas, cp_an_jkgc, k_an_wmc, delta_temp_an_c, rho_an_kgm3, betta_1c, rci_m, rto_m):
    """
    Функция для расчета коэффициента теплоотдачи при есстественной конвекции в затрубном пространстве
    :param mu_an_pas: Вязкость флюида в затрубном пространстве, Па * с
    :param cp_an_jkgc: Теплоекость флюида в затрубном пространстве, Дж / кг / С
    :param k_an_wmc: Коэффициент теплопроводности в затрубном пространстве, Ватт / м / С
    :param delta_temp_an_c: Разница температур в затрубном пространстве, С
    :param rho_an_kgm3: Плотность флюида в затрубном пространстве, кг / м3
    :param betta_1c: Коэффициент объемного температурного расширения флюида в з.п., 1 / С
    :param rci_m: Внутренний диаметр обсадной колонны, м
    :param rto_m: Внешний диаметр НКТ
    :return: Коэффициент теплоотдачи, Ватт / м2 / С
    Пример:
    example_han_var2=temp.han_var2(sys_prop.mu_an_pas, sys_prop.cp_an_jkgc,
                              sys_prop.k_an_wmc, sys_prop.delta_temp_an_c,
                              sys_prop.rho_an_km3, sys_prop.betta_1c,
                              sys_prop.rci_m, sys_prop.rto_m)
    """
    npr = __number_pr__(cp_an_jkgc, mu_an_pas, k_an_wmc)
    ngr = rho_an_kgm3 ** 2 * betta_1c * const_g_m2sec * delta_temp_an_c * (rci_m - rto_m) ** 3 / mu_an_pas ** 2
    han = 0.049 * (ngr * npr) ** (1 / 3) * npr ** 0.074 * k_an_wmc / (rto_m * math.log(rci_m / rto_m))
    han = 0.25 * han
    return han


def __hf__(ql_m3sec, qg_m3sec, rhol_kgm3, rhog_kgm3,
           mul_pas, mug_pas, kl_wmc, kg_wmc,
           cpl_jkgc, cpg_jkgc, rti_m):
    """
    Рассчет коэффициента конвективной теплоотдачи в НКТ
    :param ql_m3sec: расход жидкости, м3 / сек
    :param qg_m3sec: расход газа, м3 / сек
    :param rhol_kgm3: плотность жидкости, кг / м3
    :param rhog_kgm3: плотность газа, кг / м3
    :param mul_pas: вязкость жидкости, Па * с
    :param mug_pas: вязкость газа, Па * с
    :param kl_wmc: коэффициент теплопроводности жидкости, Ватт  / м / С
    :param kg_wmc: коэффициент теплопроводности газа,  Ватт  / м / С
    :param cpl_jkgc: теплоемкость жидкости, Дж  / кг / с
    :param cpg_jkgc: теплоемкость газа, Дж  / кг / с
    :param rti_m: внутренний диаметр НКТ, м
    :return: Коэффициент конвективной теплоотдачи, Ватт / м2 / С
    """
    d = rti_m * 2
    ap = pi/4*d**2
    vsl = ql_m3sec/ap
    vsg = qg_m3sec/ap
    vm = vsl+vsg
    llambda = vsl/vm
    rhon = rhol_kgm3*llambda+rhog_kgm3*(1-llambda)
    mun = mul_pas*llambda+mug_pas*(1-llambda)
    kn = kl_wmc*llambda+kg_wmc*(1-llambda)
    cpn = (cpl_jkgc*rhol_kgm3*llambda+cpg_jkgc*rhog_kgm3*(1-llambda))/rhon
    nren = rhon * vm * d / mun
    nprn = __number_pr__(cpn, mun, kn)
    nnu = 0.023 * nren**0.8 * nprn ** 0.3
    return nnu*kn/d


def __temp_fluid_c__(rhon, vm, at,
                     cpm, u, rto_m,
                     fc, tei_c, gg_cm, distance_m):
    """
    Расчет температуры флюида в НКТ
    :param rhon: средняя плотность флюида, кг / м3
    :param vm: средняя скорость флюида без учета скольжения газа, м / с , (vsl+vsg)
    :param at: внутреннее сечение НКТ, м2
    :param cpm: средняя теплоемкость флюида,  Дж / кг / С
    :param u: общий коэффициент теплопередачи, Ватт / м2 / м
    :param rto_m: внешний диаметр НКТ, м
    :param fc: безразмерная функция
    :param tei_c: начальная температура окружающей среды, пласта, на забое, С
    :param gg_cm: геотермический градиент, С / м
    :param distance_m: расстояние от забоя до точки исследования, м
    :return: Температура флюида в НКТ, С
    """
    dto_m = rto_m*2
    mt = rhon*vm*at
    a = mt*cpm/pi/dto_m/u
    return (tei_c-gg_cm*distance_m)+a*(1-np.exp(-distance_m/a))*(gg_cm - const_g_m2sec / cpm + fc)


def __uto__(hf, han, tempd, rti_m,
            rto_m, rco_m, rci_m,
            rwb_m, ke_wmc, kcem_wmc, kt_wmc):
    """
    Общий коэффициент теплопередачи, состоящий из нескольких частей
    :param hf: конвективный коэффициет теплоотдачи в НКТ, Ватт / м2 / С
    :param han: конвективный коэффициет теплоотдачи в затрубном пространстве, Ватт / м2 / С
    :param tempd: безразмерная температура или f(t)
    :param rti_m: внутренний диаметр НКТ, м
    :param rto_m: внешний радиус НКТ, м
    :param rco_m: внешний радиус обсадной колонны, м
    :param rci_m: внутренний радиус обсадной колонны, м
    :param rwb_m: внешний радиус скважины (внешний радиус цементного кольца), м
    :param ke_wmc: коэффициент теплопроводности ГП, Ватт / м / С
    :param kcem_wmc: коэффициент цементного кольца, Ватт / м / С
    :param kt_wmc: коэффициент теплопроводности металла труб (НКТ и ОК), Ватт / м / С
    :return: Общий коэффициент теплопередачи, Ватт / м2 / С
    """
    first_part = 1/rti_m/hf   # теплоотдача от ж к внутренней стенки НКТ
    second_part = math.log(rto_m/rti_m)/kt_wmc/2    # убрать 2- ошибка #теплопередача через стенку НКТ
    third_part = 1/rci_m/han    # конвекция через затруб
    fourth_part = math.log(rco_m/rci_m)/kt_wmc/2     # убрать 2- ошибка #теплопередача через стенку ОК
    fifth_part = math.log(rwb_m/rco_m)/kcem_wmc    # теплопередача через цементное кольцо
    sixth_part = tempd/ke_wmc    # теплопередача через цементное кольцо

    return 1/(first_part+second_part+third_part+fourth_part+fifth_part+sixth_part)/rto_m


def __temp_diff_an__(tr_c, gg_cm, distance_m,
                     rto_m, u, tf_c,
                     rti_m, han):
    """
    Разница температур в затрубном пространстве
    :param tr_c: температура пласта на забое, С
    :param gg_cm: геотермический градиент, С / м
    :param distance_m: расстояние от забоя, м
    :param rto_m: внешний радиус НКТ, м
    :param u: общий коэффициент теплопередачи, Ватт / м2 / С
    :param tf_c: температура флюида в НКТ, С
    :param rti_m: внутренний радиус НКТ, м
    :param han: коэффициент конвективной теплоотдачи в затрубном пространстве, Ватт / м2 / С
    :return: Разница температур в затрубном пространстве, С
    """
    tg_c = tr_c-distance_m*gg_cm  # температура ГП на distance_m от забоя
    q = 2*pi*rto_m*u*(tf_c-tg_c)  # тепловой поток
    temp_diff_an = q/(2*pi*rti_m*han)
    return temp_diff_an


class Hasan_Kabir_cor():
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
        self.distance_m = 1000  # расстояние от забоя в точке расчета
        self.gg_cm = 0.027
        self.gamma_gas = 0.65
        self.gamma_api = 29

        self.p_pa = 792.9 * 10 ** 3  # давление в точке расчета

        self.delta = None
        self.ap_m2 = None
        self.vsl_msec = None
        self.vsg_msec = None
        self.vs = None
        self.noslip_liquid_content = None
        self.rhon_kgm3 = None
        self.mun = None
        self.kn = None
        self.cpn = None
        self.rp = None
        self.mt = None
        self.val_hf = None
        self.val_han = None
        self.val_td = None
        self.val_tempd = None
        self.val_uto = None
        self.val_fc = None
        self.val_temp_fluid_c = None
        self.val_temp_diff_an = None






    def __temp_diff_an_iter__(self, delta):
        """
        При итерациях вычисляется температура движущегося флюида
        :param delta: начальное приближение перепада температур в затрубном пространестве
                    для расчета функцией fsolve
        :return: разница между вычисленным delta и delta, подающемся на вход
        """
        self.delta = delta
        self.ap_m2 = pi / 4 * (self.rti_m * 2) ** 2
        self.vsl_msec = self.ql_m3sec / self.ap_m2
        self.vsg_msec = self.qg_m3sec / self.ap_m2
        self.vm_msec = self.vsl_msec + self.vsg_msec
        self.noslip_liquid_content = self.vsl_msec / self.vm_msec  # TODO это точно содержание жидкости, и можно его так определять
        self.rhon_kgm3 = self.rhol_kgm3 * self.noslip_liquid_content + self.rhog_kgm3 * (1 - self.noslip_liquid_content)
        self.mun = self.mul_pas * self.noslip_liquid_content + self.mug_pas * (1 - self.noslip_liquid_content)
        self.kn = self.kl_wmc * self.noslip_liquid_content + self.kg_wmc * (1 - self.noslip_liquid_content)
        self.cpn = ((self.cpl_jkgc * self.rhol_kgm3 * self.noslip_liquid_content + self.cpg_jkgc *
                     self.rhog_kgm3 * (1 - self.noslip_liquid_content)) / self.rhon_kgm3)

        self.rp = self.qg_m3sec / self.ql_m3sec
        self.mt = self.rhon_kgm3 * self.vm_msec * self.ap_m2

        self.val_hf = __hf__(self.ql_m3sec, self.qg_m3sec, self.rhol_kgm3, self.rhog_kgm3, self.mul_pas, self.mug_pas,
                             self.kl_wmc, self.kg_wmc, self.cpl_jkgc, self.cpg_jkgc, self.rti_m)
        self.val_han = float(han_var2(self.mu_an_pas, self.cp_an_jkgc, self.k_an_wmc,
                                self.delta, self.rho_an_kgm3,
                                self.betta_1c, self.rci_m, self.rto_m))  #TODO для борьбы с fsolve
        self.val_td = self.ke_wmc * self.time_sec / (self.rhoe_kgm3 * self.cpe_jkgc*(math.pow(self.rwb_m, 2)))
        if self.val_td <= 1.5:
            self.val_tempd = 1.1281 * math.sqrt(self.val_td) * (1 - 0.3 * math.sqrt(self.val_td))
        elif self.val_td > 1.5:
            self.val_tempd =  (0.4063 + 1 / 2 * math.log(self.val_td)) * (1 + 0.6 / self.val_td)
        self.val_uto = __uto__(self.val_hf, self.val_han, self.val_tempd, self.rti_m, self.rto_m, self.rco_m,
                               self.rci_m, self.rwb_m, self.ke_wmc, self.kcem_wmc, self.kt_wmc)
        self.val_fc = __fc_var2__(self.p_pa, self.mt, self.rp, self.gamma_api, self.gamma_gas, self.gg_cm)
        self.val_temp_fluid_c = __temp_fluid_c__(self.rhon_kgm3, self.vm_msec, self.ap_m2, self.cpn, self.val_uto,
                                                 self.rto_m, self.val_fc, self.tei_c, self.gg_cm, self.distance_m)
        self.val_temp_diff_an = __temp_diff_an__(self.tei_c, self.gg_cm, self.distance_m, self.rto_m, self.val_uto,
                                                 self.val_temp_fluid_c, self.rti_m, self.val_han)
        result = self.val_temp_diff_an - self.delta
        return float(result)  #TODO для борьбы с fsolve

    def calc_t_c_fluid(self, distance_m, p_pa):
        """
        Метод расчета температуры флюида
        :param distance_m: расстояние от забоя до точки расчета
        :param p_pa: давление в точке расчета
        :return: температура в точке расчета
        """
        self.distance_m = distance_m
        self.p_pa = p_pa
        delta = fsolve(self.__temp_diff_an_iter__, 1)
        return float(self.val_temp_fluid_c)  # TODO можно ли так делать, всегда единственное решение?

# пример использования класса


depth_m = 2550
pbh_pa = 25*10**6
l = 2550
p = 10**6

temp_cor = Hasan_Kabir_cor()
t_wellhead_c= temp_cor.calc_t_c_fluid(l, p)
print(t_wellhead_c)

