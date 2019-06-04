import math
from scipy.optimize import fsolve
import numpy as np

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


def fc_var2(p_pa, mt_kgs, rp_sm3sm3, gamma_api, gamma_gas, gg_cm):
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


def number_gr(rho, betta, delta_t, d, mu):
    """
    Рассчет числа Грасгофа
    :param rho: Плотность, кг / м3
    :param betta:  Коэффициент объемного температурного расширения флюида, 1 / С
    :param delta_t: Разница температур, С
    :param d: Диаметр, м
    :param mu: Вязкость, Па * с
    :return: Число Грасгофа, безразмерное
    """
    return rho**2*betta*g*delta_t*d**3/mu**2


def number_pr(cp, mu, k):
    """
    Рассчет числа Прандтля
    :param cp: Теплоемкость,  Дж / кг / С
    :param mu: Вязкость, Па * с
    :param k: Коэффициент теплопроводности, Ватт / м / С
    :return: Число Прандтля, безразмерное
    """
    return cp*mu/k


def number_nu(h, d, k):
    """
    Рассчет числа Нуссельта
    :param h: коэффициент теплоотдачи,  Ватт / м2 / С
    :param d: диаметр, м
    :param k: коэффициент теплопроводности, Ватт / м / С
    :return: Число Нуссельта, безразмерное
    """
    return h*d/k


def number_re(rho, v, d, mu):
    """
    Число Рейнольдса
    :param rho: Плотность, кг / м3
    :param v: Скорость, м / с
    :param d: Диаметр, м
    :param mu:  Вязкость, Па * с
    :return: Число Рейнольдса, безразмерное
    """
    return rho*v*d/mu


def number_nu_corr(n_re, n_pr):
    """
    Корреляция для рассчета числа Нуссельта
    :param n_re: Число Рейнольдса
    :param n_pr: Число Прандтля
    :return: Число Нуссельта
    """
    return 0.023*n_re**0.8*n_pr**0.3


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
    npr = number_pr(cp_an_jkgc, mu_an_pas, k_an_wmc)
    ngr = number_gr(rho_an_kgm3, betta_1c, delta_temp_an_c, (rci_m - rto_m), mu_an_pas)
    han = 0.049 * (ngr * npr) ** (1 / 3) * npr ** 0.074 * k_an_wmc / (rto_m * math.log(rci_m / rto_m))
    han = 0.25 * han
    return han


def hf(ql_m3sec, qg_m3sec, rhol_kgm3, rhog_kgm3,
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
    nren = number_re(rhon, vm, d, mun)
    nprn = number_pr(cpn, mun, kn)
    nnu = number_nu_corr(nren, nprn)
    return nnu*kn/d


def td(time_sec, ke_wmc, rhoe_kgm3, cpe_jkgc, rwb_m):
    """
    Рассчет безразмерного времени
    :param time_sec: время с начала работы скважины, сек
    :param ke_wmc: коэффициент теплопроводности горных пород,  Ватт / м / С
    :param rhoe_kgm3: плотность горных пород, кг / м3
    :param cpe_jkgc: теплоемкость горных пород, Дж / кг / С
    :param rwb_m: радиус скважины, м (внешний радиус цементного кольца
    :return: безразмерное время
    """
    result = ke_wmc*time_sec/(rhoe_kgm3*cpe_jkgc*(math.pow(rwb_m, 2)))
    return result


def tempd(td):
    """
    Безразмерная температура или f(c)
    :param td: безразмерное время
    :return: бехразмерная температура
    """
    if td <= 1.5:
        return 1.1281 * math.sqrt(td) * (1 - 0.3 * math.sqrt(td))
    elif td > 1.5:
        return (0.4063 + 1 / 2 * math.log(td)) * (1 + 0.6 / td)


def temp_fluid_c(rhon, vm, at,
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
    return (tei_c-gg_cm*distance_m)+a*(1-np.exp(-distance_m/a))*(gg_cm-g/cpm+fc)


def uto(hf, han, tempd, rti_m,
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


def temp_diff_an(tr_c, gg_cm, distance_m,
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




def temp_fluid(distance_m, p_pa, delta0=1, data=def_prop):
    """
    Итоговая функция для для расчета температуры.
    На вход может подаваться numpy - матрица
    :param distance_m: расстояние от забоя, м
    :param p_pa:  давление, Па
    :param delta0: начальное приближение перепада температур в затрубном пространестве
                    для расчета функцией fsolve
    :param data: экземляр класса system_properties, со всеми необходимыми данными,
                    по умолчанию данные с примера
    :return: температура в данной точке или numpy матрица КРТ
    """
    data_temp = Save_temp_fluid_c(None)  # экземляр класса для хранения вычисленных данных


    def temp_diff_an_iter(delta):
        ap = pi / 4 * (data.rti_m * 2) ** 2
        vsl = data.ql_m3sec / ap
        vsg = data.qg_m3sec / ap
        vm = vsl + vsg
        llambda = vsl / vm
        rhon = data.rhol_kgm3 * llambda + data.rhog_kgm3 * (1 - llambda)
        mun = data.mul_pas * llambda + data.mug_pas * (1 - llambda)
        kn = data.kl_wmc * llambda + data.kg_wmc * (1 - llambda)
        cpn = ((data.cpl_jkgc * data.rhol_kgm3 * llambda + data.cpg_jkgc *
                data.rhog_kgm3 * (1 - llambda)) / rhon)

        rp = data.qg_m3sec / data.ql_m3sec
        mt = rhon * vm * ap

        val_hf = hf(data.ql_m3sec, data.qg_m3sec, data.rhol_kgm3, data.rhog_kgm3,
                    data.mul_pas, data.mug_pas, data.kl_wmc,
                    data.kg_wmc, data.cpl_jkgc, data.cpg_jkgc,
                    data.rti_m)
        val_han = han_var2(data.mu_an_pas, data.cp_an_jkgc, data.k_an_wmc,
                           delta, data.rho_an_kgm3,
                           data.betta_1c, data.rci_m, data.rto_m)
        val_td = td(data.time_sec, data.ke_wmc, data.rhoe_kgm3,
                    data.cpe_jkgc, data.rwb_m)
        val_tempd = tempd(val_td)
        val_uto = uto(val_hf, val_han, val_tempd,
                      data.rti_m, data.rto_m, data.rco_m,
                      data.rci_m, data.rwb_m, data.ke_wmc,
                      data.kcem_wmc, data.kt_wmc)
        val_fc = fc_var2(p_pa, mt, rp, data.gamma_api, data.gamma_gas, data.gg_cm)
        val_temp_fluid_c = temp_fluid_c(rhon, vm, ap, cpn, val_uto,
                                        data.rto_m, val_fc,
                                        data.tei_c, data.gg_cm,
                                        distance_m)
        data_temp.save(val_temp_fluid_c)  # сохранения в экземляр значений температуры
        val_temp_diff_an = temp_diff_an(data.tei_c, data.gg_cm, distance_m,
                                        data.rto_m, val_uto, val_temp_fluid_c,
                                        data.rti_m, val_han)
        result = val_temp_diff_an - delta
        return result


    delta = fsolve(temp_diff_an_iter, delta0)

    return data_temp.temp_fluid_c


print(3)

# Ниже пример использования
'''sys_prop=system_properties()
depth_m = 2550
pbh_pa=25*10**6
c=pbh_pa/depth_m
tei=sys_prop.tei_c
gg=sys_prop.gg_cm
tff, tfg, p=[], [], []

for i in range(1,depth_m,10):
    p_val=pbh_pa-c*i
    tg=tei-gg*i
    tfg.append(tg)
    p.append(p_val)
l=np.asarray(list(range(1,depth_m,10)))
p=np.asarray(p)
tfg=np.asarray(tfg)
mistake=p*0+1

tff=temp_fluid(l,p,mistake,sys_prop)

print(tff[-1])
l = 2550
p = 10**6

mistake=p*0+1
tff=temp_fluid(l,p,mistake,sys_prop)
print(tff)'''