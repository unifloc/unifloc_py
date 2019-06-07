# -*- coding: utf-8 -*-
"""
Created on Sat Feb  2 12:25:50 2019

Модуль с функциями рассчета есстественной и общей сепарацией

В данный момент реализовано:
    1. Новая методика Маркеза, скопированная с VBA
    2. Тест данной методики и функций (3 функции и 3 теста) 
        не по стандарту тестирования (пока его нет)

Нужно сделать:
    1. Исправить и проверить по отметкам TODO
    2. Общий класс для сепарации
    3. Стандартизированные тесты
    4. Поправить sigma_o/поверхностное натяжение в PVT
@author: oleg kobzar
"""
#импорт модулей
import sys #для подключения других частей unifloc
sys.path.append('../') #для подключения других частей unifloc
import uPVT.PVT as PVT
import math
import numpy as np


'''Расчет, перенесенный с UniflocVBA 7.4
    Calculation from UniflocVBA 7.4
    Рассчет естесственной сепарации по Маркезу, новая методика Маркеза
    Calculation of natural separation from Marques, new Marques method
    Функция рассчитвает градиет давления для неподвижной жидкости в затрубном пространстве ???
    Function calculates pressure gradient for Zero Net Liquid flow in annulus ???'''


class Marquez_separation_VBA():
    def __init__(self):
        self.pintake_atm = 80
        self.wct_perc = 22
        self.tintake_c = 80
        self.dintake_m = 100 / 1000
        self.dcasing_m = 125 / 1000

        self.sepgassep = 0.50

        # Свойства, которые нужно передавать через PVT
        self.gamma_gas = 0.9
        self.gamma_oil = 0.75
        self.gamma_water = 1
        self.rsb_m3m3 = 80
        self.rp_m3m3 = 80
        self.pb_atm = 150
        self.bo_m3m3 = 1.9522964869459403
        self.bg_m3m3 = 0.012673883940723262
        self.rho_oil_stkgm3 = 750.0
        self.rho_gas_sckgm3 = 1.1025
        self.wct_perc = 22
        self.ql_scm3day = 50
        self.surface_tension_method = 0  # 0 - VBA, корреляция Бейкер-Свердлоф, 1 - корреляция Абдул-Джабара

        self.natural_separation = None
        self.total_separation = None

    def __sigma_og_BF__(self, p_atm, t_c, gamma_o, wc_perc=0):
        # calculate surface tension according Baker Sverdloff correlation
        # расчет коэффициента поверхностного натяжения газ-нефть
        # st=serfase tension = поверхностное натяжение
        wc = wc_perc / 100
        t_f = t_c * 1.8 + 32
        p_psia = p_atm / 0.068046
        p_mpa = p_atm / 10
        oil_API = 141.5 / gamma_o - 131.5  # ???
        st68 = 39 - 0.2571 * oil_API
        st100 = 37.5 - 0.2571 * oil_API
        if t_f < 68:
            sto = st68
        else:
            tst = t_f
            if t_f > 100:
                tst = 100
            sto = (68 - (((tst - 68) * (st68 - st100)) / 32)) * (1 - (0.024 * p_psia ** (0.45)))
        # TODO stw, st calc_ST
        stw74 = (75 - (1.108 * p_psia ** (0.349)))
        stw280 = (53 - (0.1048 * p_psia ** (0.637)))
        if t_f < 74:
            stw = stw74
        else:
            tstw = t_f
            if t_f > 280:
                tstw = 280
            stw = stw74 - (((tstw - 74) * (stw74 - stw280)) / 206)
        stw = 10 ** (-(1.19 + 0.01 * p_mpa)) * 1000
        st = (stw * wc) + sto * (1 - wc)
        return st

    def __surface_tension_AM_og_din_cm__(self, t_c, gamma_oil, rs):
        surface_tension_dead_oil = (1.17013 - 1.694 * 10 ** (-3) * (1.8 * t_c + 32)) * (
                    38.085 - 0.259 * (141.5 / gamma_oil - 131.5))
        relative_surface_tension_go_od = (0.056379 + 0.94362 * np.exp(-21.6128 * 10 ** (-3) * rs))
        return surface_tension_dead_oil * relative_surface_tension_go_od

    def calc_natural_separation(self):
        # TODO разобраться и проверить параметры, в каких условиях (помечено ???)
        '''Названия параметров:
            dtub_m - диаметр приемной сетки , м
                    - internal diameter of pump intake, m
            dcas_m - диаметр обсадной колонны, м
                    - internal diameter of casing, m
            qliq_scm3day - дебит жидности в стандартных условиях, ст.м3/сутки ???
                    - liquid rate at standard conditions, m3/day
            qg_scm3day - дебит газа  в стандартных условиях, ст.м3/сутки ???
                    - gas rate at standard conditions, m3/day
            bo_m3m3 - объемный коэффициент нефти при давлении на приеме, м3/м3
                    - oil formation volume factor at reference pressure, m3/m3
            bg_m3m3 - объемный коэффициент газа при давлении на приеме, м3/м3
                    - gas formation volume factor at reference pressure, m3/sm3
            sigma_o - поверхностное натяжение на границе нефть/газ, н/м
                    - oil-gAs surface tension coefficient, Newton/m
                        ***(1 dyne/centimeter = 0.001 newton/meter)
            rho_osc - плотность нефти в стандартных условиях, кг/м3 ???
                    - oil density at standard conditions, kg/m3
            rho_gsc - плотность газа в стандартных условия, кг/м3 ???
                    - gas density at standard conditions, kg/m3
            wct_perc - обводненность, %

            p - давление на приеме, атм
                    - reference pressure, atma

        '''
        dtub_m = self.dintake_m
        dcas_m = self.dcasing_m
        qliq_scm3day = self.ql_scm3day
        qo_scm3day = (self.ql_scm3day*(1-self.wct_perc/100))
        qg_scm3day = qo_scm3day * self.rp_m3m3  # +qgfree_scm3day
        bo_m3m3 = self.bo_m3m3
        bg_m3m3 = self.bg_m3m3
        sigma_o = 200 / 1000  # Заложенная константа в VBA
        if self.surface_tension_method == 0:
            sigma_o = self.__sigma_og_BF__(self.pintake_atm, self.tintake_c, self.gamma_oil, self.wct_perc) / 1000
        if self.surface_tension_method == 1:
            sigma_o = self.__surface_tension_AM_og_din_cm__(self.tintake_c, self.gamma_oil, self.rp_m3m3) / 1000
        rho_osc = self.rho_oil_stkgm3
        rho_gsc = self.rho_gas_sckgm3
        wct_perc = self.wct_perc
        # константы для расчета
        pi = math.pi
        const_g = 9.81

        if qliq_scm3day == 0 or dtub_m == dcas_m:
            return 1
        # water-gas surface tension coefficient (Newton/m)
        sigma_w = 0.01  # ??? TODO - нужно сделать рассчет
        # calculate pressure gradient
        # cnnulus cross-sectional area
        a_p = pi * (dcas_m ** 2 - dtub_m ** 2) / 4
        q_g = bg_m3m3 * qg_scm3day
        q_l = bo_m3m3 * qliq_scm3day * (1 - wct_perc / 100) + qliq_scm3day * wct_perc / 100
        # calculate oil density
        # volume fraction of liquid at no-slip conditions
        lambda_l = q_l / (q_l + q_g)
        # densities
        rho_o = rho_osc / bo_m3m3
        rho_w = 1000  # TODO replace water density
        rho_l = rho_o * (1 - wct_perc / 100) + rho_w * wct_perc / 100
        rho_g = rho_gsc / bg_m3m3
        # no-slip mixture density
        rho_n = rho_l * lambda_l + rho_g * (1 - lambda_l)  # TODO что это?
        # gas sureficial velocity
        V_sg = 0.000011574 * q_g / a_p
        # liquid sureficial velocity
        V_sl = 0.000011574 * q_l / a_p
        # liquid surface tension
        sigma_l = sigma_o * (1 - wct_perc / 100) + sigma_w * wct_perc / 100
        # sureficial velocities
        v_m = V_sl + V_sg
        # Froude number
        n_fr = v_m ** 2 / (const_g * (dcas_m - dtub_m))
        # determine flow pattern
        if n_fr >= (316 * lambda_l ** 0.302) or n_fr >= (0.5 * lambda_l ** (-6.738)):
            flow_pattern = 2
        else:
            if n_fr <= 0.000925 * lambda_l ** -2.468:
                flow_pattern = 0
            else:
                if n_fr <= 0.1 * lambda_l ** -1.452:
                    flow_pattern = 3
                else:
                    flow_pattern = 1
        # calculate terminal gas rise velosity
        if flow_pattern == 0 or flow_pattern == 1:
            v_inf = 1.53 * (const_g * sigma_l * (rho_l - rho_g) / rho_l ** 2) ** 0.25
        else:
            v_inf = 1.41 * (const_g * sigma_l * (rho_l - rho_g) / rho_l ** 2) ** 0.25
        # calculate separation efficienty
        a = -0.0093
        B = 57.758
        c = 34.4
        d = 1.308
        ST = 272
        backst = 1 / 272
        M = V_sl / v_inf
        if M > 13:
            self.natural_separation = 0
            return self.natural_separation
        else:
            self.natural_separation  = ((1 + (a * B + c * M ** d) / (B + M ** d)) ** ST + M ** ST) ** backst - M
            return self.natural_separation

    def calc_total_separation(self):
        self.total_separation = self.natural_separation + (1 - self.natural_separation) * self.sepgassep
        return self.total_separation



