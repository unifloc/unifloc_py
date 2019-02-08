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






'''Расчет, перенесенный с UniflocVBA 7.4
    Calculation from UniflocVBA 7.4
    Рассчет естесственной сепарации по Маркезу, новая методика Маркеза
    Calculation of natural separation from Marques, new Marques method
    Функция рассчитвает градиет давления для неподвижной жидкости в затрубном пространстве ???
    Function calculates pressure gradient for Zero Net Liquid flow in annulus ???'''
        
def unf_calc_natural_separation(dtub_m, dcas_m, qliq_scm3day,
                               qg_scm3day, bo_m3m3, bg_m3m3,
                               sigma_o,
                               rho_osc, rho_gsc, wct_perc, units=1):
    #TODO разобраться и проверить параметры, в каких условиях (помечено ???)
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
        
        units - константа для будущего добавления вывода форматов/корреляций
                - const for future
        p - давление на приеме, атм
                - reference pressure, atma
                    
                        
    '''
    #константы для расчета
    pi = math.pi
    const_g=9.81
    
    if qliq_scm3day == 0 or dtub_m == dcas_m:
        return 1
    #water-gas surface tension coefficient (Newton/m)
    sigma_w=0.01  #??? TODO - нужно сделать рассчет
    #calculate pressure gradient
    #cnnulus cross-sectional area
    a_p = pi * (dcas_m**2-dtub_m**2)/4
    q_g = bg_m3m3 * qg_scm3day
    q_l=bo_m3m3*qliq_scm3day*(1-wct_perc/100) + qliq_scm3day * wct_perc/100
    #calculate oil density
    #volume fraction of liquid at no-slip conditions
    lambda_l=q_l/(q_l+q_g)
    #densities
    rho_o=rho_osc/bo_m3m3
    rho_w=1000 #TODO replace water density
    rho_l=rho_o*(1-wct_perc/100)+rho_w*wct_perc/100
    rho_g=rho_gsc/bg_m3m3
    #no-slip mixture density
    rho_n = rho_l * lambda_l + rho_g * (1 - lambda_l) # TODO что это?
    #gas sureficial velocity
    V_sg = 0.000011574 * q_g / a_p
    #liquid sureficial velocity
    V_sl = 0.000011574 * q_l / a_p
    #liquid surface tension
    sigma_l = sigma_o * (1 - wct_perc / 100) + sigma_w * wct_perc / 100
    #sureficial velocities
    v_m=V_sl+V_sg
    #Froude number
    n_fr = v_m ** 2 / (const_g * (dcas_m - dtub_m))
    #determine flow pattern
    if n_fr >= (316 * lambda_l ** 0.302) or n_fr >= (0.5 * lambda_l **( -6.738)):
        flow_pattern=2
    else:
        if n_fr <= 0.000925 * lambda_l ** -2.468:
            flow_pattern=0
        else:
            if n_fr <= 0.1 * lambda_l ** -1.452:
                flow_pattern=3
            else:
                flow_pattern=1
    #calculate terminal gas rise velosity
    if flow_pattern == 0 or flow_pattern == 1:
        v_inf=1.53 * (const_g * sigma_l * (rho_l - rho_g) / rho_l ** 2) ** 0.25
    else:
        v_inf = 1.41 * (const_g * sigma_l * (rho_l - rho_g) / rho_l ** 2) ** 0.25
    #calculate separation efficienty
    a=-0.0093
    B=57.758
    c=34.4
    d=1.308
    ST=272
    backst=1/272
    M=V_sl/v_inf
    if M>13:
        return 0
    else:
        return ((1 + (a * B + c * M ** d) / (B + M ** d)) ** ST + M ** ST) ** backst - M

'''Дополнительная функция для расчета расхода газа
    Additional function for calculation of gas rate
'''
#TODO разобраться с условиями данной функции
def qgas_scm3day(qo_scm3day,rp_m3m3,qgfree_scm3day=0):
    return qo_scm3day*rp_m3m3+qgfree_scm3day
'''qo_scm3day - дебит нефти в данных условиях, м3/сутки
                - oil rate in this conditions, m3/day
    rp_m3m3 - газосодержание при данных услових, м3/м3
                - gas-oil solution ratio in this conditions, sm3/sm3
    qgfree_scm3day - 
                
'''

'''Дополнительная функция для расчета расхода нефти
    Additional function for calculation of oil rate
'''
#TODO разобраться с условиями данной функции
def qo_scm3day(ql_scm3day,wct_perc):
    return ql_scm3day*(1-wct_perc/100)
'''qo_scm3day - дебит нефти в данных условиях, м3/сутки
                - oil rate in this conditions, m3/day
    ql_scm3day - дебит жидкости в данных условиях, м3/сутки
                - liquid rate in this conditions, m3/day
    wct_perc - обводненность, %
                watercut, %
                
'''
#TODO определиться с размерностью сепарации - доли/безрамерное//м3/м3
def unf_calc_total_separation(natsep, sepgassep):
    return natsep + (1 - natsep) * sepgassep
'''natsep        - естественная сепарация, доли/безразмерное/м3/м3???
    sepgassep     - искусственная сепарация (газосепаратор), доли/безразмерное/м3/м3???
'''
 


'''Тестовые функции, тестирующие как отдельные функции для сепарации
    Последняя функция объединяет в себе все тесты
    Сравнивается факт с должным значением, 
    посчитанным заранее для данных значений параметров
    При импорте/запуске модуля separation общий тест запустится автоматически
    Вывод:
        название функции + работает/неработает
'''
work_right=' -work right'
dont_work=' -work wrong - ERROR!!!'


def unf_calc_natural_separation_test():
    '''copy_test_data=(100/1000,125/1000,
               50,
               3120.0,1.9522964869459403,
               0.012673883940723262,
               130/1000,750.0,
               1.1025,22)'''
     
    fact=unf_calc_natural_separation(100/1000,125/1000,
                                       50,
                                       3120.0,1.9522964869459403,
                                       0.012673883940723262,
                                       130/1000,750.0,
                                       1.1025,22)
    if fact==0.6470225555424021:
        return print('unf_calc_natural_separation' + work_right)
    else:
        return print('unf_calc_natural_separation' + dont_work)

def qgas_scm3day_test():
    '''copy_test_data=(10,1.2)'''
    fact=qgas_scm3day(10,1.2)
    if fact==12.0:
        return print('qgas_scm3day' + work_right)
    else:
        return print('qgas_scm3day' + dont_work)
    
def qo_scm3day_test():
    '''copy_test_data=(10,22)'''
    fact=qo_scm3day(10,22)
    if fact==7.800000000000001:
        return print('qo_scm3day' + work_right)
    else:
        return print('qo_scm3day' + dont_work)

def unf_calc_total_separation_test():
    '''copy_test_data=(0.35,0.50)'''
    fact=unf_calc_total_separation(0.35,0.50)
    if fact==0.675:
        return print('unf_calc_total_separation' + work_right)
    else:
        return print('unf_calc_total_separation' + dont_work)
    
def separation_test():
    print('Result of separation check:')
    qgas_scm3day_test()
    qo_scm3day_test()
    unf_calc_natural_separation_test()
    unf_calc_total_separation_test()

separation_test()






