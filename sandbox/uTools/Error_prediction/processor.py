"""
Модуль для массового расчета скважин, оснащенных УЭЦН, используя расчетное ядро UniflocVBA
Кобзарь О.С Хабибуллин Р.А. 21.08.2019
"""
# TODO отрефакторить
# TODO сразу ошибка для определения дебита на фактических точках
# TODO интеграл
# TODO поверхность решения
# TODO точность
# TODO use COBYLA, разобраться с методами
# TODO изменение функции ошибки (деление на  добавление линейного давления, добавления штуцера)
# TODO сохранять параметры расчета
import sys
import os
import time
import numpy as np
from scipy.optimize import minimize
import pandas as pd
import xlwings as xw
import Error_prediction.processor_utils as proc_ut
from Error_prediction.CalcOptions import CalcOptions
from Error_prediction.ESP_DATA import AllEspData
from Error_prediction.TrDataGetter import read_tr_and_get_data

sys.path.append("../../../")
import unifloc_vba.description_generated.python_api as python_api

time_mark = ''


def straight_calc(unifloc_vba, this_state):
    """
    Функция для прямого расчета скважины от приема ЭЦН
    :param unifloc_vba: API для вызова функций
    :param this_state: класс-состояние со всеми параметрами скважины
    :return: result - результат расчета в форме списка
    """
    pvt_str = unifloc_vba.calc_PVT_encode_string(this_state.gamma_gas, this_state.gamma_oil,
                                                 this_state.gamma_wat, this_state.rsb_m3m3, this_state.rp_m3m3,
                                                 this_state.pb_atm, this_state.tres_c,
                                                 this_state.bob_m3m3, this_state.muob_cp,
                                                 ksep_fr=this_state.ksep_d, pksep_atma=this_state.psep_atm,
                                                 tksep_C=this_state.tsep_c)
    well_str = unifloc_vba.calc_well_encode_string(this_state.h_perf_m,
                                                   this_state.h_pump_m,
                                                   this_state.udl_m,
                                                   this_state.dcas_mm,
                                                   this_state.d_tube_mm,
                                                   this_state.d_choke_mm,
                                                   tbh_C=this_state.tsep_c)
    esp_str = unifloc_vba.calc_ESP_encode_string(this_state.esp_id,
                                                 this_state.ESP_head_nom,
                                                 this_state.ESP_freq,
                                                 this_state.u_motor_data_v,
                                                 this_state.power_motor_nom_kwt,
                                                 this_state.tsep_c,
                                                 t_dis_C=-1,
                                                 KsepGS_fr=this_state.KsepGS_fr,
                                                 ESP_Hmes_m=this_state.h_tube_m,
                                                 c_calibr_head=this_state.c_calibr_head_d,
                                                 c_calibr_rate=this_state.c_calibr_rate_d,
                                                 c_calibr_power=this_state.c_calibr_power_d,
                                                 cos_phi=this_state.cos_phi_data_d)
    result = unifloc_vba.calc_well_plin_pwf_atma(this_state.qliq_m3day, this_state.watercut_perc,
                                                 this_state.p_wf_atm,
                                                 this_state.p_cas_data_atm, well_str,
                                                 pvt_str, esp_str, this_state.hydr_corr,
                                                 this_state.ksep_d, this_state.c_calibr_head_d,
                                                 this_state.c_calibr_power_d,
                                                 this_state.c_calibr_rate_d)
    return result


def calc_well_plin_pwf_atma_for_fsolve(minimaze_parameters, options, state, unifloc_vba):
    """
    Фунция один раз рассчитывает модель скважины в UniflocVBA.
    Передается в оптимизатор scipy.minimaze
    :param unifloc_vba:
    :param state:
    :param options:
    :param minimaze_parameters: список подбираемых параметров - калибровки или расходы фаз
    :return: значение функции ошибки
    """
    # if not options.vfm_calc_option
    # if options.restore_q_liq_only:
    state.qliq_m3day = minimaze_parameters[0]
    if options.debug_mode:
        print('qliq_m3day = ' + str(state.qliq_m3day))
    # прямой расчет
    result = straight_calc(unifloc_vba, state)
    # сохранение результата в форме списка в структуру для последующего извлечения
    state.result = result
    p_line_calc_atm = result[0][0]
    p_buf_calc_atm = result[0][2]
    power_cs_calc_w = result[0][16]
    if options.use_pwh_in_loss:  # функция ошибки
        result_for_folve = options.hydr_part_weight_in_error_coeff * \
                           ((
                                    p_line_calc_atm - state.p_wellhead_data_atm)
                            / state.p_wellhead_data_max_atm) ** 2 + (1 - options.hydr_part_weight_in_error_coeff) * (
                                   (power_cs_calc_w - state.active_power_cs_data_kwt) /
                                   state.active_power_cs_data_max_kwt) ** 2
    else:
        result_for_folve = options.hydr_part_weight_in_error_coeff * \
                           ((p_buf_calc_atm - state.p_buf_data_atm) / state.p_buf_data_max_atm) ** 2 + \
                           (1 - options.hydr_part_weight_in_error_coeff) * (
                                   (power_cs_calc_w - state.active_power_cs_data_kwt) /
                                   state.active_power_cs_data_max_kwt) ** 2

    if options.debug_mode:
        print(f'Линейное давление в модели = {p_line_calc_atm}')
        print(f'Буферное давление в модели = {p_buf_calc_atm}')
        print(f'Мощность в модели = {power_cs_calc_w}' + str())
        print(f'ошибка на текущем шаге = {result_for_folve}')
    state.error_in_step = result_for_folve
    return result_for_folve


def mass_calculation(state, options, unifloc_vba):
    """
    Функция для массового расчета - модель скважины UniflocVBA + оптимизатор scipy
    :param unifloc_vba:
    :param state: структура со всеми необходимыми данными модели
    :param options:
    :return: результат оптимизационной задачи - параметры скважины - для определенного набора данных
    """

    # if options.vfm_calc_option
    # if options.restore_q_liq_only:
    print(np.array([state.qliq_m3day]))
    result = minimize(calc_well_plin_pwf_atma_for_fsolve, np.array(state.qliq_m3day),
                      args=(options, state, unifloc_vba), method='SLSQP', bounds=[[0, None]]
                      )
    # TODO разобраться с левой границей

    print(result)
    true_result = state.result  # сохранение результатов расчета оптимизированной модели
    return true_result


def calc(options=CalcOptions()):
    """
    Основная расчетная функция, в которой есть все
    :param options: структура со всеми надстройками, данными, параметрами
    :return: None
    """
    unifloc_vba = python_api.API(f'../../../unifloc_vba/{options.addin_name}')
    calc_mark_str = str(options.number_of_thread)

    # создание директорий для результатов расчета
    # if not options.vfm_calc_option:
    input_path = f'data/{options.well}/{options.dir_name_with_input_data}/{options.well}_restore_input'
    dir_to_save_calculated_data = os.getcwd() + '\\data\\' + options.well + '\\' + 'restore_' + time_mark
    try:
        os.mkdir(dir_to_save_calculated_data)
    except:
        pass
    if options.multiprocessing:
        dir_to_save_calculated_data += '\\' + 'multiprocessing__err_prediction'
        try:
            os.mkdir(dir_to_save_calculated_data)
        except:
            pass
    # основной цикл расчета начинается здесь
    # if options.calc_option:
    # чтение входных данных
    prepared_data = pd.read_csv(input_path + ".csv")
    prepared_data = proc_ut.divide_prepared_data(prepared_data, options)
    prepared_data.index = pd.to_datetime(prepared_data["Время"])
    del prepared_data["Время"]

    result_dataframe = {'d': [2]}
    result_dataframe = pd.DataFrame(result_dataframe)
    start_time = time.time()
    # прочитаем техрежим и извлечем данным
    tr_file_full_path = os.getcwd() + '\\data\\tr\\' + options.tr_name
    tr_data = read_tr_and_get_data(tr_file_full_path, options.well)
    this_state = AllEspData(unifloc_vba, tr_data)
    # сделаем начальное приближение
    init_guess_q3 = prepared_data['Объемный дебит жидкости (СУ)'].values.mean()
    this_state.qliq_m3day = init_guess_q3

    this_state.active_power_cs_data_max_kwt = prepared_data['Активная мощность (СУ)'].max() * 1000
    this_state.p_buf_data_max_atm = prepared_data['Рбуф (Ш)'].max()
    this_state.p_wellhead_data_max_atm = prepared_data['Линейное давление (СУ)'].max() * 10
    this_state.qliq_max_m3day = prepared_data['Объемный дебит жидкости (СУ)'].max()
    # начало итерации по строкам - наборам данных для определенного времени
    for i in range(prepared_data.shape[0]):
        check = i % options.amount_iters_before_restart
        # защита против подвисаний экселя - не работает в многопотоке
        if check == 0 and i != 0:
            if options.debug_mode:
                print('Перезапуск Excel и VBA')
            unifloc_vba.book.close()
            time.sleep(options.sleep_time_sec)
            unifloc_vba.book = xw.Book(unifloc_vba + options.addin_name)
        start_in_loop_time = time.time()
        row_in_prepared_data = prepared_data.iloc[i]
        if options.debug_mode:
            print("Расчет для времени: " + str(prepared_data.index[i]))
            print('Итерация № ' + str(i) + ' из ' + str(prepared_data.shape[0]) +
                  ' в потоке №' + str(options.number_of_thread))
        this_state = proc_ut.transfer_data_from_row_to_state(this_state, row_in_prepared_data,
                                                             options.vfm_calc_option)
        # расчет
        this_result = mass_calculation(this_state, options, unifloc_vba)
        end_in_loop_time = time.time()

        if options.debug_mode:
            print("Затрачено времени в итерации: " + str(i) + " - " + str(end_in_loop_time - start_in_loop_time))

        new_dataframe = proc_ut.create_new_result_df(this_result, this_state, prepared_data, i)
        result_dataframe = result_dataframe.append(new_dataframe, sort=False)

        # if options.vfm_calc_option:
        result_dataframe.to_csv(dir_to_save_calculated_data + '\\' + options.well + "_restore_" +
                                calc_mark_str + ".csv")

    end_time = time.time()
    print("Затрачено всего: " + str(end_time - start_time))
    close_f = unifloc_vba.book.macro('close_book_by_macro')
    close_f()
