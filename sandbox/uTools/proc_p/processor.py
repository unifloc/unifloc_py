"""
Модуль для массового расчета скважин, оснащенных УЭЦН, используя расчетное ядро UniflocVBA

Кобзарь О.С Хабибуллин Р.А. 21.08.2019
"""
# TODO отрефакторить
# TODO интеграл
# TODO поверхность решения
# TODO изменение функции ошибки (деление на  добавление линейного давления, добавления штуцера)
# TODO сохранять параметры расчета
import sys
import os
import shutil
sys.path.append('../' * 4)
sys.path.append('../' * 5)
current_path = os.getcwd()
path_to_sys = current_path.replace(r'unifloc\sandbox\uTools\proc_p', '')
path_to_sys = current_path.replace(r'unifloc\sandbox\uTools', '')
sys.path.append(path_to_sys)  # добавляем путь в sys, чтобы нашелся проект unifloc_vba
current_path = current_path.replace(r'unifloc\sandbox\uTools\proc_p', r'unifloc_vba\\')
current_path = current_path.replace(r'unifloc\sandbox\uTools', r'unifloc_vba\\')
print(current_path)
import unifloc_vba.description_generated.python_api as python_api
from scipy.optimize import minimize
import pandas as pd
import xlwings as xw
sys.path.append("../")
import datetime
import time
from multiprocessing import Pool
import unifloc.sandbox.uTools.preproc_p.workflow_tr_data as workflow_tr_data
import unifloc.sandbox.uTools.proc_p.well_calculation as well_calculation
import unifloc.sandbox.uTools.proc_p.workflow_input_data as workflow_input_data
import unifloc.sandbox.uTools.proc_p.proc_tool as proc_tool


time_mark = ''  # datetime.datetime.today().strftime('%Y_%m_%d_%H_%M')  # временная метка для сохранения без перезаписи


def calc(options=well_calculation.Calc_options()):
    """
    Основная расчетная функция, в которой есть все
    :param options: структура со всеми надстройками, данными, параметрами
    :return: None
    """
    opt = options
    UniflocVBA = python_api.API(current_path + options.addin_name)

    app_path = os.getcwd().replace(r'proc_p', '')
    static_data = workflow_tr_data.Static_data()
    static_data_df = pd.read_excel(opt.static_data_full_path)
    static_data = workflow_tr_data.fill_static_data_structure_by_df(static_data, static_data_df, opt.well_name + " (ready)")

    input_data_filename_str, dir_to_save_calculated_data = \
        proc_tool.create_directories(opt.vfm_calc_option, app_path, opt.well_name, options, opt.dir_name_with_input_data, time_mark)

    def mass_calculation(this_state: workflow_input_data.all_ESP_data, debug_print = False, restore_flow=False, restore_q_liq_only = True):
        """
        Функция для массового расчета - модель скважины UniflocVBA + оптимизатор scipy
        :param this_state: структура со всеми необходимыми данными модели
        :param debug_print: флаг для вывода разных параметров для контроля состояния
        :param restore_flow: флаг для восстановления дебитов, False - адаптация
        :param restore_q_liq_only: флаг для метода восстновления дебитов
        :return: результат оптимизационной задачи - параметры скважины - для определенного набора данных
        """
        def calc_well_plin_pwf_atma_for_fsolve(minimaze_parameters):
            """
            Фунция один раз рассчитывает модель скважины в UniflocVBA.
            Передается в оптимизатор scipy.minimaze
            :param minimaze_parameters: список подбираемых параметров - калибровки или расходы фаз
            :return: значение функции ошибки
            """
            if restore_flow == False: # определение и сохранение подбираемых параметров
                this_state.c_calibr_power_d = minimaze_parameters[1]
                this_state.c_calibr_head_d = minimaze_parameters[0]
                this_state.c_calibr_rate_d = this_state.c_calibr_rate_d
                if debug_print:
                    print('c_calibr_power_d = ' + str(this_state.c_calibr_power_d))
                    print('c_calibr_head_d = ' + str(this_state.c_calibr_head_d))
            else:
                if restore_q_liq_only == True:
                    this_state.qliq_m3day = minimaze_parameters[0]
                    if debug_print:
                        print('qliq_m3day = ' + str(this_state.qliq_m3day))
                else:
                    this_state.qliq_m3day = minimaze_parameters[0]
                    this_state.watercut_perc = minimaze_parameters[1]
                    if debug_print:
                        print('qliq_m3day = ' + str(this_state.qliq_m3day))
                        print('watercut_perc = ' + str(this_state.watercut_perc))

            result = well_calculation.straight_calc(UniflocVBA, this_state)  # прямой расчет

            this_state.result = result  # сохранение результата в форме списка в структуру для последующего извлечения
            p_line_calc_atm = result[0][0]
            p_buf_calc_atm = result[0][2]
            power_CS_calc_W = result[0][16]
            if options.use_pwh_in_loss == True: # функция ошибки
                result_for_folve = opt.hydr_part_weight_in_error_coeff * \
                                   ((p_line_calc_atm - this_state.p_wellhead_data_atm) / this_state.p_wellhead_data_max_atm) ** 2 + \
                                   (1 - opt.hydr_part_weight_in_error_coeff) * ((power_CS_calc_W - this_state.active_power_cs_data_kwt) /
                                    this_state.active_power_cs_data_max_kwt) ** 2
            else:
                result_for_folve = opt.hydr_part_weight_in_error_coeff * \
                                   ((p_buf_calc_atm - this_state.p_buf_data_atm) / this_state.p_buf_data_max_atm) ** 2 + \
                                   (1 - opt.hydr_part_weight_in_error_coeff) * ((power_CS_calc_W - this_state.active_power_cs_data_kwt) /
                                    this_state.active_power_cs_data_max_kwt) ** 2

            if debug_print:
                print("Линейное давление в модели = " + str(p_line_calc_atm))
                print("Буферное давление в модели = " + str(p_buf_calc_atm))
                print("Мощность в модели = " + str(power_CS_calc_W))
                print("ошибка на текущем шаге = " + str(result_for_folve))
            this_state.error_in_step = result_for_folve
            return result_for_folve

        if restore_flow == False: # выполнение оптимизации модели скважины с текущим набором данных
            result = minimize(calc_well_plin_pwf_atma_for_fsolve, [this_state.c_calibr_head_d, this_state.c_calibr_power_d], method='SLSQP', tol= 1e-07,
                              bounds=[[this_state.c_calibr_head_d_min_limit, this_state.c_calibr_head_d_max_limit],
                                      [this_state.c_calibr_power_d_min_limit, this_state.c_calibr_power_d_max_limit]],
                              options={'maxiter': 50, 'ftol': 1e-07})
        else:
            if restore_q_liq_only == True:
                result = minimize(calc_well_plin_pwf_atma_for_fsolve, [this_state.qliq_m3day], bounds=[[20,  100]], options={'maxiter': 100, 'ftol': 1e-07})  #TODO разобраться с левой границей
            else:
                result = minimize(calc_well_plin_pwf_atma_for_fsolve, [100, 20], bounds=[[5, 175], [10, 35]], options={'maxiter': 50, 'ftol': 1e-04})
        print(result)
        true_result = this_state.result # сохранение результатов расчета оптимизированной модели
        return true_result

    prepared_data = pd.read_csv(input_data_filename_str + ".csv")  # чтение входных данных
    if opt.number_of_thread > prepared_data.shape[0]:
        print(f'Лишний поток {opt.number_of_thread} для входных данных с количеством строк {prepared_data.shape[0]}')
        opt.calc_option = False
    if opt.calc_option:  # основной цикл расчета начинается здесь

        prepared_data = workflow_input_data.divide_prepared_data(prepared_data, options)

        prepared_data.index = pd.to_datetime(prepared_data["Время"])
        del prepared_data["Время"]

        result_dataframe = {'d': [2]}
        result_dataframe = pd.DataFrame(result_dataframe)
        start_time = time.time()
        this_state = workflow_input_data.all_ESP_data(UniflocVBA, static_data)
        this_state.active_power_cs_data_max_kwt = prepared_data['Активная мощность (СУ)'].max() * 1000
        this_state.p_buf_data_max_atm = prepared_data['Рбуф (Ш)'].max()
        #this_state.p_buf_data_max_atm = prepared_data['Рлин ТМ (Ш)'].max()  # костыль
        this_state.p_wellhead_data_max_atm = prepared_data['Линейное давление (СУ)'].max() * 10
        this_state.qliq_max_m3day = prepared_data['Объемный дебит жидкости (СУ)'].max()

        for i in range(prepared_data.shape[0]):  # начало итерации по строкам - наборам данных для определенного времени
        #for i in range(3):
            proc_tool.auto_restart(i, options, UniflocVBA, current_path)
            start_in_loop_time = time.time()
            row_in_prepared_data = prepared_data.iloc[i]
            print('Итерация № ' + str(i+1) + ' из ' + str(prepared_data.shape[0]) +
                  ' в потоке №' + str(options.number_of_thread) + ' для времени ' + str(prepared_data.index[i]))

            this_state = workflow_input_data.transfer_data_from_row_to_state(this_state, row_in_prepared_data, opt.vfm_calc_option)

            this_result = mass_calculation(this_state, opt.debug_mode, opt.vfm_calc_option, opt.restore_q_liq_only)  # расчет

            end_in_loop_time = time.time()
            print("Затрачено времени в итерации: " + str(i) + " - " + str(end_in_loop_time - start_in_loop_time))

            new_dataframe = workflow_input_data.create_new_result_df(this_result, this_state, prepared_data, i)

            result_dataframe = result_dataframe.append(new_dataframe, sort=False)
            if opt.vfm_calc_option == True:
                result_dataframe.to_csv(dir_to_save_calculated_data + '\\' + opt.well_name + "_restore_" + str(opt.number_of_thread) + ".csv")
            else:
                result_dataframe.to_csv(dir_to_save_calculated_data + '\\' + opt.well_name + "_adapt_" + str(opt.number_of_thread) + ".csv")

        end_time = time.time()
        print("Затрачено всего: " + str(end_time - start_time))
    close_f = UniflocVBA.book.macro('close_book_by_macro')
    close_f()

# TODO добавить расчет для одного ядра


def run_calculation(thread_option_list):
    """
    Функция запускает многоточный расчет при прямом запуске из модуля, при импорте в app.ipynb не работает
    :param thread_option_list: спиской настроек для каждого потока
    :return:
    """
    if __name__ == '__main__':
        with Pool(amount_of_threads) as p:
            p.map(calc,
                  thread_option_list)


def create_thread_list(well_name, dir_name_with_input_data,static_data_full_path,
                       amount_of_threads):
    thread_list = []
    if 'restore' in dir_name_with_input_data:
        vfm_calc_option = restore_q_liq_only = True
    elif 'adapt' in dir_name_with_input_data:
        vfm_calc_option = restore_q_liq_only = False

    for number_of_thread in range(amount_of_threads):
        addin_name = 'UniflocVBA_7_%s.xlam' % str(number_of_thread)
        shutil.copyfile(current_path + 'UniflocVBA_7.xlam',
                        current_path + addin_name)
        this_thread = well_calculation.Calc_options(well_name=well_name,
                                   dir_name_with_input_data=dir_name_with_input_data,
                                   addin_name=addin_name,
                                   number_of_thread=number_of_thread, amount_of_threads=amount_of_threads,
                                   vfm_calc_option=vfm_calc_option, restore_q_liq_only=restore_q_liq_only,
                                                    static_data_full_path=static_data_full_path)
        thread_list.append(this_thread)
    return thread_list


static_data_full_path = "E:\\Git\\unifloc\\sandbox\\uTools\\data\\tr\\static_data.xlsx"

well_name = '6012'
dir_name_with_input_data = 'restore_input_'

amount_of_threads = 1

thread_option_list = create_thread_list(well_name, dir_name_with_input_data, static_data_full_path,
                       amount_of_threads)

start_time = time.time()
run_calculation(thread_option_list)
end_time = time.time()
print('Затрачено времени всего: ' + str(end_time - start_time))
