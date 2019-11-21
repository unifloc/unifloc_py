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

import sys
import os
sys.path.append('../')
current_path = os.getcwd()
path_to_sys = current_path.replace(r'unifloc\sandbox\uTools', '')
sys.path.append(path_to_sys)  # добавляем путь в sys, чтобы нашелся проект unifloc_vba
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
import unifloc.sandbox.uTools.preprocessor as prep


time_mark = datetime.datetime.today().strftime('%Y_%m_%d_%H_%M')  # временная метка для сохранения без перезаписи

class Calc_options():  #TODO сделать класс-структуру со всем (настройки расчета отдельно здесь, алгоритм отдельно)
    def __init__(self, well_name='252',  # менять тут для адаптации/восстановления
                 dir_name_with_input_data='restore_input_2019_11_16_17_36_10',  # менять тут для адаптации/восстановления
                 multiprocessing=True,
                 addin_name="UniflocVBA_7.xlam",
                 tr_name="Техрежим, , февраль 2019.xls",
                 number_of_thread=1,
                 amount_of_threads=4,
                 use_pwh_in_loss=False,
                 calc_option=True,
                 debug_mode=False,
                 vfm_calc_option=True,  # менять тут для адаптации/восстановления
                 restore_q_liq_only=True,  # менять тут для адаптации/восстановления
                 amount_iters_before_restart=100,
                 sleep_time_sec=25,
                 hydr_part_weight_in_error_coeff=0.5):  #TODO добавлять насосы в UniflocVBA
        """
        класс для сбора всех настроек, необходимых для расчета
        :param well_name: имя скважины
        :param dir_name_with_input_data: название директории с входными данными (adaptation_input или restore_input)
        :param multiprocessing: флаг для расчета в многопотоке(предварительно нужно размножить unifloc_vba.xlam
        :param addin_name: название надстройки
        :param number_of_thread: порядковый номер этого потока
        :param amount_of_threads: общее число потоков
        :param use_pwh_in_loss: флаг использования линейного давления в функции ошибки
        :param calc_option: флаг расчета, если True - начала итераций по строкам в df
        :param debug_mode: флаг отладки, если True - онлайн вывод значений функции ошибки и других важных параметров
        :param vfm_calc_option: флаг метода расчета, если True - восстановление, если - False - адаптация
        :param restore_q_liq_only: флаг метода восстановления, если True - только дебита жидкости
        :param amount_iters_before_restart: количество итераций перед перезапуском экселя
        :param sleep_time_sec: время отдыха после закрытия экселя
        :param hydr_part_weight_in_error_coeff: гиперпараметр на гидравлическую часть в функции ошибки
        """
        self.well_name = well_name
        self.dir_name_with_input_data = dir_name_with_input_data
        self.multiprocessing = multiprocessing
        self.addin_name = addin_name
        self.number_of_thread = number_of_thread
        self.amount_of_threads = amount_of_threads
        self.use_pwh_in_loss = use_pwh_in_loss
        self.tr_name = tr_name
        self.calc_option = calc_option
        self.debug_mode = debug_mode
        self.vfm_calc_option = vfm_calc_option
        self.restore_q_liq_only = restore_q_liq_only
        self.amount_iters_before_restart = amount_iters_before_restart
        self.sleep_time_sec = sleep_time_sec
        self.hydr_part_weight_in_error_coeff = hydr_part_weight_in_error_coeff

def transfer_data_from_row_to_state(this_state, row_in_prepared_data, vfm_calc_option):
    """
    заполнение класса-состояния скважины с ЭЦН текущим набором входных данных (для данного момента времени)
    :param this_state: состояние скважины со всеми параметрами
    :param row_in_prepared_data: набора данных - строка входного DataFrame
    :param vfm_calc_option: флаг восстановления дебитов - если False - адаптация
    :return: заполненное состояние this_state
    """
    this_state.watercut_perc = row_in_prepared_data['Процент обводненности (СУ)']  # заполнение структуры данными
    this_state.rp_m3m3 = row_in_prepared_data['ГФ (СУ)']
    this_state.p_buf_data_atm = row_in_prepared_data['Рбуф (Ш)']
    # this_state.p_wellhead_data_atm = row_in_prepared_data['Рлин ТМ (Ш)']
    this_state.p_wellhead_data_atm = row_in_prepared_data['Линейное давление (СУ)'] * 10
    this_state.tsep_c = row_in_prepared_data['Температура на приеме насоса (пласт. жидкость) (СУ)']
    this_state.p_intake_data_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
    this_state.psep_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
    this_state.p_wf_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
    this_state.d_choke_mm = row_in_prepared_data['Dшт (Ш)']
    this_state.ESP_freq = row_in_prepared_data['F вращ ТМ (Ш)']
    # this_state.ESP_freq = row_in_prepared_data['Выходная частота ПЧ (СУ)']
    this_state.active_power_cs_data_kwt = row_in_prepared_data['Активная мощность (СУ)'] * 1000
    this_state.u_motor_data_v = row_in_prepared_data['Напряжение на выходе ТМПН (СУ)']
    this_state.cos_phi_data_d = row_in_prepared_data['Коэффициент мощности (СУ)']
    if vfm_calc_option == True:
        this_state.c_calibr_head_d = row_in_prepared_data[
            "К. калибровки по напору - множитель (Модель) (Подготовленные)"]
        this_state.c_calibr_power_d = row_in_prepared_data[
            "К. калибровки по мощности - множитель (Модель) (Подготовленные)"]
    else:
        this_state.qliq_m3day = row_in_prepared_data['Объемный дебит жидкости (СУ)']
    return this_state

class all_ESP_data(): # класс, в котором хранятся данные
    def __init__(self, UniflocVBA, tr_data):
        """
        класс для хранение и доступа ко всем данным скважины - входным, выходным
        :param UniflocVBA: текущая надстройка UniflocVBA API
        :param tr_data: данные техрежима
        """
        self.ESP_rate_nom = tr_data.esp_nom_rate_m3day
        self.esp_id = UniflocVBA.calc_ESP_id_by_rate(self.ESP_rate_nom)
        self.ESP_head_nom = tr_data.esp_nom_head_m
        self.dcas_mm = tr_data.d_cas_mm
        self.h_pump_m = tr_data.h_pump_m
        self.d_tube_mm = tr_data.d_tube_mm
        self.p_cas_data_atm = -1  # нет расчета затрубного пространства - он долгий и немножко бесполезный

        self.eff_motor_d = 0.89
        self.i_motor_nom_a = tr_data.i_motor_nom_a
        self.power_motor_nom_kwt = tr_data.power_motor_nom_kwt
        self.h_tube_m = self.h_pump_m  # ТР
        self.h_perf_m = self.h_pump_m + 1  # ТР
        self.udl_m = tr_data.udl_m  # ТР

        self.c_calibr_rate_d = 1

        self.ksep_d = 0.7  # ТР
        self.KsepGS_fr = 0.7  # ТР
        self.hydr_corr = 1  # 0 - BB, 1 - Ansari
        self.gamma_oil = 0.945
        self.gamma_gas = 0.9
        self.gamma_wat = 1.011
        self.rsb_m3m3 = 29.25
        self.tres_c = 16
        self.pb_atm = 40
        self.bob_m3m3 = 1.045
        self.muob_cp = 100
        self.rp_m3m3 = 30

        self.psep_atm = None
        self.tsep_c = None

        self.d_choke_mm = None
        self.ESP_freq = None
        self.p_intake_data_atm = None
        self.p_wellhead_data_atm = None
        self.p_buf_data_atm = None
        self.p_wf_atm = None
        self.cos_phi_data_d = None
        self.u_motor_data_v = None
        self.active_power_cs_data_kwt = None
        self.qliq_m3day = 100 # initial guess
        self.watercut_perc = None
        self.p_buf_data_atm = None
        self.c_calibr_head_d = 0.7  # initial guess
        self.c_calibr_power_d = 1.2  # initial guess

        self.result = None
        self.error_in_step = None
        self.p_buf_data_max_atm = None
        self.active_power_cs_data_max_kwt = None
        self.p_wellhead_data_max_atm = None
        self.qliq_max_m3day = None

def calc(options=Calc_options()):
    """
    Основная расчетная функция, в которой есть все
    :param options: структура со всеми надстройками, данными, параметрами
    :return: None
    """
    UniflocVBA = python_api.API(current_path + options.addin_name)
    well_name = options.well_name
    dir_name_with_input_data = options.dir_name_with_input_data

    calc_mark_str = str(options.number_of_thread)
    calc_option = options.calc_option # флаг расчета, если  False, не будет делать ничего
    debug_mode = options.debug_mode
    vfm_calc_option = options.vfm_calc_option  # True - для адаптации, False - для восстановления
    restore_q_liq_only = options.restore_q_liq_only  # True - для адаптации, False - для восстановления
    amount_iters_before_restart = options.amount_iters_before_restart  # после 25 итерации (временных) могут возникать ошибки
    sleep_time_sec = options.sleep_time_sec
    hydr_part_weight_in_error_coeff = options.hydr_part_weight_in_error_coeff

    tr_file_full_path = os.getcwd() + '\\data\\tr\\' + options.tr_name
    tr_data = prep.read_tr_and_get_data(tr_file_full_path, options.well_name)  # прочитаем техрежим и извлечем данным

    if vfm_calc_option == False:  # создание директорий для результатов расчета
        input_data_filename_str = os.getcwd() + '\\data\\' + well_name + '\\' + dir_name_with_input_data + '\\' + well_name + '_adapt_input'
        dir_to_save_calculated_data = os.getcwd() + '\\data\\' + well_name + '\\' + 'adaptation_' + time_mark
        try:
            os.mkdir(dir_to_save_calculated_data)
        except:
            pass
    else:
        input_data_filename_str = os.getcwd() + '\\data\\' + well_name + '\\' + dir_name_with_input_data + '\\' + well_name + '_restore_input'
        dir_to_save_calculated_data = os.getcwd() + '\\data\\' + well_name + '\\' + 'restore_' + time_mark
        try:
            os.mkdir(dir_to_save_calculated_data)
        except:
            pass
    if options.multiprocessing:
        dir_to_save_calculated_data += '\\' + 'multiprocessing'
        try:
            os.mkdir(dir_to_save_calculated_data)
        except:
            pass



    def mass_calculation(this_state, debug_print = False, restore_flow=False, restore_q_liq_only = True):
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
            # последовательный запуск функций UniflocVBA для расчета модели
            PVTstr = UniflocVBA.calc_PVT_encode_string(this_state.gamma_gas, this_state.gamma_oil,
                                                       this_state.gamma_wat, this_state.rsb_m3m3, this_state.rp_m3m3,
                                                       this_state.pb_atm, this_state.tres_c,
                                                       this_state.bob_m3m3, this_state.muob_cp,
                                                       ksep_fr=this_state.ksep_d, pksep_atma=this_state.psep_atm,
                                                       tksep_C=this_state.tsep_c)
            Wellstr = UniflocVBA.calc_well_encode_string(this_state.h_perf_m,
                                                         this_state.h_pump_m,
                                                         this_state.udl_m,
                                                         this_state.dcas_mm,
                                                         this_state.d_tube_mm,
                                                         this_state.d_choke_mm,
                                                         tbh_C=this_state.tsep_c)
            ESPstr = UniflocVBA.calc_ESP_encode_string(this_state.esp_id,
                                                       this_state.ESP_head_nom,
                                                       this_state.ESP_freq,
                                                       this_state.u_motor_data_v,
                                                       this_state.power_motor_nom_kwt,
                                                       this_state.tsep_c,
                                                       t_dis_C = -1,
                                                       KsepGS_fr=this_state.KsepGS_fr,
                                                       ESP_Hmes_m=this_state.h_tube_m,
                                                       c_calibr_head=this_state.c_calibr_head_d,
                                                       c_calibr_rate=this_state.c_calibr_rate_d,
                                                       c_calibr_power=this_state.c_calibr_power_d,
                                                       cos_phi=this_state.cos_phi_data_d)
            result = UniflocVBA.calc_well_plin_pwf_atma(this_state.qliq_m3day, this_state.watercut_perc,
                                                        this_state.p_wf_atm,
                                                        this_state.p_cas_data_atm, Wellstr,
                                                        PVTstr, ESPstr, this_state.hydr_corr,
                                                        this_state.ksep_d, this_state.c_calibr_head_d, this_state.c_calibr_power_d,
                                                        this_state.c_calibr_rate_d)

            this_state.result = result # сохранение результата в форме списка в структуру для последующего извлечения
            p_line_calc_atm = result[0][0]
            p_buf_calc_atm = result[0][2]
            power_CS_calc_W = result[0][16]
            if options.use_pwh_in_loss == True: # функция ошибки
                result_for_folve = hydr_part_weight_in_error_coeff * \
                                   ((p_line_calc_atm - this_state.p_wellhead_data_atm) / this_state.p_wellhead_data_max_atm) ** 2 + \
                                   (1 - hydr_part_weight_in_error_coeff) * ((power_CS_calc_W - this_state.active_power_cs_data_kwt) /
                                    this_state.active_power_cs_data_max_kwt) ** 2
            else:
                result_for_folve = hydr_part_weight_in_error_coeff * \
                                   ((p_buf_calc_atm - this_state.p_buf_data_atm) / this_state.p_buf_data_max_atm) ** 2 + \
                                   (1 - hydr_part_weight_in_error_coeff) * ((power_CS_calc_W - this_state.active_power_cs_data_kwt) /
                                    this_state.active_power_cs_data_max_kwt) ** 2

            if debug_print:
                print("Линейное давление в модели = " + str(p_line_calc_atm))
                print("Мощность в модели = " + str(power_CS_calc_W))
                print("Абсолютная ошибка по мощности = " + str(power_CS_calc_W - this_state.active_power_cs_data_kwt))
                print("Абсолютная ошибка по давлению = " + str(p_line_calc_atm - this_state.p_wellhead_data_atm))
                print("ошибка на текущем шаге = " + str(result_for_folve))
            this_state.error_in_step = result_for_folve
            return result_for_folve

        if restore_flow == False: # выполнение оптимизации модели скважины с текущим набором данных
            result = minimize(calc_well_plin_pwf_atma_for_fsolve, [this_state.c_calibr_head_d, this_state.c_calibr_power_d],
                              bounds=[[0.45, 5], [0.45, 5]])
        else:
            if restore_q_liq_only == True:
                result = minimize(calc_well_plin_pwf_atma_for_fsolve, [this_state.qliq_m3day], bounds=[[3, this_state.qliq_max_m3day * 1.2]])
            else:
                result = minimize(calc_well_plin_pwf_atma_for_fsolve, [100, 20], bounds=[[5, 175], [10, 35]])
        print(result)
        true_result = this_state.result # сохранение результатов расчета оптимизированной модели
        return true_result

    if calc_option == True: # основной цикл расчета начинается здесь
        prepared_data = pd.read_csv(input_data_filename_str + ".csv") # чтение входных данных

        if options.number_of_thread == options.amount_of_threads == 1: # определение задействования многопоточности
            pass
        elif options.number_of_thread == options.amount_of_threads: # TODO переделать разбивку данных - есть пропуски
            prepared_data = prepared_data.iloc[-int(len(prepared_data.index) / options.amount_of_threads)::]
        elif options.number_of_thread == 1:
            prepared_data = prepared_data.iloc[0:int(len(prepared_data.index) / options.amount_of_threads)]
        else:
            first_index = int(len(prepared_data.index) / options.amount_of_threads * (options.number_of_thread - 1))
            second_index = first_index + int(len(prepared_data.index) / options.amount_of_threads)
            prepared_data = prepared_data.iloc[first_index: second_index]

        prepared_data.index = pd.to_datetime(prepared_data["Время"])
        del prepared_data["Время"]

        result_list = []
        result_dataframe = {'d':[2]}
        result_dataframe = pd.DataFrame(result_dataframe)
        start_time = time.time()
        this_state = all_ESP_data(UniflocVBA, tr_data)
        for i in range(prepared_data.shape[0]):  # начало итерации по строкам - наборам данных для определенного времени
        #for i in range(3):
            check = i % amount_iters_before_restart
            if check == 0 and i != 0: # защита против подвисаний экселя - не работает в многопотоке
                print('Перезапуск Excel и VBA')
                UniflocVBA.book.close()
                time.sleep(sleep_time_sec)
                UniflocVBA.book = xw.Book(current_path + options.addin_name)
            start_in_loop_time = time.time()
            row_in_prepared_data = prepared_data.iloc[i]
            print("Расчет для времени:")
            print(prepared_data.index[i])
            print('Итерация № ' + str(i) + ' из ' + str(prepared_data.shape[0]) +
                  ' в потоке №' + str(options.number_of_thread))

            this_state = transfer_data_from_row_to_state(this_state, row_in_prepared_data, vfm_calc_option)

            this_state.active_power_cs_data_max_kwt = prepared_data['Активная мощность (СУ)'].max() * 1000
            this_state.p_buf_data_max_atm = prepared_data['Рбуф (Ш)'].max()
            this_state.p_wellhead_data_max_atm = prepared_data['Линейное давление (СУ)'].max() * 10
            this_state.qliq_max_m3day = prepared_data['Объемный дебит жидкости (СУ)'].max()

            this_result = mass_calculation(this_state, debug_mode, vfm_calc_option, restore_q_liq_only)  # расчет

            end_in_loop_time = time.time()
            print("Затрачено времени в итерации: " + str(i) + " - " + str(end_in_loop_time - start_in_loop_time))
            new_dict = {} # преобразование и сохранение результатов
            result_list.append(this_result)
            for j in range(len(this_result[1])):
                new_dict[this_result[1][j]] = [this_result[0][j]]
                print(str(this_result[1][j]) + " -  " + str(this_result[0][j]))
            new_dict['ГФ'] = [this_state.rp_m3m3]
            new_dict['Значение функции ошибки'] = [this_state.error_in_step]
            new_dict['Время'] = [prepared_data.index[i]]
            new_dataframe = pd.DataFrame(new_dict)
            new_dataframe.index = new_dataframe['Время']
            result_dataframe = result_dataframe.append(new_dataframe, sort=False)
            if vfm_calc_option == True:
                result_dataframe.to_csv(dir_to_save_calculated_data + '\\' + well_name + "_restore_" + calc_mark_str + ".csv")
            else:
                result_dataframe.to_csv(dir_to_save_calculated_data + '\\' + well_name + "_adapt_" + calc_mark_str + ".csv")

        end_time = time.time()
        print("Затрачено всего: " + str(end_time - start_time))
    close_f = UniflocVBA.book.macro('close_book_by_macro')
    close_f()

# настройка многопоточности
amount_of_threads = 4

first_thread = Calc_options(addin_name="UniflocVBA_7.xlam", number_of_thread=1, amount_of_threads=amount_of_threads)
second_thread = Calc_options(addin_name="UniflocVBA_7_1.xlam", number_of_thread=2, amount_of_threads=amount_of_threads)
third_thread = Calc_options(addin_name="UniflocVBA_7_2.xlam", number_of_thread=3, amount_of_threads=amount_of_threads)
fourth_thread = Calc_options(addin_name="UniflocVBA_7_3.xlam", number_of_thread=4, amount_of_threads=amount_of_threads)

#TODO добавить расчет для одного ядра

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

thread_option_list =  [first_thread, second_thread, third_thread, fourth_thread]
run_calculation(thread_option_list)

