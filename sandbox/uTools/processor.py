"""
Модуль для массового расчета скважин, оснащенных УЭЦН

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
sys.path.append('C:\\Users\\olegk\\Documents\\')
current_path = os.getcwd()
current_path = current_path.replace(r'unifloc\sandbox\uTools', r'unifloc_vba\\')
print(current_path)

import unifloc_vba.description_generated.python_api as python_api
from scipy.optimize import minimize
import pandas as pd

import sys
import xlwings as xw
sys.path.append("../")
import datetime
import time
import os
time_mark = datetime.datetime.today().strftime('%Y_%m_%d_%H_%M_%S')
def calc(options):
    UniflocVBA = python_api.API(current_path + options[0])
    well_name = '507'
    dir_name_with_input_data = 'restore_input_2019_11_05_21_21_03'

    calc_mark_str = options[2]
    calc_option = True
    debug_mode = True
    vfm_calc_option = True
    restore_q_liq_only = True
    amount_iters_before_restart = 100
    sleep_time_sec = 25
    p_buf_value_in_error_coeff = 0.5

    if vfm_calc_option == False:
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

    class all_ESP_data():
        def __init__(self):
            self.ESP_rate_nom = 320
            self.esp_id = UniflocVBA.calc_ESP_id_by_rate(self.ESP_rate_nom)
            self.ESP_head_nom = 1200
            self.dcas_mm = 160
            self.h_tube_m = 833
            self.d_tube_mm = 76
            self.p_cas_data_atm = -1  # нет расчета затрубного пространства - он долгий и немножко бесполезный

            self.eff_motor_d = 0.89
            self.i_motor_nom_a = 50
            self.power_motor_nom_kwt = 125
            self.h_perf_m = 834  # ТР
            self.h_pump_m = 833  # ТР
            self.udl_m = 87  # ТР

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
            self.c_calibr_head_d = 0.5  # initial guess
            self.c_calibr_power_d = 0.5  # initial guess

            self.result = None
            self.error_in_step = None
            self.p_buf_data_max_atm = None
            self.active_power_cs_data_max_kwt = None


    def mass_calculation(this_state, debug_print = False, restore_flow=False, restore_q_liq_only = True):
        def calc_well_plin_pwf_atma_for_fsolve(minimaze_parameters):
            if restore_flow == False: # TODO изменить коэффициенты для восстановления дебита
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

            this_state.result = result
            p_buf_calc_atm = result[0][2]
            power_CS_calc_W = result[0][16]
            result_for_folve = p_buf_value_in_error_coeff * \
                               ((p_buf_calc_atm - this_state.p_buf_data_atm) / this_state.p_buf_data_max_atm) ** 2 + \
                               (1 - p_buf_value_in_error_coeff) * ((power_CS_calc_W - this_state.active_power_cs_data_kwt) /
                                this_state.active_power_cs_data_max_kwt) ** 2
            if debug_print:
                print("power_CS_calc_W = " + str(power_CS_calc_W))
                print("active_power_cs_data_kwt = " + str(this_state.active_power_cs_data_kwt))
                print("ошибка на текущем шаге = " + str(result_for_folve))
            this_state.error_in_step = result_for_folve
            return result_for_folve
        if restore_flow == False:
            result = minimize(calc_well_plin_pwf_atma_for_fsolve, [this_state.c_calibr_head_d, this_state.c_calibr_power_d],
                              bounds=[[0, 5], [0, 5]])
        else:
            if restore_q_liq_only == True:
                result = minimize(calc_well_plin_pwf_atma_for_fsolve, [this_state.qliq_m3day], bounds=[[3, 350]])
            else:
                result = minimize(calc_well_plin_pwf_atma_for_fsolve, [100, 20], bounds=[[5, 175], [10, 35]])
        print(result)
        true_result = this_state.result
        return true_result


    if calc_option == True:
        prepared_data = pd.read_csv(input_data_filename_str + ".csv")
        if options[1] == True:
            prepared_data = prepared_data.iloc[0:int(len(prepared_data.index) / 2)]
        else:
            prepared_data = prepared_data.iloc[int(len(prepared_data.index) / 2)::]
        prepared_data.index = pd.to_datetime(prepared_data["Время"])
        del prepared_data["Время"]

        result_list = []
        result_dataframe = {'d':[2]}
        result_dataframe = pd.DataFrame(result_dataframe)
        start_time = time.time()
        this_state = all_ESP_data()
        for i in range(prepared_data.shape[0]):
        #for i in range(3):
            check = i % amount_iters_before_restart
            if check == 0 and i != 0:
                print('Перезапуск Excel и VBA')
                UniflocVBA.book.close()
                time.sleep(sleep_time_sec)
                UniflocVBA.book = xw.Book(current_path + options[0])
            start_in_loop_time = time.time()
            row_in_prepared_data = prepared_data.iloc[i]
            print("Расчет для времени:")
            print(prepared_data.index[i])
            print('Итерация № ' + str(i) + ' из ' + str(prepared_data.shape[0]))
            this_state.watercut_perc = row_in_prepared_data['Процент обводненности (СУ)']
            this_state.rp_m3m3 = row_in_prepared_data['ГФ (СУ)']
            this_state.p_buf_data_atm = row_in_prepared_data['Рбуф (Ш)']
            this_state.p_wellhead_data_atm = row_in_prepared_data['Рлин ТМ (Ш)']
            this_state.tsep_c = row_in_prepared_data['Температура на приеме насоса (пласт. жидкость) (СУ)']
            this_state.p_intake_data_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
            this_state.psep_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
            this_state.p_wf_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
            this_state.d_choke_mm = row_in_prepared_data['Dшт (Ш)']
            this_state.ESP_freq = row_in_prepared_data['F вращ ТМ (Ш)']
            #this_state.ESP_freq = row_in_prepared_data['Выходная частота ПЧ (СУ)']
            this_state.active_power_cs_data_kwt = row_in_prepared_data['Активная мощность (СУ)'] * 1000
            this_state.u_motor_data_v = row_in_prepared_data['Напряжение на выходе ТМПН (СУ)']
            this_state.cos_phi_data_d = row_in_prepared_data['Коэффициент мощности (СУ)']
            if vfm_calc_option == True:
                this_state.c_calibr_head_d = row_in_prepared_data["К. калибровки по напору - множитель (Модель) (Подготовленные)"]
                this_state.c_calibr_power_d = row_in_prepared_data["К. калибровки по мощности - множитель (Модель) (Подготовленные)"]
            else:
                this_state.qliq_m3day = row_in_prepared_data['Объемный дебит жидкости (СУ)']

            this_state.active_power_cs_data_max_kwt = prepared_data['Активная мощность (СУ)'].max() * 1000
            this_state.p_buf_data_max_atm = prepared_data['Рбуф (Ш)'].max()
            this_result = mass_calculation(this_state, debug_mode, vfm_calc_option, restore_q_liq_only)
            result_list.append(this_result)
            end_in_loop_time = time.time()
            print("Затрачено времени в итерации: " + str(i) + " - " + str(end_in_loop_time - start_in_loop_time))
            new_dict = {}
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

from multiprocessing import Pool

options1 =["UniflocVBA_7.xlam",  False, '1']
options2 =["UniflocVBA_7_1.xlam",  True, '2']

if __name__ == '__main__':
    with Pool(2) as p:
        p.map(calc,
              [options1, options2])
