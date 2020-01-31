import sys
sys.path.append('../' * 4)
sys.path.append('../' * 3)
sys.path.append('../' * 2)
import sandbox.uTools.preproc_p.preproc_tool as preproc_tool
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool
import pandas as pd
from unifloc.sandbox.uTools.preproc_p import workflow_tr_data

global_names = preproc_tool.GlobalNames()


class all_ESP_data():  # класс, в котором хранятся данные
    def __init__(self, UniflocVBA, static_data: workflow_tr_data.Static_data):
        """
        класс для хранение и доступа ко всем данным скважины - входным, выходным
        :param UniflocVBA: текущая надстройка UniflocVBA API
        :param static_data: данные техрежима
        """
        self.esp_nom_rate_m3day = static_data.esp_nom_rate_m3day
        #self.esp_id = UniflocVBA.calc_ESP_id_by_rate(self.esp_nom_rate_m3day)
        self.esp_id = static_data.esp_id

        self.esp_nom_head_m = static_data.esp_nom_head_m
        self.d_cas_mm = static_data.d_cas_mm
        self.h_pump_m = static_data.h_pump_m
        self.d_tube_mm = static_data.d_tube_mm
        self.p_cas_data_atm = -1  # нет расчета затрубного пространства - он долгий и немножко бесполезный

        self.eff_motor_d = 0.89
        self.i_motor_nom_a = static_data.i_motor_nom_a
        self.power_motor_nom_kwt = static_data.power_motor_nom_kwt
        self.h_tube_m = self.h_pump_m  # ТР
        self.h_perf_m = self.h_pump_m + 0.0001  # ТР
        self.udl_m = static_data.udl_m  # ТР

        self.c_calibr_rate_d = 1

        self.ksep_d = 0.7  # ТР
        self.KsepGS_fr = 0.7  # ТР
        self.hydr_corr = 1  # 0 - BB, 1 - Ansari
        self.gamma_oil = static_data.gamma_oil
        self.gamma_gas = static_data.gamma_gas
        self.gamma_wat = static_data.gamma_wat
        self.rsb_m3m3 = static_data.rsb_m3m3
        self.tres_c = static_data.tres_c
        self.pb_atm = static_data.pb_atm
        self.bob_m3m3 = static_data.bob_m3m3
        self.muob_cp = static_data.muob_cp
        self.rp_m3m3 = static_data.rp_m3m3

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
        self.qliq_m3day = static_data.qliq_m3day_initial_guess  # initial guess
        self.watercut_perc = None
        self.p_buf_data_atm = None
        self.c_calibr_head_d = static_data.c_calibr_head_d_initial_guess  # initial guess
        self.c_calibr_power_d = static_data.c_calibr_power_d_initial_guess # initial guess

        self.result = None
        self.error_in_step = None
        self.p_buf_data_max_atm = None
        self.active_power_cs_data_max_kwt = None
        self.p_wellhead_data_max_atm = None
        self.qliq_max_m3day = None

        self.c_calibr_head_d_max_limit = static_data.c_calibr_head_d_max_limit
        self.c_calibr_head_d_min_limit = static_data.c_calibr_head_d_min_limit
        self.c_calibr_power_d_max_limit = static_data.c_calibr_power_d_max_limit
        self.c_calibr_power_d_min_limit = static_data.c_calibr_power_d_min_limit


def transfer_data_from_row_to_state(this_state, row_in_prepared_data, vfm_calc_option):
    """
    заполнение класса-состояния скважины с ЭЦН текущим набором входных данных (для данного момента времени)
    :param this_state: состояние скважины со всеми параметрами
    :param row_in_prepared_data: набора данных - строка входного DataFrame
    :param vfm_calc_option: флаг восстановления дебитов - если False - адаптация
    :return: заполненное состояние this_state
    """
    this_state.watercut_perc = row_in_prepared_data[global_names.watercut_perc]  # заполнение структуры данными
    this_state.rp_m3m3 = row_in_prepared_data[global_names.gor_m3m3]

    this_state.p_buf_data_atm = row_in_prepared_data[global_names.p_buf_atm]  #TODO заменить настоящим буф
    this_state.p_wellhead_data_atm = row_in_prepared_data[global_names.p_buf_atm]  #TODO заменить настоящим лин
    this_state.tsep_c = row_in_prepared_data[global_names.t_intake_c]
    this_state.p_intake_data_atm = row_in_prepared_data[global_names.p_intake_atm]
    this_state.psep_atm = row_in_prepared_data[global_names.p_intake_atm]
    this_state.p_wf_atm = row_in_prepared_data[global_names.p_intake_atm]
    this_state.d_choke_mm = row_in_prepared_data[global_names.d_choke_mm]
    this_state.ESP_freq = row_in_prepared_data[global_names.freq_hz]
    this_state.active_power_cs_data_kwt = row_in_prepared_data[global_names.active_power_kwt]
    this_state.u_motor_data_v = row_in_prepared_data[global_names.u_motor_v]
    this_state.cos_phi_data_d = row_in_prepared_data[global_names.cos_phi_d]
    if vfm_calc_option == True:
        this_state.c_calibr_head_d = row_in_prepared_data[global_names.c_calibr_head_d]
        this_state.c_calibr_power_d = row_in_prepared_data[global_names.c_calibr_power_d]
    else:
        this_state.qliq_m3day = row_in_prepared_data[global_names.q_liq_m3day]
    return this_state


def get_fragmentation(length: int, slaves: int) -> list:
    """
    with length of jobs and amount of slaves returns list with turples.
    job_list[out[slave][0]: out[slave][1]] is a sub-list of jobs for this slave. slave \in [0, slaves-1]
    :param length: number of jobs
    :param slaves: amount of slaves
    :return: turples to distribute jobs
    """
    # defining step size
    step = int(length / slaves)
    # but int-like devision returns a bit less
    r = length - step * slaves
    # preparing out list with turples. r will be distributed over first slaves
    out = []
    cur_point = 0
    for i in range(slaves):
        if r > 0:
            # if we have non dealed r, push a piece
            out_el = (cur_point, cur_point + step + 1)
            r -= 1
        else:
            # if all the r already distributed then ok
            out_el = (cur_point, cur_point + step)
        cur_point = out_el[1]
        out.append(out_el)
    return out


def divide_prepared_data(prepared_data, options):  # TODO сделать разбивку с запасом - убрать потери точек
    """
    Разбивка всех исходных данных на равные части для реализации многопоточности
    :param prepared_data: подготовленные входные данные
    :param options: класс настроек расчета
    :return: определенная часть исходных данных, которая будет считаться данным потоком
    """
    fragmentation = get_fragmentation(prepared_data.shape[0], options.amount_of_threads)
    out = prepared_data[fragmentation[options.number_of_thread-1][0]: fragmentation[options.number_of_thread-1][1]]
    return out


def create_new_result_df(this_result, this_state, prepared_data, i, global_names = global_names):
    """
    Объединение всех результатов для данной итерации в один DataFrame
    :param this_result: список с результатами UniflocVBA
    :param this_state: класс-состояние скважины для данной итерации (входные данные)
    :param prepared_data: DataFrame входных данных
    :param i: номер строки в prepared_data для текущей итерации
    :return: new_dataframe - сводный результат расчета
    """
    new_dict = {}
    for j in range(len(this_result[1])):
        new_dict[this_result[1][j]] = [this_result[0][j]]
        print(str(this_result[1][j]) + " -  " + str(this_result[0][j]))
    new_dict[global_names.gor_m3m3] = [this_state.rp_m3m3]
    new_dict[global_names.error_in_model] = [this_state.error_in_step]
    new_dict['Время'] = [prepared_data.index[i]]
    new_dataframe = pd.DataFrame(new_dict)
    new_dataframe.index = new_dataframe['Время']
    return new_dataframe



