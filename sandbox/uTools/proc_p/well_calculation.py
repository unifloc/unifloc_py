import os
import sys
sys.path.append('../'*4)


class Calc_options():
    def __init__(self, well_name='1628',  # менять тут для адаптации/восстановления
                 dir_name_with_input_data='restore_input_',  # менять тут для адаптации/восстановления
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


def straight_calc(UniflocVBA, this_state):
    """
    Функция для прямого расчета скважины от приема ЭЦН
    :param UniflocVBA: API для вызова функций
    :param this_state: класс-состояние со всеми параметрами скважины
    :return: result - результат расчета в форме списка
    """
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
                                               t_dis_C=-1,
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
                                                this_state.ksep_d, this_state.c_calibr_head_d,
                                                this_state.c_calibr_power_d,
                                                this_state.c_calibr_rate_d)  # TODO сделать прямой расчет
    return result



