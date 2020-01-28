import pandas as pd
import sys
import os

sys.path.append('../' * 4)
sys.path.append('../' * 3)
sys.path.append('../' * 2)
import sandbox.uTools.preproc_p.preproc_tool as preproc_tool
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

global_names = preproc_tool.GlobalNames()


def create_banches_for_report(report_type):
    """
    Создание списка словарей для построения графиков в форме report
    :param report_type: string c типом report: second_edit_data, adaptation, adapt_input, adapt_report, restore_input,
            overall_result
    :return:
    """
    if report_type == 'second_edit_data':
        qliq = {global_names.q_liq_m3day: [global_names.q_liq_m3day]}
        gor = {global_names.gor_m3m3: [global_names.gor_m3m3]}
        wc = {global_names.watercut_perc: [global_names.watercut_perc]}
        pressure_intake = {global_names.p_intake_atm: [global_names.p_intake_atm]}
        pressure_wh = {global_names.p_buf_atm: [global_names.p_buf_atm]}
        temp_intake = {global_names.t_intake_c: [global_names.t_intake_c]}
        frequencies = {global_names.freq_hz: [global_names.freq_hz]}
        load = {global_names.motor_load_perc: [global_names.motor_load_perc]}
        power = {global_names.active_power_kwt: [global_names.active_power_kwt]}
        voltage = {global_names.u_motor_v: [global_names.u_motor_v]}
        cos = {global_names.cos_phi_d: [global_names.cos_phi_d]}
        current = {global_names.i_a_motor_a: [global_names.i_a_motor_a]}
        choke = {global_names.d_choke_mm: [global_names.d_choke_mm]}
        all_banches = [qliq, gor, wc, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load, power, voltage, cos, current, choke]
        return all_banches
    if report_type == 'adaptation':
        mark = " (ADAPT)"
        qliq = {global_names.q_liq_m3day: [global_names.q_liq_m3day, global_names.q_liq_m3day + mark]}
        calibr = {"Калибровки по напору и мощности": [global_names.c_calibr_head_d + mark,
                                                      global_names.c_calibr_power_d + mark]}
        error = {global_names.error_in_model: [global_names.error_in_model + mark]}
        gor = {global_names.gor_m3m3: [global_names.gor_m3m3, global_names.gor_m3m3 + mark]}
        wc = {global_names.watercut_perc: [global_names.watercut_perc, global_names.watercut_perc + mark]}
        pressure_intake = {global_names.p_intake_atm: [global_names.p_intake_atm, global_names.p_intake_atm + mark]}
        pressure_wh = {global_names.p_buf_atm: [global_names.p_buf_atm, global_names.p_buf_atm + mark]}
        temp_intake = {global_names.t_intake_c: [global_names.t_intake_c, global_names.t_intake_c + mark]}
        frequencies = {global_names.freq_hz: [global_names.freq_hz, global_names.freq_hz + mark]}
        load = {global_names.motor_load_perc: [global_names.motor_load_perc, global_names.motor_load_perc + mark]}
        power = {global_names.active_power_kwt: [global_names.active_power_kwt, global_names.active_power_kwt + mark]}
        voltage = {global_names.u_motor_v: [global_names.u_motor_v, global_names.u_motor_v + mark]}
        cos = {global_names.cos_phi_d: [global_names.cos_phi_d]}
        current = {global_names.i_a_motor_a: [global_names.i_a_motor_a, global_names.i_motor_a + mark]}
        choke = {global_names.d_choke_mm: [global_names.d_choke_mm]}
        all_banches = [qliq, calibr, error, gor, wc, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load, power, voltage, cos, current, choke]
        return all_banches
    if report_type == 'predict_input':
        mark = " (ADAPT)"
        qliq = {global_names.q_liq_m3day: [global_names.q_liq_m3day]}
        calibr = {"Калибровки по напору и мощности": [global_names.c_calibr_head_d, global_names.c_calibr_head_d + mark,
                                                      global_names.c_calibr_power_d, global_names.c_calibr_power_d + mark]}
        gor = {global_names.gor_m3m3: [global_names.gor_m3m3]}
        wc = {global_names.watercut_perc: [global_names.watercut_perc]}
        pressure_intake = {global_names.p_intake_atm: [global_names.p_intake_atm]}
        pressure_wh = {global_names.p_buf_atm: [global_names.p_buf_atm]}
        temp_intake = {global_names.t_intake_c: [global_names.t_intake_c]}
        frequencies = {global_names.freq_hz: [global_names.freq_hz]}
        load = {global_names.motor_load_perc: [global_names.motor_load_perc]}
        power = {global_names.active_power_kwt: [global_names.active_power_kwt]}
        voltage = {global_names.u_motor_v: [global_names.u_motor_v]}
        cos = {global_names.cos_phi_d: [global_names.cos_phi_d]}
        choke = {global_names.d_choke_mm: [global_names.d_choke_mm]}
        all_banches = [qliq, calibr, gor, wc, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load, power, voltage, cos, choke]
        return all_banches
    if report_type == 'prediction':
        mark = " (PREDICTION)"
        qliq = {global_names.q_liq_m3day: [global_names.q_liq_m3day, global_names.q_liq_m3day + mark]}
        calibr = {"Калибровки по напору и мощности": [global_names.c_calibr_head_d + " (ADAPT)",
                                                      global_names.c_calibr_head_d + mark,
                                                      global_names.c_calibr_power_d + " (ADAPT)",
                                                      global_names.c_calibr_power_d + mark]}
        error = {global_names.error_in_model: [global_names.error_in_model + mark]}
        gor = {global_names.gor_m3m3: [global_names.gor_m3m3, global_names.gor_m3m3 + mark]}
        wc = {global_names.watercut_perc: [global_names.watercut_perc, global_names.watercut_perc + mark]}
        pressure_intake = {global_names.p_intake_atm: [global_names.p_intake_atm, global_names.p_intake_atm + mark]}
        pressure_wh = {global_names.p_buf_atm: [global_names.p_buf_atm, global_names.p_buf_atm + mark]}
        temp_intake = {global_names.t_intake_c: [global_names.t_intake_c, global_names.t_intake_c + mark]}
        frequencies = {global_names.freq_hz: [global_names.freq_hz, global_names.freq_hz + mark]}
        load = {global_names.motor_load_perc: [global_names.motor_load_perc, global_names.motor_load_perc + mark]}
        power = {global_names.active_power_kwt: [global_names.active_power_kwt, global_names.active_power_kwt + mark]}
        voltage = {global_names.u_motor_v: [global_names.u_motor_v, global_names.u_motor_v + mark]}
        cos = {global_names.cos_phi_d: [global_names.cos_phi_d]}
        choke = {global_names.d_choke_mm: [global_names.d_choke_mm]}
        all_banches = [qliq, calibr, error, gor, wc, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load, power, voltage, cos, choke]
        return all_banches
    if report_type == 'overall_result':
        mark = " (PREDICTION)"
        mark0 = " (ADAPT)"
        qliq = {global_names.q_liq_m3day: [global_names.q_liq_m3day+ mark0, global_names.q_liq_m3day + mark]}
        calibr = {"Калибровки по напору и мощности": [global_names.c_calibr_head_d + mark0,
                                                      global_names.c_calibr_head_d + mark,
                                                      global_names.c_calibr_power_d + mark0,
                                                      global_names.c_calibr_power_d + mark]}
        relative_errors = {"Относительные ошибки расчета": [global_names.relative_error_active_power_perc,
                                                            global_names.relative_error_q_liq_perc,
                                                            global_names.relative_error_head_calibr_perc,
                                                            global_names.relative_error_power_calibr_perc]}
        error = {global_names.error_in_model: [global_names.error_in_model + mark, global_names.error_in_model + mark0]}
        gor = {global_names.gor_m3m3: [global_names.gor_m3m3+ mark0, global_names.gor_m3m3 + mark]}
        wc = {global_names.watercut_perc: [global_names.watercut_perc + mark0, global_names.watercut_perc + mark]}
        pressure_intake = {global_names.p_intake_atm: [global_names.p_intake_atm + mark0, global_names.p_intake_atm + mark]}
        pressure_wh = {global_names.p_buf_atm: [global_names.p_buf_atm + mark0, global_names.p_buf_atm + mark]}
        temp_intake = {global_names.t_intake_c: [global_names.t_intake_c + mark0, global_names.t_intake_c + mark]}
        frequencies = {global_names.freq_hz: [global_names.freq_hz + mark0, global_names.freq_hz + mark]}
        load = {global_names.motor_load_perc: [global_names.motor_load_perc + mark0, global_names.motor_load_perc + mark]}
        power = {global_names.active_power_kwt: [global_names.active_power_kwt + mark0, global_names.active_power_kwt + mark]}
        voltage = {global_names.u_motor_v: [global_names.u_motor_v + mark0, global_names.u_motor_v + mark]}
        cos = {global_names.cos_phi_d: [global_names.cos_phi_d]}
        choke = {global_names.d_choke_mm: [global_names.d_choke_mm]}
        all_banches = [qliq, relative_errors, calibr,  error, gor, wc, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load, power, voltage, cos, choke]
        return all_banches