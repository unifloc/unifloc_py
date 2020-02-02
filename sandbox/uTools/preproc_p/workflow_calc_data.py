import pandas as pd
#from sandbox.uTools.preproc_p import preproc_tool
import sys
import os
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

global_names = preproc_tool.GlobalNames()

columns_name_dict = {"Pline_atma": global_names.p_lin_atm,
                     "pbuf_atma": global_names.p_buf_atm,
                     "Pdis_atma": global_names.p_discharge_atm,
                     "Pint_atma": global_names.p_intake_atm,
                     "pwf_atma": global_names.p_wf_atm,
                     "qliq_sm3day": global_names.q_liq_m3day,
                     "fw_perc": global_names.watercut_perc,
                     "tbh_C": global_names.t_wf_c,
                     "twh_C": global_names.t_wh_c,
                     "Tsurf_C": global_names.t_surface_c,
                     "t_dis_C": global_names.t_discharge_c,
                     "Tintake_C": global_names.t_intake_c,
                     "choke.c_degrad_fr": global_names.c_degrad_fr,
                     "Pcas_atma": global_names.p_cas_atm,
                     "Hdyn_m": global_names.h_dyn_m,
                     "ESP.power_CS_calc_W": global_names.active_power_kwt,
                     "ESP.freq_Hz": global_names.freq_hz, #TODO частота тока
                     "ESP.I_A": global_names.i_motor_a, #TODO ток двигателя
                     "ESP.U_V": global_names.u_motor_v, #TODO напряжение на двигателе
                     "ESP.load_fr": global_names.motor_load_perc,
                     "ESP.ESPpump.EffiencyESP_d":  global_names.efficiency_esp_d,
                     "ESP.KsepTotal_fr": global_names.KsepTotal_fr,
                     "ESP.KsepNat_fr": global_names.KsepNat_fr,
                     "ESP.KSepGasSep_fr": global_names.KSepGasSep_fr,
                     "ESP.c_calibr_head": global_names.c_calibr_head_d,
                     "ESP.c_calibr_power": global_names.c_calibr_power_d,
                     "ESP.c_calibr_rate": global_names.c_calibr_rate_d,
                     "ESP.power_motor_nom_W": global_names.power_motor_nom_kwt,
                     "ESP.cable_dU_V": global_names.cable_dU_V,
                     "ESP.dPower_GasSep_W": global_names.dPower_GasSep_kwt,
                     "ESP.dPower_protector_W": global_names.dPower_protector_kwt,
                     "ESP.cable_dPower_W": global_names.cable_dPower_kwt,
                     "ESP.dPower_transform_W": global_names.dPower_transform_kwt,
                     "ESP.dPower_CS_W": global_names.dPower_CS_kwt,
                     "ESP.ESPpump.Powerfluid_Wt": global_names.Powerfluid_kwt,
                     "ESP.ESPpump.PowerESP_Wt": global_names.PowerESP_kwt,
                     "ESP.power_shaft_W": global_names.power_shaft_kwt,
                     "ESP.power_motor_W": global_names.power_motor_kwt,
                     "ESP.cable_power_W": global_names.cable_power_kwt,
                     "ESP.power_CS_teor_calc_W": global_names.power_CS_teor_calc_kwt}


columns_to_dim_solve = [global_names.active_power_kwt,
                        global_names.power_motor_nom_kwt, global_names.dPower_GasSep_kwt,
                        global_names.dPower_protector_kwt, global_names.cable_dPower_kwt,
                        global_names.dPower_transform_kwt, global_names.dPower_CS_kwt,
                        global_names.Powerfluid_kwt, global_names.PowerESP_kwt, global_names.power_shaft_kwt,
                        global_names.power_motor_kwt,
                        global_names.cable_power_kwt,
                        global_names.power_CS_teor_calc_kwt]


def rename_columns_by_dict(df, dict=columns_name_dict):
    """
    Специальное изменение названий столбцов по словарю
    :param df:
    :param dict:
    :return:
    """
    for i in df.columns:
        if i in dict.keys():
            df = df.rename(columns={i: dict[i]})
    return df


def load_calculated_data_from_csv(full_file_name, mark, columns_to_dim_solve = columns_to_dim_solve, global_names = global_names):
    """
    Загрузка данных расчета модели
    :param full_file_name: полный путь к файлу
    :param mark: пометка данных модели (ADAPT - адаптация, PREDICT - прогноз)
    :return: исправленный файл с нормальныйми колонками
    """
    """
    Загрузка результатов расчета модели (адаптации или восстановления) и первичная обработка
    :param full_file_name:
    :return:
    """
    calculated_data = pd.read_csv(full_file_name)
    del calculated_data['Unnamed: 0']
    try:
        del calculated_data['d']
        calculated_data = calculated_data.iloc[1:]
    except:
        pass
    calculated_data.index = pd.to_datetime(calculated_data['Время'])
    del calculated_data['Время']
    calculated_data = rename_columns_by_dict(calculated_data)
    calculated_data[global_names.ch_cp_multiply] = calculated_data[global_names.c_calibr_head_d] * \
                                                       calculated_data[global_names.c_calibr_power_d]
    calculated_data[global_names.dp_esp_atm] = calculated_data[global_names.p_discharge_atm] - \
                                                     calculated_data[global_names.p_intake_atm]
    calculated_data[columns_to_dim_solve] = calculated_data[columns_to_dim_solve] / 1000  #Перевод в кВт
    calculated_data[global_names.motor_load_perc] = calculated_data[global_names.motor_load_perc] * 100
    calculated_data = preproc_tool.mark_df_columns(calculated_data, mark)
    return calculated_data