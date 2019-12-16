import pandas as pd
#from sandbox.uTools.preproc_p import preproc_tool
import sys
import os
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

columns_name_dict = {"Pline_atma": "P лин., атм",
                     "pbuf_atma": "P буф., атм",
                     "Pdis_atma": "P выкид ЭЦН, атм",
                     "Pint_atma": "P прием ЭЦН, атм",
                     "pwf_atma": "P прием ЭЦН, атм (P заб. модели)",
                     "qliq_sm3day": "Q ж, м3/сут",
                     "fw_perc": "Обв, %",
                     "tbh_C": "T прием, С ",
                     "twh_C": "T устья, С",
                     "Tsurf_C": "T поверхности, С",
                     "t_dis_C": "T выкид ЭЦН, С",
                     "Tintake_C": "T прием ЭЦН, C",
                     "choke.c_degrad_fr": "К. деградации для штуцера, ед",
                     "Pcas_atma": "P затрубное, атм",
                     "Hdyn_m": "H дин.ур., м",
                     "ESP.power_CS_calc_W": "Акт. мощность на СУ",
                     "ESP.freq_Hz": "F тока, ГЦ",
                     "ESP.I_A": "I, А",
                     "ESP.U_V": "U, В",
                     "ESP.load_fr": "Загрузка двигателя",
                     "ESP.ESPpump.EffiencyESP_d": "КПД ЭЦН, д.ед.",
                     "ESP.KsepTotal_fr": "К. сеп. общий, д.ед.",
                     "ESP.KsepNat_fr": "К. сеп. естесственной, д.ед.",
                     "ESP.KSepGasSep_fr": "К. сеп. газосепаратора",
                     "ESP.c_calibr_head": "К. калибровки по напору - множитель",
                     "ESP.c_calibr_power": "К. калибровки по мощности - множитель",
                     "ESP.c_calibr_rate": "К. калибровки по дебиту - множитель",
                     "ESP.power_motor_nom_W": "Номинальная мощность ПЭД, Вт",
                     "ESP.cable_dU_V": "dU в кабеле, В",
                     "ESP.dPower_GasSep_W": "Мощность, потребляемая газосепаратором",
                     "ESP.dPower_protector_W": "Мощность, потребляемая протектором",
                     "ESP.cable_dPower_W": "Мощность, потребляемая (рассеиваемая) кабелем",
                     "ESP.dPower_transform_W": "Мощность, потребляемая ТМПН",
                     "ESP.dPower_CS_W": "Мощность, потребляемая СУ",
                     "ESP.ESPpump.Powerfluid_Wt": "Мощность, передаваемая жидкости",
                     "ESP.ESPpump.PowerESP_Wt": "Мощность, передаваемая ЭЦН",
                     "ESP.power_shaft_W": "Мощность, передаваемая валу от ПЭД",
                     "ESP.power_motor_W": "Мощность, передаваемая ПЭД",
                     "ESP.cable_power_W": "Мощность, передаваемая кабелю",
                     "ESP.power_CS_teor_calc_W": "Мощность, передаваемая СУ"}


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


def load_calculated_data_from_csv(full_file_name):
    """
    Загрузка результатов расчета модели (адаптации или восстановления) и первичная обработка
    :param full_file_name:
    :return:
    """
    calculated_data = pd.read_csv(full_file_name)
    del calculated_data['Unnamed: 0']
    del calculated_data['Unnamed: 42']
    try:
        del calculated_data['d']
        calculated_data = calculated_data.iloc[1:]
    except:
        pass
    calculated_data.index = pd.to_datetime(calculated_data['Время'])
    del calculated_data['Время']
    calculated_data = rename_columns_by_dict(calculated_data)
    calculated_data['Произведение калибровок H и N'] = calculated_data['К. калибровки по напору - множитель'] * \
                                                       calculated_data['К. калибровки по мощности - множитель']
    calculated_data['Перепад давления в ЭЦН, атм'] = calculated_data['P выкид ЭЦН, атм'] - \
                                                     calculated_data['P прием ЭЦН, атм']
    calculated_data = preproc_tool.mark_df_columns(calculated_data, 'Модель')
    return calculated_data