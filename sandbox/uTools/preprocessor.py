"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль для обработки данных скважин, оснащенных УЭЦН (со СУ и шахматки)
"""
import sys
import os

current_path = os.getcwd()
path_to_vba = current_path.replace(r'unifloc\sandbox\uTools', '')
sys.path.append(path_to_vba)
import pandas as pd

# import unifloc_vba.description_generated.python_api as python_api

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


def mark_df_columns(df, mark):
    for i in df.columns:
        df = df.rename(columns={i: i + ' (' + mark + ')'})
    return df


def initial_editing(df, wellname):
    if len(df.columns) == 4:
        del df[0]
    df.columns = [0, 1, 2]
    test_str = df[0][0]
    index = test_str.find(wellname)
    str_to_delete = test_str[:index] + wellname + '. '
    df[0] = df[0].str.replace(str_to_delete, "")
    df = df.rename(columns={1: 'Время'})
    df.index = pd.to_datetime(df['Время'])
    del df['Время']
    return df


def extract_df_one_parametr_and_edit(df, list_of_params, number_of_param_in_list):
    extracted_df_one_param = df[df[0] == list_of_params[number_of_param_in_list]]
    extracted_df_one_param = extracted_df_one_param.dropna()
    extracted_df_one_param = extracted_df_one_param.rename(columns={2: extracted_df_one_param[0][0]})
    del extracted_df_one_param[0]
    return extracted_df_one_param


def create_edited_df(df):
    parametrs_list = df[0].unique()
    init_one_parametr_df = extract_df_one_parametr_and_edit(df, parametrs_list, 0)
    result = init_one_parametr_df
    for i in range(1, len(parametrs_list)):
        new_one_parametr_df = extract_df_one_parametr_and_edit(df, parametrs_list, i)
        result = result.join(new_one_parametr_df, how="outer", sort=True)
    return result


def cut_df(df, left_boundary, right_boundary):
    """
    Вырезка из DataFrame временного интервала
    :param df:
    :param left_boundary:
    :param right_boundary:
    :return:
    """
    for start, end in zip(left_boundary, right_boundary):
        df = df[(df.index >= start) & (df.index <= end)]
    return df


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
    calculated_data = mark_df_columns(calculated_data, 'Модель')
    return calculated_data


def make_gaps_and_interpolate(df, reverse=False):
    """
    Выкалывание точек и линейная интерполяция. (Восстановление дебитов путем интерполяции)
    :param df:
    :param reverse:
    :return:
    """
    try_check = df.copy()
    try_check['Время'] = try_check.index
    lenth = len(try_check['Время'])
    try_check.index = range(lenth)
    if reverse:
        try_check = try_check[(try_check.index) % 2 != 0]
    else:
        try_check = try_check[(try_check.index) % 2 == 0]
    try_check = try_check.interpolate()
    empty = pd.DataFrame({'empty': list(range(lenth))})
    result = empty.join(try_check, how='outer')
    result.index = df.index
    result = result.interpolate()
    result['Время'] = result.index
    result.index = result['Время']
    del result['empty']
    del result['Время']
    return result


def load_and_edit_cs_data(cs_data_filename, created_input_data_type=0, time_to_resamle=None):
    """
    Загрузка и обработка данных со СУ (не сырых, предварительно обработанных)
    :param cs_data_filename: data
    :param created_input_data_type: some kind of falg, 0 as default
    :param time_to_resamle:
    :return:
    """
    edited_data_cs = pd.read_csv(cs_data_filename, parse_dates=True, index_col='Время')
    if time_to_resamle is None:
        print('lol')
    else:
        edited_data_cs = edited_data_cs.resample(time_to_resamle).mean()
    edited_data_cs['Выходная частота ПЧ'] = edited_data_cs['Выходная частота ПЧ'].fillna(method='ffill')
    edited_data_cs['Температура на приеме насоса (пласт. жидкость)'] = \
        edited_data_cs['Температура на приеме насоса (пласт. жидкость)'].fillna(method='ffill')
    if created_input_data_type == 0:
        edited_data_cs = edited_data_cs.dropna(subset=['Объемный дебит жидкости'])
    edited_data_cs = edited_data_cs.fillna(method='ffill')
    edited_data_cs['ГФ'] = edited_data_cs['Объемный дебит газа'] / edited_data_cs['Объемный дебит нефти']
    edited_data_cs = mark_df_columns(edited_data_cs, 'СУ')

    return edited_data_cs


def get_dt_str_from_rus_date(rus_date: str) -> str:
    """
    convert date in russian to numerical like
    :param rus_date: string like '28 фев 2019'
    :return: string like '28/2/2019'
    """
    tokens = rus_date.split(' ')
    month_dict = dict(авг=8, сен=9, окт=10, ноя=11, дек=12, янв=1, фев=2)
    out = f'{tokens[0]}/{month_dict[tokens[1]]}/{tokens[2]}'
    return out



def load_and_edit_chess_data(chess_data_filename, time_to_resamle, without_changing=False):
    """
    Загрузка и обработка данных с шахматки
    :param without_changing:
    :param chess_data_filename:
    :param time_to_resamle:
    :return:
    """
    # reading data
    out = pd.read_excel(chess_data_filename)
    # 2 header rows
    if out.columns.values[0] != 'Дата':
        col_n_0 = out.T[0].values
        col_n_1 = out.T[1].values
        col_d = {}
        cols = out.columns.values
        # if 2nd is null, chose 1st nave for column
        for i in range(len(col_n_0)):
            if pd.isnull(col_n_1[i]):
                col_d[cols[i]] = col_n_0[i]
            else:
                col_d[cols[i]] = col_n_1[i]
        # renaming columns and dropping damn 2 columns
        out.drop([0, 1], inplace=True)
        out.rename(columns=col_d, inplace=True)
        # converting data
        out['Дата'] = out.apply(lambda x: get_dt_str_from_rus_date(x['Дата']), axis=1)
    # following with Oleg's code
    out.index = pd.to_datetime(out['Дата'], dayfirst=True, format="%d.%m.%Y", infer_datetime_format=True)
    del out['Дата']
    out.index.name = 'Время'
    out = out[out.columns[5:]]
    if without_changing:
        return out
    else:
        out = out.resample(time_to_resamle).last()
        out = out.fillna(method='ffill')
        out = mark_df_columns(out, 'Ш')
        return out


def extract_power_from_motor_name(name_str):
    name_str = name_str.upper()
    name_str = name_str.replace('9.8.4ЭДБТ ', '')
    name_str = name_str.replace(' ', '')
    name_str = name_str.replace('ПЭДНС', '')
    name_str = name_str.replace('9ЭДБТК', '')
    name_str = name_str.replace('ПЭДC', '')
    name_str = name_str.replace('ПЭДН', '')
    name_str = name_str.replace('9.8.4ЭДБТ', '')
    name_str = name_str.replace('ПВЭДН', '')
    name_str = name_str.replace('9ЭДБСТ', '')
    name_str = name_str.replace('9ЭДБТ', '')
    name_str = name_str.replace('ЭДБТ', '')
    name_str = name_str.replace('ПЭД', '')
    if name_str[0] == '-':
        name_str = name_str[1:]
    if name_str[3] == '-':
        name_str = name_str[0:3]
    elif name_str[2] == '-':
        name_str = name_str[0:2]
    return float(name_str)


class tr_data:
    def __init__(self, row):
        self.d_cas_mm = row[('D э/к', 'Unnamed: 9_level_1', 'Unnamed: 9_level_2', 'мм')].values[0]
        self.d_tube_mm = row[('D нкт', 'Unnamed: 10_level_1', 'Unnamed: 10_level_2', 'мм')].values[0]
        self.esp_nom_rate_m3day = \
        row[('Номинальная\nпроизводительность', 'Unnamed: 16_level_1', 'Unnamed: 16_level_2', 'м3/сут')].values[0]
        self.esp_nom_head_m = row[('Номинальный напор', 'Unnamed: 17_level_1', 'Unnamed: 17_level_2', 'м')].values[0]
        self.h_pump_m = row[('Н сп', 'Unnamed: 20_level_1', 'Unnamed: 20_level_2', 'м')].values[0]
        self.esp_name_str = \
        row[('Тип насоса', 'Unnamed: 15_level_1', 'Unnamed: 15_level_2', 'Unnamed: 15_level_3')].values[0]
        self.udl_m = row[('Удл (Нсп)', 'Unnamed: 161_level_1', 'Unnamed: 161_level_2', 'м')].values[0]
        self.i_motor_nom_a = row[('ПЭД', 'Unnamed: 131_level_1', 'I ном', 'А')].values[0]
        self.motor_name_str = row[('ПЭД', 'Unnamed: 128_level_1', 'Марка', 'Unnamed: 128_level_3')].values[0]
        self.power_motor_nom_kwt = extract_power_from_motor_name(
            row[('ПЭД', 'Unnamed: 128_level_1', 'Марка', 'Unnamed: 128_level_3')].values[0])


def read_tr_and_get_data(tr_file_full_path, well_name):
    tr = pd.read_excel(tr_file_full_path, skiprows=6,
                       header=[0, 1, 2, 3])  # при ошибке файл нужно открыть и сохранить повторно без изменений
    this_well_row = tr[tr[('№\nскв', 'Unnamed: 4_level_1', 'Unnamed: 4_level_2', 'Unnamed: 4_level_3')] == well_name]
    tr_class = tr_data(this_well_row)
    return tr_class


def del_inf_in_columns_name(df, well_name):
    columns = df.columns
    new_columns = []
    for i in columns:
        test_str = i
        index = test_str.find(well_name)
        str_to_delete = test_str[:index] + well_name + '. '
        new_str = i.replace(str_to_delete, "")
        new_columns.append(new_str)
    return new_columns
