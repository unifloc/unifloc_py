import os
import pandas as pd
from pathlib import Path


class GlobalNames():
    """
    Класс для хранения названий всех важных названий, а также приведения их к одному типу
    """
    def __init__(self):
        self.q_liq_m3day = 'Дебит жидкости, м3/сут'
        self.q_gas_m3day = 'Дебит газа, м3/сут'
        self.q_wat_m3day = 'Дебит воды, м3/сут'
        self.q_oil_m3day = 'Дебит нефти, м3/сут'
        self.q_oil_mass_tday = 'Дебит нефти массовый, т/сут'
        self.watercut_perc = 'Обводненность, %'
        self.gor_m3m3 = 'ГФ, м3/м3'

        self.p_buf_atm = 'Буферное давление, атм'
        self.p_lin_atm = 'Линейное давление, атм'
        self.p_intake_atm = 'Давление на приеме, атм'
        self.t_intake_c = 'Температура на приеме, С'
        self.t_motor_c = 'Температура двигателя, С'

        self.d_choke_mm = 'Диаметр штуцера, мм'

        self.cos_phi_d = 'Коэффициент мощности, д.ед.'
        self.u_motor_v = 'Напряжение на выходе ТМПН, В'
        self.u_ab_v = 'Напряжение AB, В'
        self.i_a_motor_a = 'Ток фазы А, А'
        self.motor_load_perc = 'Загрузка двигателя, %'
        self.freq_hz = 'Частота вращения, Гц'
        self.active_power_kwt = 'Активная мощность, кВт'
        self.full_power_kwt = 'Полная мощность, кВт'

        self.c_calibr_head_d = 'К. калибровки по напору - множитель, ед'
        self.c_calibr_power_d = 'К. калибровки по мощности - множитель, ед'

        self.chosen_q_liq_m3day = None
        self.chosen_watercut_perc = None
        self.chosen_gor_m3m3 = None

        self.chosen_p_buf_atm = None
        self.chosen_p_intake_atm = None
        self.chosen_t_intake_c = None

        self.chosen_d_choke_mm = None

        self.chosen_cos_phi_d = None
        self.chosen_u_motor_v = None
        self.chosen_motor_load_perc = None
        self.chosen_freq_hz = None
        self.chosen_active_power_kwt = None

    def return_dict_column_to_rename(self):
        columns_name_to_rename = {
            self.active_power_kwt: ["Активная мощность", "Активная мощность (ТМ)", 'Pa,кВт', 'акт.P,кВт', 'Pакт(кВт)'],
            self.full_power_kwt: ["Pполн,кВт", 'P, кВА', 'Pполн(кВA)'],
            self.p_lin_atm: ["Линейное давление", "Давление линейное (ТМ)"],

            self.p_intake_atm: ["Давление на приеме насоса (пласт. жидкость)",
                                        "Давление на входе ЭЦН (ТМ)",
                                        'P на приеме,ат',
                                        'P, ат.', 'P,atm', 'Pвх(МПа)'],
            self.t_intake_c: ["Температура насоса ЭЦН (ТМ)", "Температура на приеме насоса (пласт. жидкость)",
                                      "Тжид,Гр",
                                      'Tжид, °C', 'Твх(°С)'],

            self.t_motor_c: ["Температура двигателя ЭЦН (ТМ)", "ТПЭД,Гр", 'Tдвиг, °C',
                                     'Тобм(°С)'],

            self.motor_load_perc: ["Загрузка двигателя",
                                            "Загрузка ПЭД (ТМ)", "Загр,%", 'Загр., %', 'Загр., %',
                                           'Загрузка,%', 'Загр(%)'],
            self.u_ab_v: ["Входное напряжение АВ", "Напряжение AB (ТМ)", "UAB,В", 'Uвх.AB,В', 'Uab,В',
                                  'UвхAB(B)'],
            self.i_a_motor_a: ["Ток фазы А", "Ток фазы A (ТМ)", "Ia,А", 'Ia, A', 'Iа(A)'],
            self.freq_hz: ["Выходная частота ПЧ", "Частота вращения (ТМ)", "F,Гц", 'F, Гц', 'F(Гц)',
                                   'Коэффициент мощности'],

            self.cos_phi_d: ["Коэффициент мощности", "Коэффициент мощности (ТМ)", "Cos", 'cos',
                                        'Коэффициент мощности'],

            self.q_liq_m3day: ["Объемный дебит жидкости", "Дебит жидкости (ТМ)"],
            self.q_gas_m3day: ["Объемный дебит газа", "Дебит газа (ТМ)"],
            self.watercut_perc: ["Процент обводненности", "Обводненность (ТМ)"],
            self.q_oil_m3day: ["Объемный дебит нефти"],
            self.q_oil_mass_tday: ["Дебит нефти (ТМ)"]}

        return columns_name_to_rename

    def return_essential_parameters(self):
        essential_parameters_list = [self.q_liq_m3day,
                                     self.q_gas_m3day,
                                     self.q_wat_m3day,
                                     self.q_oil_m3day,
                                     self.q_oil_mass_tday,
                                     self.watercut_perc,
                                     self.gor_m3m3,
                                     self.p_buf_atm,
                                     self.p_lin_atm,
                                     self.p_intake_atm,
                                     self.t_intake_c,
                                     self.t_motor_c,
                                     self.cos_phi_d,
                                     self.u_motor_v,
                                     self.u_ab_v,
                                     self.i_a_motor_a,
                                     self.motor_load_perc,
                                     self.freq_hz,
                                     self.active_power_kwt]
        return essential_parameters_list

    def return_chosen_parameters(self):
        chosen_parameters_list = [self.chosen_q_liq_m3day,
                                  self.chosen_watercut_perc,
                                  self.chosen_gor_m3m3,
                                  self.chosen_p_buf_atm,
                                  self.chosen_p_intake_atm,
                                  self.chosen_t_intake_c,
                                  self.chosen_d_choke_mm,
                                  self.chosen_cos_phi_d,
                                  self.chosen_u_motor_v,
                                  self.chosen_motor_load_perc,
                                  self.chosen_freq_hz,
                                  self.chosen_active_power_kwt]
        return chosen_parameters_list


global_names = GlobalNames()


def rename_columns_by_dict(df, columns_name_dict = global_names.return_dict_column_to_rename()):
    """
    Специальное изменение названий столбцов по словарю
    :param df:
    :param dict:
    :return:
    """

    for i in df.columns:
        for items in columns_name_dict.items():
            if i in items[1]:
                df = df.rename(columns={i: items[0]})
    return df


def mark_df_columns(df, mark):
    """
    Пометка названий столбцов DataFrame c помошью (this_mark)
    :param df: исходный DataFrame
    :param mark: str, который будет приписан к названию столбца (например, СУ, или Ш)
    :return: DataFrame с новыми названиями столбцов
    """
    for i in df.columns:
        df = df.rename(columns={i: i + ' (' + mark + ')'})
    return df


def combine_multiprocessing_result(path_to_work_dir, dir_name_with_calculated_data): #TODO избавиться от путей
    """
    Объединение результатов расчета в один файл
    :param path_to_work_dir: абсолютный путь к рабочей директории
    :param dir_name_with_calculated_data: путь к данным скважины
    :return: объединенный файл результатов расчета в DataFrame
    """
    filenames_list = []
    for (dirpath, dirnames, filenames) in os.walk(path_to_work_dir + dir_name_with_calculated_data + 'multiprocessing\\'):
        filenames_list.extend(filenames)
        break
    print(filenames_list)
    for i, j in enumerate(filenames_list):
        if i == 0:
            first_result_data = pd.read_csv(path_to_work_dir + dir_name_with_calculated_data + 'multiprocessing\\' + j,
                                            parse_dates=True, index_col='Время')
        else:
            another_result_data = pd.read_csv(path_to_work_dir + dir_name_with_calculated_data + 'multiprocessing\\' + j,
                                              parse_dates=True, index_col='Время')
            first_result_data = first_result_data.append(another_result_data, sort=True)
            del first_result_data['d']
            first_result_data = first_result_data.dropna(subset=['ESP.ESPpump.EffiencyESP_d'])
    first_result_data = first_result_data.sort_index()
    return first_result_data


def cut_df(df, left_boundary, right_boundary):
    """
    Вырезка из DataFrame временного интервала
    :param df:
    :param left_boundary:
    :param right_boundary:
    :return:
    """
    df_list = []
    for start, end in zip(left_boundary, right_boundary):
        this_df = df[(df.index >= start) & (df.index <= end)]
        df_list.append(this_df)
    if len(df_list) == 1:
        return df_list[0]
    else:
        for number, this_df in enumerate(df_list):
            if number == 0:
                df = this_df
            else:
                df = df.append(this_df)
        return df


def make_gaps_and_interpolate(df, reverse=False):
    """
    Выкалывание точек и линейная интерполяция. (Восстановление дебитов путем интерполяции)
    :param df: исходный DataFrame
    :param reverse: выкалывается каждая вторая точка начиная с первой
    :return: DataFrame с выколотыми вторыми точками и проинтерполированными значениями, чтобы заполнить выколотый точки
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


def filtr_data_by_min_value(df, column_name, min_value):
    init_amount_rows = df.shape[0]
    df = df[df[column_name] > min_value]
    new_amount_rows = df.shape[0]
    print('Отфильтровано по параметру: ' + column_name +
          f' c минимальным значением {min_value}. Отброшено значений {init_amount_rows - new_amount_rows}')
    return df


def filtr_data_by_drop_nan(df: pd.DataFrame, column_name):
    init_amount_rows = df.shape[0]
    df = df.dropna(subset=[column_name])
    new_amount_rows = df.shape[0]
    print('Отфильтровано по параметру: ' + column_name +
          f' по NaN. Отброшено значений {init_amount_rows - new_amount_rows}')
    return df


def check_input_data(df):
    """
    Проверка входных данных модели, выбрасывания неполных строк данных или наборов данных, при которых UniflocVBA падает
    :param df: исходный DataFrame
    :return: DataFrame с отфильтрованными данными
    """
    df = filtr_data_by_min_value(df, global_names.freq_hz, 30)
    df = filtr_data_by_min_value(df, global_names.q_liq_m3day, 10)
    df = filtr_data_by_min_value(df, global_names.p_intake_atm, 1)
    df = filtr_data_by_drop_nan(df, global_names.p_buf_atm)
    return df


def find_full_path_by_pattern(initial_dir, pattern, additional_pattern_string=None):
    full_path_list = []
    print('Найденные пути по основному паттерну:')
    for filename in Path(initial_dir).rglob(pattern):
        full_path_list.append(str(filename))
        print(filename)
    if additional_pattern_string != None:
        specific_full_path_list = []
        for i in full_path_list:
            if additional_pattern_string in i:
                specific_full_path_list.append(i)
        print('Найденные пути по вспомогательному паттерну:')
        print(specific_full_path_list)
        return specific_full_path_list
    else:
        return full_path_list


def find_column_in_df_columns(df, pattern):
    """
    Функция для поиска названий столбцов по шаблону
    :param df: DataFrame, в котором ведется поиск
    :param pattern: шаблон, например обв, или ШТР
    :return:
    """
    found_columns = []
    for i in df.columns:
        if pattern in i:
            found_columns.append(i)
    return found_columns


def solve_dimensions(df: pd.DataFrame, global_names=global_names):
    if global_names.p_lin_atm in df.columns:
        p_lin_series = df[global_names.p_lin_atm]
        p_lin_series = p_lin_series.dropna()
        if len(p_lin_series) != 0:
            first_value = p_lin_series[0]
            if first_value < 5:
                print(f'Приведение размерностей. Давление в МПа, т.к первое значение равно = {first_value},'
                      f' колонка {global_names.p_lin_atm} будет умножена на 10')
                df[global_names.p_lin_atm] = df[global_names.p_lin_atm] * 10
            else:
                print(f'Приведение размерностей. Давление в МПа, т.к первое значение равно = {first_value},'
                      f' колонка {global_names.p_lin_atm} не будет умножена на 10')
    else:
        print(f"Приведение размерностей. Нет столбца с именем: {global_names.p_lin_atm}")
    return df
