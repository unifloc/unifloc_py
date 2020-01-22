import os
import pandas as pd

from pathlib import Path

def rename_columns_by_dict(df, dict):
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
    df = filtr_data_by_min_value(df, 'F вращ ТМ (Ш)', 30)
    df = filtr_data_by_min_value(df, 'Объемный дебит жидкости (СУ)', 10)
    df = filtr_data_by_min_value(df, 'Давление на приеме насоса (пласт. жидкость) (СУ)', 1)
    df = filtr_data_by_drop_nan(df, 'Рбуф (Ш)')
    return df


def find_full_path_by_pattern(initial_dir, pattern):
    full_path_list = []
    for filename in Path(initial_dir).rglob(pattern):
        full_path_list.append(str(filename))
        print(filename)
    return full_path_list
