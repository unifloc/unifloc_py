import os
import pandas as pd


def mark_df_columns(df, mark):
    for i in df.columns:
        df = df.rename(columns={i: i + ' (' + mark + ')'})
    return df


def combine_multiprocessing_result(path_to_work_dir, dir_name_with_calculated_data):
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

def check_input_data(df):
    init_amount_rows = df.shape[0]
    df = df[df['F вращ ТМ (Ш)'] > 30]
    amount_rows = df.shape[0]
    print('Отфильтровано по F вращ ТМ (Ш): ' + str(init_amount_rows - amount_rows))
    df = df[df['Объемный дебит жидкости (СУ)'] > 10]
    new_amount_rows = df.shape[0]
    print('Отфильтровано по Объемный дебит жидкости (СУ): ' + str(amount_rows - new_amount_rows))
    df = df[df['Давление на приеме насоса (пласт. жидкость) (СУ)'] > 1]
    new2_amount_rows = df.shape[0]
    print('Отфильтровано по Давление на приеме насоса (пласт. жидкость) (СУ): ' + str(new_amount_rows - new2_amount_rows))
    return df