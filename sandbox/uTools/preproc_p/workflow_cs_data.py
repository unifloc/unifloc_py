import pandas as pd
import sys
import os

sys.path.append('../' * 4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

global_names = preproc_tool.GlobalNames()


def initial_editing(df, wellname):
    if len(df.columns) == 4:
        del df[0]
    df.columns = [0, 1, 2]
    test_str = df[0][0]
    index = test_str.find(wellname)
    if index != -1:
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


def read_and_format_good_tm_data(well_name, file_name_full_path, time_to_resamle='3h'):
    """
    Загрузка данных в стандартном формате ГРАДА и его вариациях, преобразования его в шахмоткоподобный вид
    :param well_name: название скважины
    :param file_name_full_path: полный путь к файлу .csv с данными
    :param time_to_resamle: время осреднения - ресемпла - '3h', '1d', будет произведено при недостатке памяти
    :return:  DataFrame в шахмоткоподобном виде
    """
    try:
        try:
            well_data = pd.read_csv(file_name_full_path, sep=';', header=None, skiprows=1)
        except:
            well_data = pd.read_csv(file_name_full_path, sep='\\t', header=None)
            if well_data[0][0] == 'Параметр;Дата;Значение':
                well_data = pd.read_csv(file_name_full_path, sep=';', skipfooter=1)
        if well_data[0][0].split(';')[0] == 'Месторождение':  # Формат выгрузки града
            well_data = pd.read_csv(file_name_full_path, delimiter=';', engine='python')
            well_data.reset_index(inplace=True)
            if 'index' in well_data.columns:
                del well_data['index']
                columns_to_del = ['Месторождение', 'Скважина', 'Скважина ОИС', 'Дата Общая']
            else:
                well_data.columns = list(well_data.columns[1::].values) + ['to_del']
                columns_to_del = ['Месторождение', 'Скважина', 'Скважина ОИС', 'Дата Общая', 'to_del']
            for i in columns_to_del:
                del well_data[i]
            well_data = initial_editing(well_data, well_name)
            well_data = create_edited_df(well_data)
        else:
            well_data = initial_editing(well_data, well_name)
            well_data = create_edited_df(well_data)
    except:
        well_data = pd.read_csv(file_name_full_path, sep='\\t', skiprows=1, skipfooter=1, index_col='Названия строк')
        well_data.index.name = 'Время'
        well_data.index = pd.to_datetime(well_data.index)
        del well_data['Общий итог']
        well_data.columns = del_inf_in_columns_name(well_data, well_name)
    if well_data.memory_usage().sum() / 1024 / 1024 > 1000:
        print('Потребление памяти слишком велико, произведем ресемпл')
        well_data = well_data = well_data.resample(
            time_to_resamle).mean()  # TODO сначала нужна фильтрация, затем ресемпл
    return well_data


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
        try:
            edited_data_cs = edited_data_cs.dropna(subset=['Объемный дебит жидкости'])
            print('Произведена фильтрация по None по Объемному дебиту жидкости')
        except:pass
    edited_data_cs = edited_data_cs.fillna(method='ffill')
    try:
        edited_data_cs['ГФ'] = edited_data_cs['Объемный дебит газа'] / edited_data_cs[
            'Объемный дебит нефти']  # TODO технический долг - когда дебит по нефти равен нулю - inf - ошибка в адаптации или восстановлении калибровок
        # TODO надо избавляться от inf - возможно переключение расчета на газовую скважину, или 0
        edited_data_cs = edited_data_cs[edited_data_cs['Объемный дебит нефти'] != 0]  # TODO технический долг убрать костыль
    except:
        pass
    edited_data_cs = preproc_tool.mark_df_columns(edited_data_cs, 'СУ')

    return edited_data_cs


def drop_string_columns(df):
    init_amount_of_rows = df.shape[1]
    columns_list = df.columns
    for i in columns_list:
        if type(df[i][0]) == str:
            df = df.drop(columns=i)
    deleted_rows = init_amount_of_rows - df.shape[1]
    print(f"Удалено столбцов, в которых есть текст вместо параметров (пометки СУ): {deleted_rows}")
    return df


def replace_string_by_nan(value):
    try:
        float_value = float(value)
        return float_value
    except:
        return None


def delete_string_axis(df: pd.DataFrame, column_name):
    df[column_name] = df[column_name].apply(replace_string_by_nan)
    df = df.dropna(subset=[column_name])
    return df


def clear_df_from_string(df: pd.DataFrame):
    column_names = df.columns
    delete_all = False
    for i in column_names:
        if i in df.columns and not delete_all:
            df = delete_string_axis(df, i)
            delete_all = True
    return df


def read_and_format_bad_tm_data(filename_full_path):
    """
    Чтение данных телеметрии (высокочастотные данные со СУ) разных новых форматов
    :param filename_full_path: полный путь к файлу, либо просто название, если находится в той же директории
    :return:
    """
    print(f"Чтение файла{filename_full_path}")
    flash_data = pd.read_excel(filename_full_path, header=None)
    if flash_data[0][0] == 'Основная страница':
        print('Тип данных: Зеленый Борец')
        loaded_file = pd.read_excel(filename_full_path, skiprows=4, index_col='Дата/Время', parse_dates=True, dayfirst=True)
        loaded_file = loaded_file.replace(to_replace='###', value=None)
        loaded_file = loaded_file[loaded_file['   Состояние   '] == 'Работа']

    elif flash_data[0][0] == 'Дата, Время':
        print('Тип данных: Почти нормальный тип')
        loaded_file = pd.read_excel(filename_full_path, index_col='Дата, Время', parse_dates=True)
        loaded_file = loaded_file.replace(to_replace='-----', value=None)
        loaded_file = loaded_file.dropna(subset=['P, кВА'])

    elif flash_data[0][2] == 'Nп/п':
        print('Тип данных: Борец энергетика')
        loaded_file = pd.read_excel(filename_full_path, skiprows=2, index_col='Дата        Время', parse_dates=True)

    elif flash_data[1][1] == 'Информация:':
        print('Тип данных: Зеленый Борец с несколькими листами')
        loaded_file = pd.read_excel(filename_full_path, sheet_name='Журнал', skiprows=4, index_col='Дата/Время', parse_dates=True)
        loaded_file = loaded_file.replace(to_replace='###', value=None)
        loaded_file[(loaded_file['   Состояние   '] == 'ПИД реж.') | (loaded_file['   Состояние   '] == 'работа')]
    elif flash_data[0][0] == 'ID':
        print('Тип данных: почти нормальный с большой точностью')
        loaded_file = pd.read_excel(filename_full_path, index_col='Дата время', parse_dates=True)
        loaded_file = loaded_file.dropna(subset=['№Скв'])
    else:
        print('Тип данных: тип не распознан')

    loaded_file = drop_string_columns(loaded_file)
    loaded_file = clear_df_from_string(loaded_file)
    loaded_file.index.name = 'Время'
    return loaded_file


def find_parameters_in_df_columns(df, pattern):
    """
    Поиск параметров в столбцах DataFrame
    :param df: pd.DataFrame
    :param pattern: str, например обв или ТМ
    :return: список параметров (названий столбцов)
    """
    parameters_list = []
    for i in df.columns:
        if pattern in i:
            parameters_list.append(i)
    return parameters_list

