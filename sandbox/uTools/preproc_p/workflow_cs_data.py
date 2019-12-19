import pandas as pd
#from sandbox.uTools.preproc_p import preproc_tool
import sys
import os
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

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


def read_and_edit_init_cs_data(well_name, path_to_work_dir, time_to_resamle = '3h'):
    data_file_path = path_to_work_dir + well_name + ".csv"
    try:
        try:
            well_data = pd.read_csv(data_file_path,sep=';', header=None, skiprows = 1)
        except:
            well_data = pd.read_csv(data_file_path,sep='\\t', header=None)
            if well_data[0][0] == 'Параметр;Дата;Значение':
                well_data = pd.read_csv(data_file_path, sep=';', skipfooter=1)
        well_data = initial_editing(well_data, well_name)
        well_data = create_edited_df(well_data)
    except:
        data_file_path = path_to_work_dir + well_name + ".csv"
        well_data = pd.read_csv(data_file_path, sep='\\t', skiprows=1, skipfooter=1, index_col='Названия строк')
        well_data.index.name = 'Время'
        well_data.index = pd.to_datetime(well_data.index)
        del well_data['Общий итог']
        well_data.columns = del_inf_in_columns_name(well_data, well_name)
    if well_data.memory_usage().sum()/1024/1024 > 1000:
        print('Потребление памяти слишком велико, произведем ресемпл')
        well_data = well_data = well_data.resample('3h').mean()
    return  well_data


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
    edited_data_cs['ГФ'] = edited_data_cs['Объемный дебит газа'] / edited_data_cs['Объемный дебит нефти']  #TODO технический долг - когда дебит по нефти равен нулю - inf - ошибка в адаптации или восстановлении калибровок
    # TODO надо избавляться от inf - возможно переключение расчета на газовую скважину, или 0
    edited_data_cs = edited_data_cs[edited_data_cs['Объемный дебит нефти'] != 0]  #TODO технический долг убрать костыль
    edited_data_cs = preproc_tool.mark_df_columns(edited_data_cs, 'СУ')

    return edited_data_cs
