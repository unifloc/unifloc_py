import pandas as pd


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


def read_and_edit_init_cs_data(well_name, path_to_work_dir):
    data_file_path = path_to_work_dir + well_name + ".csv"
    try:
        try:
            well_data = pd.read_csv(data_file_path,sep=';', header=None, skiprows = 1)
        except:
            well_data = pd.read_csv(data_file_path,sep='\\t', header=None)
        well_data = initial_editing(well_data, well_name)
        well_data = create_edited_df(well_data)
    except:
        data_file_path = path_to_work_dir + well_name + ".csv"
        well_data = pd.read_csv(data_file_path, sep='\\t', skiprows = 1, skipfooter =1, index_col = 'Названия строк')
        well_data.index.name = 'Время'
        well_data.index = pd.to_datetime(well_data.index)
        del well_data['Общий итог']
        well_data.columns = del_inf_in_columns_name(well_data, well_name)
    return  well_data