import sys
import os
current_path = os.getcwd()
path_to_vba = current_path.replace(r'unifloc\sandbox\uTools', '')
sys.path.append(path_to_vba)
import pandas as pd

import unifloc_vba.description_generated.python_api as python_api

def mark_df_columns(df, mark):
    for i in df.columns:
        df = df.rename(columns={i: i + ' (' + mark + ')' })
    return df

def initial_editing(df, wellname):
    if len(df.columns)==4:
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
    extracted_df_one_param = extracted_df_one_param.rename(columns = {2: extracted_df_one_param[0][0]})
    del extracted_df_one_param[0]
    return extracted_df_one_param

def create_edited_df(df):
    parametrs_list = df[0].unique()
    init_one_parametr_df = extract_df_one_parametr_and_edit(df, parametrs_list, 0)
    result = init_one_parametr_df
    for i in range(1, len(parametrs_list)):
        new_one_parametr_df = extract_df_one_parametr_and_edit(df, parametrs_list, i)
        result = result.join(new_one_parametr_df, how = "outer", sort=True)
    return result

def cut_df(df, left_boundary, right_boundary):
    for start, end in zip(left_boundary, right_boundary):
        df = df[(df.index >= start) & (df.index <= end)]
    return df
