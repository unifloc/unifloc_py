import pandas as pd
#from sandbox.uTools.preproc_p import preproc_tool
import sys
import os
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

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
        out = preproc_tool.mark_df_columns(out, 'Ш')
        return out

