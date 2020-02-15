import pandas as pd
import numpy as np
import sys
import os
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool
import scipy

def get_filtred_by_sigma(df: pd.DataFrame, column_name='Объемный дебит жидкости (СУ)', lower_sigma=2,
                         upper_sigma=3): #TODO сделать возможность замены значений в колонке на None, а не дропа строк
    """

    :param df:
    :param column_name: названия столбца, по которому будет фильтрация
    :param lower_sigma: количество нормальных отклонений для нижней границе, по которой мы отсеиваем выбросы.
     По идее должна быть меньшн чем верхняя
    :param upper_sigma: верхняя граница разумных значений в терминах отклонения по нормальному распределению.
    :return:
    """
    init_amount_of_row = df.shape[0]
    m = df[column_name].mean()
    sigma = df[column_name].values.std()
    df = df[df[column_name] <= m + upper_sigma * sigma]
    df = df[df[column_name] >= m - lower_sigma * sigma]
    amount_of_filtered_rows = init_amount_of_row - df.shape[0]
    print(f"Произведена фильтрация по стандартному отклонению (верхнее: {upper_sigma}, нижнее: {lower_sigma}, удалено "
          f"строк: {amount_of_filtered_rows}")
    return df


def get_filtred_by_measurng_time(input_data: pd.DataFrame, first_edit_data:pd.DataFrame, critical_difference=30): #TODO переделать или выкинуть
    """
    Это функция по даным, после первичной обработки отбросит явно лишние данные, которые неверно измерили: плновое
    время замера отличается от фактичекого
    :param input_data: входные данные, котрые нужно отфильтровать
    :param first_edit_data: данные после первичной обработки. Можно подать только кусок с колонками "Время" и
     "Время замера фактическое"
    :param critical_difference: разница во времени замера фактическом и плановом. Рассматриваем, только если
     замеряли меньше
    :return: отфильтрованные данные
    """

    # сделаем копию данных, чтобы не менять вход
    loc_cs = first_edit_data.copy()
    loc_cs = loc_cs[['Время замера фактическое', 'Время']].dropna().set_index('Время')
    loc_cs.index = pd.to_datetime(loc_cs.index)
    loc_cs = loc_cs.resample('3h').mean()
    # объединяем данные, чтобы затем отбросить
    out = input_data.join(loc_cs)
    out['Время замера фактическое'].fillna(np.inf, inplace=True)
    # избавляемся от некорректных замеров
    out = out[
        out['Время замера фактическое'].dropna() > critical_difference]
    # чистим колонку, взятую из данных после первичной обработки
    out.drop(columns=['Время замера фактическое'])
    return out


def check_medfit(df, column_name, items, plot=True):
    """
    Использование медианного фильтра к колонке dataframe
    :param df: исходный зашумленный df
    :param column_name: имя колонки для фильтрации
    :param items: количество точек для окна медианного фильтра
    :param plot: построение графиков
    :return: df с колонками column_name + ' (real)' и + ' (median)'
    """
    real = df[column_name].dropna()
    median = scipy.signal.medfilt(df[column_name].dropna().values, items)
    df = pd.DataFrame({column_name + ' (real)': real,
                      column_name + ' (median)': median})
    if plot:
        df.plot()
    return df