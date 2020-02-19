import pandas as pd
import numpy as np
import sys
import os
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool
import scipy


def carefull_filtr_data(value, min_border, max_border):
    """
    Функция для фильтрации данных - фильтруемое значение заменяется на None
    :param value: значение
    :param min_border: верхняя граница включительно
    :param max_border: нижняя граница включительно
    :return: value если входит в интервал, None если не входит
    """
    if min_border <= value <= max_border:
        return value
    else:
        return None


def get_filtred_by_sigma(df: pd.DataFrame, column_name, lower_sigma=2,
                         upper_sigma=3):
    """

    :param df:
    :param column_name: названия столбца, по которому будет фильтрация
    :param lower_sigma: количество нормальных отклонений для нижней границе, по которой мы отсеиваем выбросы.
     По идее должна быть меньшн чем верхняя
    :param upper_sigma: верхняя граница разумных значений в терминах отклонения по нормальному распределению.
    :return:
    """
    init_amount_of_row = len(df[column_name].dropna())
    m = df[column_name].dropna().mean()
    sigma = df[column_name].dropna().values.std()
    max_border = m + upper_sigma * sigma
    min_border = m - lower_sigma * sigma
    df[column_name] = df[column_name].apply(carefull_filtr_data, args=[min_border, max_border])
    amount_of_filtered_rows = init_amount_of_row - len(df[column_name].dropna())

    print(f"Произведена фильтрация по стандартному отклонению для колонки {column_name}"
          f" Границы: верхняя: {max_border} ({upper_sigma}), "
          f"нижняя: {min_border} ({lower_sigma}), заменено "
          f"значенией на None: {amount_of_filtered_rows}")
    return df


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