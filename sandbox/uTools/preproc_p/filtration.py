import pandas as pd
import numpy as np
import sys
import os
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool


def get_filtred_by_sigma(input_data: pd.DataFrame, lower_sigma=2, upper_sigma=3, inplace=False):
    """
    Обязательно, чтобы в данном фрэйме был столбец 'Объемный дебит жидкости (СУ)'
    :param input_data:
    :param lower_sigma: количество нормальных отклонений для нижней границе, по которой мы отсеиваем выбросы.
     По идее должна быть меньшн чем верхняя
    :param upper_sigma: верхняя граница разумных значений в терминах отклонения по нормальному распределению.
    :param inplace: если мы не хотим засорять память и не возвращать df, а на месте его исправить
    :return:
    """
    m = input_data['Объемный дебит жидкости (СУ)'].mean()
    sigma = input_data['Объемный дебит жидкости (СУ)'].values.std()
    if not inplace:
        out = input_data.copy()
        out = out[out['Объемный дебит жидкости (СУ)'] <= m + upper_sigma * sigma]
        out = out[out['Объемный дебит жидкости (СУ)'] >= m - lower_sigma * sigma]
        return out
    else:
        input_data = input_data[input_data['Объемный дебит жидкости (СУ)'] <= m + upper_sigma * sigma]
        input_data = input_data[input_data['Объемный дебит жидкости (СУ)'] >= m - lower_sigma * sigma]
        return 0


def get_filtred_by_measurng_time(input_data: pd.DataFrame, first_edit_data:pd.DataFrame, critical_difference=30):
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
        out['Время замера фактическое'].dropna() - out['Время замера плановое (СУ)'] - 30 > 0]
    # чистим колонку, взятую из данных после первичной обработки
    out.drop(columns=['Время замера фактическое'])
    return out