"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль для построения графиков через plotly

"""

import pandas as pd
import numpy as np
import sys
sys.path.append('../')
from _plotly_future_ import v4_subplots
import plotly.graph_objs as go
from plotly.subplots import make_subplots
from plotly.offline import download_plotlyjs, plot, iplot
from plotly import tools

from datetime import date


def create_plotly_trace(data_x, data_y, namexy, chosen_mode='lines', low_memory = True):
    """
    Создание одного trace по данным

    :param data_x: данные для оси x
    :param data_y:  данные для оси y
    :param namexy: название для trace
    :param chosen_mode: настройка отображения 'lines', 'markers'
    :return: один trace
    """
    if low_memory == True:
        one_trace = go.Scattergl(
			x=data_x,
			y=data_y,
			name=namexy,
			mode=chosen_mode,
            hovertemplate = namexy + "<br>Значение: %{y:.3f}<extra></extra>"
		)
    else:
        one_trace = go.Scatter(
            x=data_x,
            y=data_y,
            name=namexy,
            mode=chosen_mode
        )
    return one_trace


def plot_func(data, plot_title_str, filename_str):
    """
    Итоговая функция для построения графиков

    :param data: созданный список из trace
    :param plot_title_str: название графика
    :param filename_str: названия html файлика
    :return: None
    """
    layout = dict(title=plot_title_str)

    fig = dict(data=data, layout=layout)

    # fig = make_subplots(rows=8, cols=1)

    plot(fig, filename=filename_str)
    # iplot(fig)


def plot_subplots(data_traces, filename_str, two_equal_subplots=False):
    """
    Построение нескольких графиков

    :param data_traces: подготовленный список trace
    :param filename_str: имя файла .html
    :param two_equal_subplots: если True график будет разделен на 2 одинаковых друг по другом, если False - все в колонку
    :return: None
    """
    if two_equal_subplots:
        items_in_one_subplot = int(len(data_traces))
        fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.02)
        for i in range(items_in_one_subplot):
            fig.append_trace(data_traces[i], row=1, col=1)
            fig.append_trace(data_traces[i], row=2, col=1)
    else:
        fig = make_subplots(rows=len(data_traces), cols=1, shared_xaxes=True, vertical_spacing=0.02)
        for i in range(len(data_traces)):
            fig.append_trace(data_traces[i], row=i + 1, col=1)

    plot(fig, filename=filename_str)


"""def create_traces_list_by_num(data_x_values, data_y, num_y_list):
    trace_list = []
    for i in num_y_list:
        namexy = data_y.get_saved_parameter_name_by_number(i)
        this_trace = create_plotly_trace(data_x_values, data_y.get_saved_values_by_number(i), namexy)
        trace_list.append(this_trace)
    return trace_list"""


def create_traces_list_for_all_columms(data_frame, chosen_mode='lines', low_memory = False):
    """
    Создание списка из trace для данного DataFrame для передачи их в data и последующего строительства графика.

    :param data_frame: подготовленный Pandas DataFrame с названиями колонок и обработанным индексом
    :param chosen_mode: выбор отображения 'lines', 'markers' и т.п.
    :return: trace_list для data
    """
    trace_list = []
    columns_name_list = data_frame.columns
    for i in columns_name_list:
        column_name = i
        this_series = data_frame[column_name].dropna()
        this_trace = create_plotly_trace(this_series.index, this_series, column_name, chosen_mode, low_memory)
        trace_list.append(this_trace)
    return trace_list


def connect_traces(traces1, trace2):
    """
    Создание единого списка trace из двух. Удобно при построении графиков из разных DataFrame

    :param traces1: первый список с trace
    :param trace2: второй список с trace
    :return: объединенный вариант
    """
    connected_traces = []
    for i in traces1:
        connected_traces.append(i)
    for j in trace2:
        connected_traces.append(j)
    return connected_traces


def create_report_html(df, all_banches, filename):
    subplot_amount = len(all_banches)
    subplot_titles = []
    for z in all_banches:
        subplot_titles.append(list(z.keys())[0])
    fig = make_subplots(
        rows=subplot_amount, cols=1, shared_xaxes=True,
        vertical_spacing=0.01,
        subplot_titles=subplot_titles
    )
    for i in range(subplot_amount):
        this_df = df[all_banches[i][subplot_titles[i]]]
        this_banch_trace = create_traces_list_for_all_columms(this_df, chosen_mode='lines+markers', low_memory=True)
        for j in this_banch_trace:
            fig.add_trace(j, row=i + 1, col=1)

    fig.layout.hovermode = 'x'
    fig.layout.height = 450 * subplot_amount
    plot(fig, filename=filename)
