"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль для построения графиков через plotly

"""
# TODO сделать через классы
# TODO добавить оси и форматирование
# TODO сделать аналогично для dash и matplotlib
import pandas as pd
import numpy as np
import sys
sys.path.append('../')
import plotly.graph_objs as go
from plotly.subplots import make_subplots
from plotly.offline import plot



def create_plotly_trace(data_x, data_y, namexy, chosen_mode='lines', use_gl = True):
    """
    Создание одного trace по данным

    :param data_x: данные для оси x
    :param data_y:  данные для оси y
    :param namexy: название для trace
    :param chosen_mode: настройка отображения 'lines', 'markers'
    :return: один trace
    """
    if use_gl == True:
        one_trace = go.Scattergl(
			x=data_x,
			y=data_y,
			name=namexy,
			mode=chosen_mode,
            hovertemplate = namexy + ": %{y}<extra></extra>"
		)
    else:
        one_trace = go.Scatter(
            x=data_x,
            y=data_y,
            name=namexy,
            mode=chosen_mode,
            hovertemplate=namexy + ": %{y:.3f}<extra></extra>"
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


def plot_subplots(data_traces, filename_str, two_equal_subplots=False, auto_open = True):
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
    fig.layout.hovermode = 'x'
    plot(fig, filename=filename_str, auto_open=auto_open)


def create_traces_list_for_all_columms(data_frame, chosen_mode='lines', use_gl=True):
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
        this_trace = create_plotly_trace(this_series.index, this_series, column_name, chosen_mode, use_gl)
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
