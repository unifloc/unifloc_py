"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль для построения графиков через plotly

"""

import pandas as pd
import numpy as np
import sys
sys.path.append('../')
import plotly.graph_objs as go
from plotly.subplots import make_subplots
from plotly.offline import plot, iplot
import re


def create_plotly_trace(data_x, data_y, namexy, chosen_mode='lines', use_gl = True, swap_xy = False):
    """
    Создание одного trace по данным

    :param data_x: данные для оси x
    :param data_y:  данные для оси y
    :param namexy: название для trace
    :param chosen_mode: настройка отображения 'lines', 'markers'
    :return: один trace
    """
    if swap_xy:
        data_x, data_y = data_y, data_x
        hovertemplate = namexy + ": %{x}<extra></extra>"
    else:
        hovertemplate = namexy + ": %{y}<extra></extra>"
    if use_gl == True:
        one_trace = go.Scattergl(
			x=data_x,
			y=data_y,
			name=namexy,
			mode=chosen_mode,
            hovertemplate=hovertemplate
		)
    else:
        one_trace = go.Scatter(
            x=data_x,
            y=data_y,
            name=namexy,
            mode=chosen_mode,
            hovertemplate=hovertemplate
        )
    return one_trace


def plot_func(data, plot_title_str, filename_str, reversed_y=False, iplot_option=False, x_name=None, y_name=None):
    """
    Итоговая функция для построения графиков

    :param reversed_y:
    :param data: созданный список из trace
    :param plot_title_str: название графика
    :param filename_str: названия html файлика
    :return: None
    """
    if reversed_y:
        layout = dict(title=plot_title_str, yaxis=dict(autorange='reversed'), hovermode='x')
    else:
        layout = dict(title=plot_title_str)
    if x_name != None:
        layout['xaxis_title'] = x_name
    if y_name != None:
        layout['yaxis_title'] = y_name
    fig = dict(data=data, layout=layout)
    if iplot_option:
        iplot(fig, filename=filename_str)
    else:
        plot(fig, filename=filename_str)


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



def create_traces_list_for_all_columms(data_frame, chosen_mode='lines', use_gl=True, swap_xy=False, traces_names=None):
    """
    Создание списка из trace для данного DataFrame для передачи их в data и последующего строительства графика.

    :param data_frame: подготовленный Pandas DataFrame с названиями колонок и обработанным индексом
    :param chosen_mode: выбор отображения 'lines', 'markers' и т.п.
    :return: trace_list для data
    """
    trace_list = []
    columns_name_list = data_frame.columns
    if traces_names != None and len(traces_names) == len(columns_name_list):
        for i, j in zip(columns_name_list, traces_names):
            column_name = i
            this_series = data_frame[column_name].dropna()
            this_trace = create_plotly_trace(this_series.index, this_series, j, chosen_mode, use_gl, swap_xy)
            trace_list.append(this_trace)
    else:
        for i in columns_name_list:
            column_name = i
            this_series = data_frame[column_name].dropna()
            this_trace = create_plotly_trace(this_series.index, this_series, column_name, chosen_mode, use_gl, swap_xy)
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


def find_by_patterns(patterns, list_to_search):
    res = [x for x in list_to_search if re.search(patterns[0], x)]
    if len(patterns) >1:
        for i in patterns[1:]:
            res = [x for x in res if re.search(i, x)]
    return res


def plot_specific_columns(result_df, columns_to_plot=None, swap_xy=True, reversed_y=True, iplot_option=True,
                          plot_name='this_plot', x_name=None, y_name=None, traces_names=None):
    """
    Функция для быстрого построения графиков, только для определенных колонок DataFrame
    :param result_df:
    :param columns_to_plot:
    :param swap_xy:
    :param reversed_y:
    :param iplot_option:
    :param plot_name:
    :return:
    """
    if columns_to_plot == None:
        columns_to_plot = result_df.columns
    result_df_to_plot = result_df[columns_to_plot]
    all_traces = create_traces_list_for_all_columms(result_df_to_plot, 'lines+markers', swap_xy=swap_xy, traces_names=traces_names)
    plot_func(all_traces, plot_name, f'{plot_name}.html', reversed_y=reversed_y, iplot_option= iplot_option, x_name=x_name, y_name=y_name)


def filtr_by_antipatterns(init_list: list, antipatterns: list, print_all: bool = True):
    """
    Фильтрация списка параметров по антипаттернам, удаления нежелательных элементов типа string
    :param print_all: опция - выводить все удаленные совпадения по антипаттерну
    :param init_list:
    :param antipatterns:
    :return:
    """
    new_list = init_list.copy()
    droped_values = []
    for j in antipatterns:
        new_list = [i for i in new_list if j not in i ]
    for i in init_list:
        if i not in new_list:
            droped_values.append(i)
    if print_all:
        print(f"Удаленные совпадения по антипаттерну: {droped_values}")
    return new_list


def create_columns_to_plot(result_df, group_patterns, antipatterns=[], print_all=False):
    if type(group_patterns[0]) == str:
        columns_to_plot = find_by_patterns(group_patterns, result_df.columns)
    else:
        columns_to_plot = []
        for i in group_patterns:
            this_column_to_plot = find_by_patterns(i, result_df.columns)
            columns_to_plot += this_column_to_plot
    if print_all:
        print(f"Найденные совпадения: {columns_to_plot}")
    if len(antipatterns) > 0:
        columns_to_plot = filtr_by_antipatterns(columns_to_plot, antipatterns, print_all=print_all)
    return columns_to_plot

def plot_by_patterns(result_df, group_patterns, antipatterns=[],
                     swap_xy=True, reversed_y=True, iplot_option=True, plot_name='this_plot', print_all=True, x_name=None, y_name=None, traces_names = None):
    """
    Функция для построения графиков с учетом групп паттернов (в каждой группе должны выполняться все условия)
    и антипаттернов для выбора колонок для отображения

    :param print_all: опция - выводить все найденные совпадения и удаленные антипаттерны
    :param result_df:
    :param group_patterns:
    :param antipatterns:
    :return:
    """
    columns_to_plot = create_columns_to_plot(result_df, group_patterns, antipatterns, print_all)
    plot_specific_columns(result_df, columns_to_plot, swap_xy=swap_xy, reversed_y=reversed_y,
                          iplot_option=iplot_option, plot_name=plot_name, x_name=x_name, y_name=y_name, traces_names=traces_names)


def create_banches_from_pattern(df, banches_with_patterns: dict):
    banches = []
    for i,j in banches_with_patterns.items():
        columns_to_plot = create_columns_to_plot(df, j[0], j[1], print_all=False)
        one_banch = {i: columns_to_plot}
        banches.append(one_banch)
    return banches


def create_report_html(df, all_banches, filename, shared_xaxes=True,
                       shared_yaxes=False,
                      cols=1, one_plot_height=450,
                      verical_spacing=0.01, title_text='Распределение параметров',
                       swap_xy=False, reversed_y=False):
    """
    Создание шаблонизированного и удобного набора графиков
    :param df:
    :param all_banches:
    :param filename:
    :return:
    """
    subplot_amount = len(all_banches)
    subplot_titles = []
    for z in all_banches:
        subplot_titles.append(list(z.keys())[0])

    if cols == 1:
        rows = subplot_amount
    else:
        rows = subplot_amount // cols
        if subplot_amount % cols != 0:
            rows += 1


    fig = make_subplots(
        rows=rows, cols=cols, shared_xaxes=shared_xaxes,
        shared_yaxes=shared_yaxes,
        vertical_spacing=verical_spacing,
        subplot_titles=subplot_titles
    )
    for i in range(subplot_amount):
        this_df = df[all_banches[i][subplot_titles[i]]]
        this_banch_trace = create_traces_list_for_all_columms(this_df, chosen_mode='lines+markers', use_gl=True,
                                                              swap_xy=swap_xy)
        for j in this_banch_trace:
            if cols == 1:
                this_row = i+1
                this_col = 1
            else:
                this_row = i // cols + 1
                this_col = i % cols + 1
            fig.add_trace(j, row=this_row, col=this_col)

    fig.layout.hovermode = 'x'
    fig.layout.height = one_plot_height * rows
    fig.update_layout(
                      title_text=title_text)
    if reversed_y:
        fig.update_yaxes(autorange="reversed")
    plot(fig, filename=filename)
