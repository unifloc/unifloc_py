"""
Веб-просмотрщик графиков. Открывается в браузере. Для построения графиков датафреймов по временному индексу.

Считывает все файлы .csv которые лежат в dir_path и строит графики всех колонок от временного индекса.
Можно добавить до 6 графиков.

К каждому графику идут 2 элемента управления:
    - 1-ый отвечает за выбор файлов для графика
    - 2-ый отвечает за выбор колонок. Список колонок уникальный из всех файлов.
      Поэтому если для конкретного файла выбранной колонки нет, то график не построится.

Код универсален и может быть использован для любого проекта,
нужно только поменять директорию для считывания файлов - переменная dir_path

01/04/2020

А.Водопьян
O.Koбзарь
"""

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import pandas as pd
from glob import glob
import json
import ctypes
from dash.exceptions import PreventUpdate

user32 = ctypes.windll.user32


# Служебные переменные, можно менять настройки перед запуском

ver = '0.5.1'
date = '04/2020'
external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
graph_mode = 'lines+markers' # 'markers', 'lines', 'lines+markers'
dir_path = 'input/'
using_screen_coef = 0.9
marker_size = 5


screen_height = int(user32.GetSystemMetrics(1) * using_screen_coef)
screen_width = int(user32.GetSystemMetrics(0))
# Считывание файлов

file_paths = glob(dir_path + '*.csv')

init_allocated_df = pd.DataFrame({'Тут': ['выделенные'],
                                  'будут': ['интервалы']},
                                 index=[0])

init_saved_df = pd.DataFrame({'Тут': ['сохраненные'],
                              'будут': ['интервалы']},
                                 index=[0])

def read_files(file_paths):
    file_names = [file_path[len(dir_path):] for file_path in file_paths]
    loaded_files_dict = {}

    for file_name, file_path in zip(file_names, file_paths):
        loaded_files_dict.update({file_name: pd.read_csv(file_path, index_col=0)})

    all_cols = list(loaded_files_dict[file_names[0]].columns)
    for i in range(1, len(file_names)):
        all_cols += list(loaded_files_dict[file_names[i]].columns)

    unique_cols = list(set(all_cols))

    return file_names, loaded_files_dict, unique_cols


file_names, loaded_files_dict, unique_cols = read_files(file_paths)


# Функции для построения графиков

def create_trace_for_table(df):
    all_columns = [df.index.name] + list(df.columns)
    #header_values = [x.replace(' ', '<br>') for x in all_columns]
    header_values = [x for x in all_columns]
    cells_values = [df.index.to_list()] + [df[x].to_list() for x in df.columns]
    trace = go.Table(
        header=dict(
            values=header_values,
            font=dict(size=10),
            align="left"
        ),
        cells=dict(
            values=cells_values,
            align="left")
    )
    return trace


def create_table_figure(df, table_name):
    fig = go.Figure(data = create_trace_for_table(df))
    fig.layout.title = table_name
    fig.update_layout(width=int(screen_width * 0.45))

    return fig


def create_traces(dfs, df_names, params, graph_type):
    """
    Создание traces для нескольки DataFrames
    :param dfs: список DataFrame
    :param df_names: название DataFrame, точнее - название файла, из которого создавался DataFrame
    :param params: общий список параметров. Есть проверка на наличие параметра в данном DataFrame и на заполненность
                    данными через dropna() для данного столбца
    :param graph_type: тип графика
    :return: список traces
    """
    traces = []
    for df_name, df in zip(df_names, dfs):
        if graph_type == 1:
            for par in params:
                if par in df.columns:
                    this_df = df.dropna(subset=[par])
                    if this_df.shape[0] > 0:
                        traces.append(go.Scattergl(x=this_df.index, y=this_df[par], name=df_name + ', ' + par,
                                                   text=[df_name + ', ' + par] * len(this_df.index),
                                                   mode=graph_mode,
                                                   #hovertemplate="<b>%{text}</b> x: %{x} y: %{y}<extra></extra>",
                                                   hovertemplate="<b>%{text}</b>: %{y}<extra></extra>",
                                                   marker=dict(size=marker_size)))
                    else:
                        pass
        else:
            if len(params) < 2:
                pass
            else:
                params = params[:2]
                if not((params[0] in df.columns) & (params[0] in df.columns)):
                    pass
                else:
                    this_df = df.dropna(subset=[params[1]]) 
                    traces.append(go.Heatmap(x=this_df.index,
                                             y=this_df[params[0]],
                                             z=this_df[params[1]],
                                        xaxis="x2",
                                        colorscale='Viridis'))
    return traces


def plot_ex(dfs_group_list, dfs_name_group_list, params_group_list, height,
            graph_types):
    """
    Построение графиков с синхронизированной осью x
    :param dfs_group_list: список из групп (списков) DataFrame для построения на каждом сабплоте
    :param dfs_name_group_list: список из групп (списков) имен DataFrame (файлов) для построения на каждом сабплоте
    :param params_group_list: список из групп (списков) параметров для построения на каждом сабплоте
    :param height: общая высота графика (рассчитывается исходя из разрешения экрана - screen_height)
    :param graph_types: список типов графиков - 1 = scatter или 2 = heatmap
    :return: объект plotly - figure, готовый для отображения
    """
    if len(dfs_group_list) == 1:
        fig = go.Figure(data=create_traces(dfs_group_list[0], dfs_name_group_list[0], params_group_list[0], graph_types[0]))
    else:
        fig = make_subplots(rows=len(dfs_group_list), shared_xaxes=True, vertical_spacing=0.02)

        for j in range(1, len(dfs_group_list) + 1):
            traces = create_traces(dfs_group_list[j - 1], dfs_name_group_list[j - 1],
                                   params_group_list[j - 1], graph_types[j - 1])
            for trace in traces:
                fig.append_trace(trace, row=j, col=1)

    fig.update_layout(height=height)
    fig.layout.hovermode = 'x'
    fig.layout.clickmode = 'event+select'
    return fig


# Код приложения
styles = {
    'pre': {
        'border': 'thin lightgrey solid',
        'overflowX': 'scroll'
    }
}

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div(children=[html.Div([html.H4('Просмотрщик графиков'),
                                          html.P('Версия %s. %s' % (ver, date)),
                                          html.P('Водопьян А.О., Кобзарь О.С.')],
                                         style={'width': '20%', 'display': 'inline-block'}),


                                html.Label('Количество графиков'),
                                dcc.RadioItems(id='radio_items_plot_chose',
                                    options=[
                                        {'label': '1',  'value': 1},
                                        {'label': '2',  'value': 2},
                                        {'label': '3',  'value': 3},
                                        {'label': '4',  'value': 4},
                                        {'label': '5',  'value': 5},
                                        {'label': '6',  'value': 6},
                                    ],
                                    value=1,
                                    labelStyle={'display': 'inline-block'}
                                ),

                                html.Label('Выбор режима работы приложения'),
                                dcc.RadioItems(id='radio_items_mode_chose',
                                    options=[
                                        {'label': 'Просмотрщик',  'value': 1},
                                        {'label': 'Разметчик',  'value': 2},
                                    ],
                                    value=1,
                                    labelStyle={'display': 'inline-block'}
                                ),


                                html.Div([
                                    html.P('Управление графиком 1', id='text-1'),
                                    dcc.RadioItems(id='graph-type-choose-1',
                                    options=[
                                        {'label': 'scatter',  'value': 1},
                                        {'label': 'heatmap',  'value': 2}
                                            ], value=1,
                                    labelStyle={'display': 'inline-block'}
                                                ),
                                    dcc.Dropdown(
                                        id='loaded_files_dict-subplot-1',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-subplot-1',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True)],
                                    id='management-1'),

                                html.Div([
                                    html.P('Управление графиком 2', id='text-2'),
                                    dcc.RadioItems(id='graph-type-choose-2',
                                    options=[
                                        {'label': 'scatter',  'value': 1},
                                        {'label': 'heatmap',  'value': 2}
                                            ], value=1,
                                    labelStyle={'display': 'inline-block'}
                                                ),                                    
                                    dcc.Dropdown(
                                        id='loaded_files_dict-subplot-2',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-subplot-2',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True)], id='management-2', style={'display': 'none'}),

                                html.Div([
                                    html.P('Управление графиком 3', id='text-3'),
                                    dcc.RadioItems(id='graph-type-choose-3',
                                    options=[
                                        {'label': 'scatter',  'value': 1},
                                        {'label': 'heatmap',  'value': 2}
                                            ], value=1,
                                    labelStyle={'display': 'inline-block'}
                                                ),
                                    dcc.Dropdown(
                                        id='loaded_files_dict-subplot-3',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-subplot-3',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True)], id='management-3', style={'display': 'none'}),

                                html.Div([
                                    html.P('Управление графиком 4', id='text-4'),
                                    dcc.RadioItems(id='graph-type-choose-4',
                                    options=[
                                        {'label': 'scatter',  'value': 1},
                                        {'label': 'heatmap',  'value': 2}
                                            ], value=1,
                                    labelStyle={'display': 'inline-block'}
                                                ),                                    
                                    dcc.Dropdown(
                                        id='loaded_files_dict-subplot-4',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-subplot-4',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True)], id='management-4', style={'display': 'none'}),

                                html.Div([
                                    html.P('Управление графиком 5', id='text-5'),
                                    dcc.RadioItems(id='graph-type-choose-5',
                                    options=[
                                        {'label': 'scatter',  'value': 1},
                                        {'label': 'heatmap',  'value': 2}
                                            ], value=1,
                                    labelStyle={'display': 'inline-block'}
                                                ),                                    
                                    dcc.Dropdown(
                                        id='loaded_files_dict-subplot-5',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-subplot-5',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True)], id='management-5', style={'display': 'none'}),

                                html.Div([
                                    html.P('Управление графиком 6', id='text-6'),
                                    dcc.RadioItems(id='graph-type-choose-6',
                                    options=[
                                        {'label': 'scatter',  'value': 1},
                                        {'label': 'heatmap',  'value': 2}
                                            ], value=1,
                                        labelStyle={'display': 'inline-block'}
                                                ),
                                    dcc.Dropdown(
                                        id='loaded_files_dict-subplot-6',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-subplot-6',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True)], id='management-6', style={'display': 'none'}),

                                html.Div([
                                    dcc.Graph(
                                        id='plot',
                                        figure=plot_ex([[loaded_files_dict[file_names[0]]]],
                                                       [[file_names[0]]],
                                                       [[loaded_files_dict[file_names[0]].columns[0]]],
                                                       screen_height, [1]))]),

                                html.Div([

                                    # html.P('Метка выделенного события', id='info'),
                                    dcc.Input(id='label_for_event', value='event1', type='text',
                                              style={'display': 'none'}),

                                    html.Button('Сохранить выделенный интервал', id='button', n_clicks=0,
                                                style={'display': 'none'}),

                                    html.Button('Удалить последний интервал', id='button_to_del', n_clicks=0,
                                                style={'display': 'none'}),

                                    dcc.Graph(
                                        id='table_for_allocated_data',
                                        figure=create_table_figure(init_allocated_df, 'Выделенные интервалы'),
                                        style={'display': 'none'}),

                                    dcc.Graph(
                                        id='table_for_saved_data',
                                        figure=create_table_figure(init_saved_df, 'Сохраненные интервалы'),
                                        style={'display': 'none'})
                                ]),

                                # Hidden div inside the app that stores the intermediate value
                                html.Div(id='data_storage_allocated', style={'display': 'none'}),
                                html.Div(id='data_storage_saved', style={'display': 'none'})

                                ],
                      id='main')


@app.callback(Output('plot', 'figure'),
              [Input('loaded_files_dict-subplot-1', 'value'),
               Input('cols-subplot-1', 'value'),
               Input('graph-type-choose-1', 'value'),
               Input('loaded_files_dict-subplot-2', 'value'),
               Input('cols-subplot-2', 'value'),
               Input('graph-type-choose-2', 'value'),
               Input('loaded_files_dict-subplot-3', 'value'),
               Input('cols-subplot-3', 'value'),
               Input('graph-type-choose-3', 'value'),
               Input('loaded_files_dict-subplot-4', 'value'),
               Input('cols-subplot-4', 'value'),
               Input('graph-type-choose-4', 'value'),               
               Input('loaded_files_dict-subplot-5', 'value'),
               Input('cols-subplot-5', 'value'),
               Input('graph-type-choose-5', 'value'),
               Input('loaded_files_dict-subplot-6', 'value'),
               Input('cols-subplot-6', 'value'),
               Input('graph-type-choose-6', 'value'),
              Input('radio_items_plot_chose', 'value'),
               Input('radio_items_mode_chose', 'value')])
def update_graph(file_vals1, col_vals1, chose_vals1, file_vals2, col_vals2, 
                 chose_vals2, file_vals3, col_vals3, chose_vals3, 
                 file_vals4, col_vals4, chose_vals4, file_vals5, col_vals5,
                 chose_vals5, file_vals6, col_vals6, chose_vals6, n_clicks, mode_value):
    
    files_plot1 = [loaded_files_dict[val] for val in file_vals1]
    files_plot = [files_plot1]
    files_vals = [file_vals1]
    col_vals = [col_vals1]
    chose_vals = [chose_vals1]
    if n_clicks >= 2:
        files_plot2 = [loaded_files_dict[val] for val in file_vals2]
        files_plot.append(files_plot2)
        files_vals.append(file_vals2)
        col_vals.append(col_vals2)
        chose_vals.append(chose_vals2)
    if n_clicks >= 3:
        files_plot3 = [loaded_files_dict[val] for val in file_vals3]
        files_plot.append(files_plot3)
        files_vals.append(file_vals3)
        col_vals.append(col_vals3)
        chose_vals.append(chose_vals3)
    if n_clicks >= 4:
        files_plot4 = [loaded_files_dict[val] for val in file_vals4]
        files_plot.append(files_plot4)
        files_vals.append(file_vals4)
        col_vals.append(col_vals4)
        chose_vals.append(chose_vals4)
    if n_clicks >= 5:
        files_plot5 = [loaded_files_dict[val] for val in file_vals5]
        files_plot.append(files_plot5)
        files_vals.append(file_vals5)
        col_vals.append(col_vals5)
        chose_vals.append(chose_vals5)
    if n_clicks >= 6:
        files_plot6 = [loaded_files_dict[val] for val in file_vals6]
        files_plot.append(files_plot6)
        files_vals.append(file_vals6)
        col_vals.append(col_vals6)
        chose_vals.append(chose_vals6)
    if mode_value == 1:
        return plot_ex(files_plot, files_vals, col_vals, screen_height, chose_vals)
    else:
        return plot_ex(files_plot, files_vals, col_vals, int(0.6 * screen_height), chose_vals)



@app.callback([Output('management-2', 'style'),
            Output('management-3', 'style'),
            Output('management-4', 'style'),
            Output('management-5', 'style'),
            Output('management-6', 'style')],
            [Input('radio_items_plot_chose', 'value')])
def show_graph(value):
    if value == 1:
        return {'display': 'none'}, {'display': 'none'}, {'display': 'none'}, {'display': 'none'}, {'display': 'none'}
    elif value == 2:
        return {'display': 'inline'}, {'display': 'none'}, {'display': 'none'}, {'display': 'none'}, {'display': 'none'}
    elif value == 3:
        return {'display': 'inline'}, {'display': 'inline'}, {'display': 'none'}, {'display': 'none'}, {'display': 'none'}
    elif value == 4:
        return {'display': 'inline'}, {'display': 'inline'}, {'display': 'inline'}, {'display': 'none'}, {'display': 'none'}
    elif value == 5:
        return {'display': 'inline'}, {'display': 'inline'}, {'display': 'inline'}, {'display': 'inline'}, {'display': 'none'}
    elif value == 6:
        return {'display': 'inline'}, {'display': 'inline'}, {'display': 'inline'}, {'display': 'inline'}, {'display': 'inline'}



@app.callback([Output('table_for_allocated_data', 'style'),
            Output('table_for_saved_data', 'style'),

            Output('button', 'style'),
            Output('button_to_del', 'style'),
               Output('label_for_event', 'style')],
            [Input('radio_items_mode_chose', 'value')])
def show_graph(value):
    if value == 1:
        ts = {'display': 'none'}
        return ts, ts, ts, ts, ts
    elif value == 2:
        button = {'width': '30%', 'height': '10%', 'display': 'inline-block', "margin-right": "25px"}
        table = {'width': '40%', 'display': 'inline-block', "margin-bottom": "1px", "margin-right": "1px"}
        return table, table, button, button, button


@app.callback(
    [Output('table_for_allocated_data', 'figure'),
    Output('data_storage_allocated', 'children')],
    [Input('plot', 'selectedData'),
     Input('loaded_files_dict-subplot-1', 'value'),
     Input('cols-subplot-1', 'value'),
     Input('loaded_files_dict-subplot-2', 'value'),
     Input('cols-subplot-2', 'value'),
     Input('loaded_files_dict-subplot-3', 'value'),
     Input('cols-subplot-3', 'value'),
     Input('loaded_files_dict-subplot-4', 'value'),
     Input('cols-subplot-4', 'value'),
     Input('loaded_files_dict-subplot-5', 'value'),
     Input('cols-subplot-5', 'value'),
     Input('loaded_files_dict-subplot-6', 'value'),
     Input('cols-subplot-6', 'value'),
     Input('label_for_event', 'value')])
def display_selected_data(selectedData, file_vals1, col_vals1,
                file_vals2, col_vals2,
                  file_vals3, col_vals3,
                 file_vals4, col_vals4,
                file_vals5, col_vals5,
                  file_vals6, col_vals6,
                          label_for_event):
    if type(selectedData) != type(None):
        if 'range' in selectedData.keys():
            df = pd.DataFrame.from_dict(selectedData['range'])
            axis_name_of_selected_subplot = df.columns[0]
            if axis_name_of_selected_subplot == 'x':
                file_vals = file_vals1
                col_vals = col_vals1
            elif axis_name_of_selected_subplot == 'x2':
                file_vals = file_vals2
                col_vals = col_vals2
            elif axis_name_of_selected_subplot == 'x3':
                file_vals = file_vals3
                col_vals = col_vals3
            elif axis_name_of_selected_subplot == 'x4':
                file_vals = file_vals4
                col_vals = col_vals4
            elif axis_name_of_selected_subplot == 'x5':
                file_vals = file_vals5
                col_vals = col_vals5
            else:
                file_vals = file_vals6
                col_vals = col_vals6

            file_vals_for_csv = ''
            for i in file_vals:
                file_vals_for_csv += i + ','
            file_vals_for_csv = file_vals_for_csv[:-1]
            col_vals_for_csv = ''
            for i in col_vals:
                col_vals_for_csv += i + ','
            col_vals_for_csv = col_vals_for_csv[:-1]


            df = pd.DataFrame({'Начало интервала': [df[df.columns[0]][0]],
                               'Конец интервала': [df[df.columns[0]][1]],
                               'Ось сабплота': [axis_name_of_selected_subplot],
                               'Название файлов': [file_vals_for_csv],
                               'Название параметров': [col_vals_for_csv]},
                              index=[label_for_event])
            df.index.name = 'Название события'

            return create_table_figure(df, 'Выделенные интервалы'), df.to_json(date_format='iso', orient='split')
        else:
            raise PreventUpdate
    else:
        raise PreventUpdate


class N_click_save(): # не знаю как сделать триггер на button
    def __init__(self):
        self.n_clicks = 0
        self.n_clicks_to_del = 0


n_click_save = N_click_save()  # костыль - так делать нельзя


@app.callback(
    [Output('table_for_saved_data', 'figure'),
    Output('data_storage_saved', 'children')],
    [Input('data_storage_allocated', 'children'),
     Input('button', 'n_clicks'),
     Input('button_to_del', 'n_clicks')])
def save_selected_data(allocated_data, n_clicks, n_clicks_to_del):
    if n_clicks > n_click_save.n_clicks or n_clicks_to_del > n_click_save.n_clicks_to_del:
        print('Вызов сохранения в таблицу')
        if n_clicks > n_click_save.n_clicks:
            n_click_save.n_clicks = n_clicks
            if type(allocated_data) != type(None):
                df = pd.read_json(allocated_data, orient='split')
                if n_clicks > 1:
                    loaded_df = pd.read_csv('saved_data.csv', index_col=[0])
                    df = loaded_df.append(df)
                    df.to_csv('saved_data.csv')
                else:
                    df.to_csv('saved_data.csv')

                return create_table_figure(df, 'Сохраненные интервалы'), df.to_json(date_format='iso', orient='split')
        else:
            print('Вызов удаления последней строчки')
            n_click_save.n_clicks_to_del = n_clicks_to_del
            if n_clicks > 0:
                loaded_df = pd.read_csv('saved_data.csv', index_col=[0])
                if loaded_df.shape[0] >= 1:
                    df = loaded_df.iloc[0:-1]
                    df.to_csv('saved_data.csv')
                else:
                    df = loaded_df[loaded_df.index == None]
                return create_table_figure(df, 'Сохраненные интервалы'), df.to_json(date_format='iso',
                                                                                   orient='split')

    else:
        raise PreventUpdate


if __name__ == '__main__':
    #app.run_server(debug=True, dev_tools_hot_reload=True)
    app.run_server()


