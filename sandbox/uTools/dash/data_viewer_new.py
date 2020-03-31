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

10/01/2020

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
user32 = ctypes.windll.user32


# Служебные переменные, можно менять настройки перед запуском

ver = '0.3.1'
date = '01/2020'
external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
graph_mode = 'lines+markers' # 'markers', 'lines', 'lines+markers'
dir_path = 'input/'
using_screen_coef = 0.99
marker_size = 5


screen_height = int(user32.GetSystemMetrics(1) * using_screen_coef)

# Считывание файлов

file_paths = glob(dir_path + '*.csv')


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


def create_traces(dfs, df_names, params):
    """
    Создание traces для нескольки DataFrames
    :param dfs: список DataFrame
    :param df_names: название DataFrame, точнее - название файла, из которого создавался DataFrame
    :param params: общий список параметров. Есть проверка на наличие параметра в данном DataFrame и на заполненность
                    данными через dropna() для данного столбца
    :return: список traces
    """
    traces = []
    for df_name, df in zip(df_names, dfs):
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
    return traces


def plot_ex(dfs_group_list, dfs_name_group_list, params_group_list, height):
    """
    Построение графиков с синхронизированной осью x
    :param dfs_group_list: список из групп (списков) DataFrame для построения на каждом сабплоте
    :param dfs_name_group_list: список из групп (списков) имен DataFrame (файлов) для построения на каждом сабплоте
    :param params_group_list: список из групп (списков) параметров для построения на каждом сабплоте
    :param height: общая высота графика (рассчитывается исходя из разрешения экрана - screen_height)
    :return: объект plotly - figure, готовый для отображения
    """
    if len(dfs_group_list) == 1:
        fig = go.Figure(data=create_traces(dfs_group_list[0], dfs_name_group_list[0], params_group_list[0]))
    else:
        fig = make_subplots(rows=len(dfs_group_list), shared_xaxes=True, vertical_spacing=0.02)

        for j in range(1, len(dfs_group_list) + 1):
            traces = create_traces(dfs_group_list[j - 1], dfs_name_group_list[j - 1], params_group_list[j - 1])
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
                                    value=1
                                ),

                                html.Div(id='my-div'),

                                #html.Div([
                                #    dcc.Markdown("""
                                #                    **Selection Data**
                                #
                                #                    Choose the lasso or rectangle tool in the graph's menu
                                #                    bar and then select points in the graph.
                                #
                                #                    Note that if `layout.clickmode = 'event+select'`, selection data also
                                #                    accumulates (or un-accumulates) selected data if you hold down the shift
                                #                    button while clicking.
                                #                """),
                                #    html.Pre(id='selected-data', style=styles['pre']),
                                #], className='three columns'),

                                html.Div([
                                    html.P('Управление графиком 1', id='text-1'),
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
                                                       screen_height))])
                                ],
                      id='main')


@app.callback(Output('plot', 'figure'),
              [Input('loaded_files_dict-subplot-1', 'value'),
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
              Input('radio_items_plot_chose', 'value')])
def update_graph(file_vals1, col_vals1, file_vals2, col_vals2, file_vals3, col_vals3, file_vals4, col_vals4,
                 file_vals5, col_vals5, file_vals6, col_vals6, n_clicks):
    files_plot1 = [loaded_files_dict[val] for val in file_vals1]
    files_plot = [files_plot1]
    files_vals = [file_vals1]
    col_vals = [col_vals1]
    if n_clicks >= 2:
        files_plot2 = [loaded_files_dict[val] for val in file_vals2]
        files_plot.append(files_plot2)
        files_vals.append(file_vals2)
        col_vals.append(col_vals2)
    if n_clicks >= 3:
        files_plot3 = [loaded_files_dict[val] for val in file_vals3]
        files_plot.append(files_plot3)
        files_vals.append(file_vals3)
        col_vals.append(col_vals3)
    if n_clicks >= 4:
        files_plot4 = [loaded_files_dict[val] for val in file_vals4]
        files_plot.append(files_plot4)
        files_vals.append(file_vals4)
        col_vals.append(col_vals4)
    if n_clicks >= 5:
        files_plot5 = [loaded_files_dict[val] for val in file_vals5]
        files_plot.append(files_plot5)
        files_vals.append(file_vals5)
        col_vals.append(col_vals5)
    if n_clicks >= 6:
        files_plot6 = [loaded_files_dict[val] for val in file_vals6]
        files_plot.append(files_plot6)
        files_vals.append(file_vals6)
        col_vals.append(col_vals6)
    return plot_ex(files_plot, files_vals, col_vals, height=screen_height)


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


#app.callback(
#   Output('selected-data', 'children'),
#   [Input('plot', 'selectedData')])
#ef display_selected_data(selectedData):
#   print(selectedData)

#   print('\n')
#   print(type(selectedData))

#   print('\n')


#   print(selectedData['range'])

#   print(selectedData['range']['x'])

#   return json.dumps(selectedData, indent=2)


@app.callback(
    Output(component_id='my-div', component_property='children'),
    [Input('plot', 'selectedData')])
def display_selected_data(selectedData):
    if type(selectedData) != type(None):
        return f"Период выбранных данных с {selectedData['range']['x'][0]} по {selectedData['range']['x'][1]}"
    else:
        return None

if __name__ == '__main__':
    app.run_server(dev_tools_hot_reload=True)

