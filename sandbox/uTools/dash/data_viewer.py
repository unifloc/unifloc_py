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
from dash.exceptions import PreventUpdate
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import pandas as pd
from glob import glob

# Служебные переменные, можно менять настройки перед запуском

ver = '0.2.1'
date = '01/2020'
external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
graph_mode = 'lines+markers' # 'markers', 'lines', 'lines+markers'
dir_path = 'input/'

# Считывание файлов

file_paths = glob(dir_path + '*.csv')
file_names = [file_path[len(dir_path):] for file_path in file_paths]
files = {}

for file_name, file_path in zip(file_names, file_paths):
    files.update({file_name: pd.read_csv(file_path, index_col=0)})

all_cols = list(files[file_names[0]].columns)
for i in range(1, len(file_names)):
    all_cols += list(files[file_names[i]].columns)

unique_cols = list(set(all_cols))

# Функции для построения графиков


def make_sub(dfs, df_keys, params):
    traces = []
    for key, df in zip(df_keys, dfs):
        for par in params:
            if par in df.columns:
                this_df = df.dropna(subset = [par])
                if this_df.shape[0] > 0:
                    traces.append(go.Scattergl(x=this_df.index, y=this_df[par], name=key + ', ' + par, mode=graph_mode,
                                               marker=dict(size=5)))
                else:
                    pass
    return traces


def plot_ex(list_dfs, list_df_keys, list_params, height):

    fig = make_subplots(rows=len(list_dfs), shared_xaxes=True, vertical_spacing=0.02)

    for j in range(1, len(list_dfs)+1):
        traces = make_sub(list_dfs[j-1], list_df_keys[j-1], list_params[j-1])
        for trace in traces:
            fig.append_trace(trace, row=j, col=1)

    fig.update_layout(height=height)
    #fig.update_yaxes(dtick=1)
    return fig

# Код приложения


app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div(children=[html.Div([html.H4('Просмотрщик графиков'),
                                          html.P('Версия %s. %s' % (ver, date)),
                                          html.P('Водопьян А.О., Кобзарь О.С.')],
                                         style={'width': '20%', 'display': 'inline-block'}),
                                html.Div([html.H4('Создайте от 2 до 6 графиков'),
                                          html.P('Выберите файл/файлы для каждого графика'),
                                          html.P('Выберите параметр/параметры для каждого графика')],
                                         style={'width': '20%', 'display': 'inline-block'}),

                                html.Div([html.Button(id='add-graph-button', n_clicks=0, children='Добавить график')],
                                         style={'display': 'inline-block'}),
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
                                        multi=True)], id='management-1'),
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
                                        multi=True)], id='management-2'),
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
                                        figure=plot_ex([[files[file_names[0]]], [files[file_names[0]]]],
                                                       [[file_names[0]], [file_names[0]]],
                                                       [[files[file_names[0]].columns[0]],
                                                        [files[file_names[0]].columns[0]]], 600)
                                    )])
                                ], id='main')


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
               Input('cols-subplot-6', 'value')],
              [State('add-graph-button', 'n_clicks')])
def update_graph(file_vals1, col_vals1, file_vals2, col_vals2, file_vals3, col_vals3, file_vals4, col_vals4,
                 file_vals5, col_vals5, file_vals6, col_vals6, n_clicks):
    files_plot1 = [files[val] for val in file_vals1]
    files_plot2 = [files[val] for val in file_vals2]
    files_plot = [files_plot1, files_plot2]
    files_vals = [file_vals1, file_vals2]
    col_vals = [col_vals1, col_vals2]
    height = 600
    if 0 < n_clicks:
        files_plot3 = [files[val] for val in file_vals3]
        files_plot.append(files_plot3)
        files_vals.append(file_vals3)
        col_vals.append(col_vals3)
        height += 200
        if 1 < n_clicks:
            files_plot4 = [files[val] for val in file_vals4]
            files_plot.append(files_plot4)
            files_vals.append(file_vals4)
            col_vals.append(col_vals4)
            height += 200
            if 2 < n_clicks:
                files_plot5 = [files[val] for val in file_vals5]
                files_plot.append(files_plot5)
                files_vals.append(file_vals5)
                col_vals.append(col_vals5)
                height += 200
                if 3 < n_clicks:
                    files_plot6 = [files[val] for val in file_vals6]
                    files_plot.append(files_plot6)
                    files_vals.append(file_vals6)
                    col_vals.append(col_vals6)
                    height += 200
    return plot_ex(files_plot, files_vals, col_vals, height)


@app.callback(Output('management-3', 'style'),
              [Input('add-graph-button', 'n_clicks')])
def update_management_3(n_clicks):
    if 0 < n_clicks < 2:
        return {'display': 'inline'}
    else:
        raise PreventUpdate


@app.callback(Output('management-4', 'style'),
              [Input('add-graph-button', 'n_clicks')])
def update_management_4(n_clicks):
    if 1 < n_clicks < 3:
        return {'display': 'inline'}
    else:
        raise PreventUpdate


@app.callback(Output('management-5', 'style'),
              [Input('add-graph-button', 'n_clicks')])
def update_management_5(n_clicks):
    if 2 < n_clicks < 4:
        return {'display': 'inline'}
    else:
        raise PreventUpdate


@app.callback(Output('management-6', 'style'),
              [Input('add-graph-button', 'n_clicks')])
def update_management_6(n_clicks):
    if 3 < n_clicks < 5:
        return {'display': 'inline'}
    else:
        raise PreventUpdate


if __name__ == '__main__':
    app.run_server()
