"""
Просмотрщик графиков
10/01/2020
O.Koбзарь
А.Водопьян
"""

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output, State
import dash_table
from datetime import datetime as dt
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import pandas as pd
from glob import glob

ver = 0.1
date = '01/2020'
external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']


file_paths = glob('input/*.csv')
file_names = [file_path[6:] for file_path in file_paths]
files = {}

for file_name, file_path in zip(file_names, file_paths):
    files.update({file_name: pd.read_csv(file_path, index_col=0)})

all_cols = list(files[file_names[0]].columns)
for i in range(1, len(file_names)):
    all_cols += list(files[file_names[i]].columns)

unique_cols = list(set(all_cols))


def plot_ex(dfs1, df_keys1, params1, dfs2, df_keys2, params2):

    traces = []
    fig = make_subplots(rows=2, shared_xaxes=True, vertical_spacing=0.02)

    for key, df in zip(df_keys1, dfs1):
        for par in params1:
            if par in df.columns:
                fig.add_trace(go.Scattergl(x=df.index, y=df[par], name=key + ' ' + par, mode='lines+markers'),
                              row=1, col=1)

    for key, df in zip(df_keys2, dfs2):
        for par in params2:
            if par in df.columns:
                fig.add_trace(go.Scattergl(x=df.index, y=df[par], name=key + ' ' + par, mode='lines+markers'),
                              row=2, col=1)

    fig.update_layout(height=400)

    return fig


app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div(children=[html.Div([html.H4('Удобный просмотрщик графиков'),
                                          html.P('Версия %s. %s' % (ver, date)),
                                          html.P('Водопьян А.О., Кобзарь О.С.')]),
                                html.Div([
                                    html.P('Управление графиком 1'),
                                    dcc.Dropdown(
                                        id='files-plot0',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-plot0',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True),
                                    html.P('Управление графиком 2'),
                                    dcc.Dropdown(
                                        id='files-plot1',
                                        options=[{'label': i, 'value': i} for i in file_names],
                                        value=[file_names[0]],
                                        multi=True),
                                    dcc.Dropdown(
                                        id='cols-plot1',
                                        options=[{'label': i, 'value': i} for i in unique_cols],
                                        value=[unique_cols[0]],
                                        multi=True),
                                    dcc.Graph(
                                        id='plot0',
                                        figure=plot_ex([files[file_names[0]]], [file_names[0]],
                                                       [files[file_names[0]].columns[0]],
                                                       [files[file_names[0]]], [file_names[0]],
                                                       [files[file_names[0]].columns[0]])
                                    )])
                                ])


@app.callback(Output('plot0', 'figure'),
              [Input('files-plot0', 'value'),
               Input('cols-plot0', 'value'),
               Input('files-plot1', 'value'),
               Input('cols-plot1', 'value')])
def update_graph(file_vals0, col_vals0, file_vals1, col_vals1):
    files_plot0 = [files[val] for val in file_vals0]
    files_plot1 = [files[val] for val in file_vals1]
    return plot_ex(files_plot0, file_vals0, col_vals0, files_plot1, file_vals1, col_vals1)


if __name__ == '__main__':
    app.run_server(debug=True)
