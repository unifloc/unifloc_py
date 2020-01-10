"""
dash experiment experiment
with WILI well
"""

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output, State
import dash_table
from datetime import datetime as dt
import plotly.graph_objs as go
import pandas as pd


styles = {
    'pre': {
        'border': 'thin lightgrey solid',
        'overflowX': 'scroll'
    }
}

"""
read data
data must be prepared with Jupyter Notebook preprocessor_notebook.ipynb
and stored in temp folder
"""

project_name = 'Brage'
well_df = pd.read_csv('../data/' + project_name + '/preprocessed/df_well_resample.csv', index_col=0)
df_sel = pd.DataFrame(columns=['Case', 'Date start', 'Date end'])


def plot_scatter(df, key1, key2, height):
    """
    preparation of scatter plot
    :param df:  dataframe to plot
    :param key1: column name in df for x axe
    :param key2: column name in df for y axe
    :return: figure to plot
    """
    layout = go.Layout(
        autosize=True,
        margin={'t': 0.5},
        height=height,
        title=None,
        xaxis=go.layout.XAxis(
            mirror=True,
            ticks='outside',
            showline=True,
            title=go.layout.xaxis.Title(
                text=key1
            )
        ),
        yaxis=go.layout.YAxis(
            mirror=True,
            ticks='outside',
            showline=True,
            title=go.layout.yaxis.Title(
                text=key2
            )
        )
    )
    # Create traces
    trace0 = go.Scatter(
        x=df[key1],
        y=df[key2],
        mode='markers',
        name='plot',
        customdata=df.index   # save index for late data collection
    )
    return go.Figure(data=[trace0], layout=layout)


def plot_timeline(df, keys, height):
    """
    preparation of time line plot
    :param df:  dataframe to plot
    :param keys: column name in df for x axe
    :return: figure to plot
    """
    layout = go.Layout(
        margin={'t': 0.5},
        autosize=True,
        height=height,
        #title='Timeline for ' + str(keys)[2:-2],
        yaxis=go.layout.YAxis(
            mirror=True,
            ticks='outside',
            showline=True,
            title=go.layout.yaxis.Title(
                text=str(keys)[2:-2]
            )
        ),
        xaxis=go.layout.XAxis(
            mirror=True,
            ticks='outside',
            showline=True,
            title=go.layout.xaxis.Title(
                text='Date'
            )
        )
    )
    # Create traces
    traces = []
    for key in keys:
        trace0 = go.Scatter(
            x=df.index,
            y=df[key],
            mode='markers',
            name=key,
            customdata=df.index  # save index for late data collection
        )
        traces.append(trace0)
    return go.Figure(data=traces, layout=layout)


external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)


app.layout = html.Div(children=[
    dcc.Tabs(id="tabs", value='tab-1', children=[
        dcc.Tab(label='Anomaly labeling tool', value='tab-1', children=[
html.Div([
            html.Div([
                html.Div([
                    html.Button(id='scatter1-button', n_clicks=0, children='Scatter 1')
                    ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'}),
                html.Div([
                    html.Button(id='scatter2-button', n_clicks=0, children='Scatter 2')
                    ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'}),
                html.Div([
                    html.Button(id='select-button', n_clicks=0, children='Select')
                ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'}),
                html.Div([
                    html.Button(id='filter-button', n_clicks=0, children='Add filter')
                ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'}),
                html.Div([
                    html.Button(id='filter-inverse-button', n_clicks=0, children='Inverse filter')
                ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'}),
                html.Div([
                    html.Button(id='add-case-button', n_clicks=0, children='Add case')
                ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'}),
                html.Div([
                    html.Button(id='add-event-button', n_clicks=0, children='Add event')
                ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'}),
                html.Div([
                    dcc.Input(
                        id='case',
                        placeholder='Enter name of the case',
                        type='text'
                    )
                ], style={'marginTop': 30, 'marginLeft': 45, 'display': 'inline-block'}),
                html.Div([
                    html.Button(id='load-button', n_clicks=0, children='Save')
                ], style={'marginTop': 30, 'marginLeft': 25, 'display': 'inline-block'})],
               style={'borderBottom': 'thin lightgrey solid',
                      'backgroundColor': 'rgb(250, 250, 250)',
                      'padding': '10px'}),
            html.Div([
                html.Div([
                    dcc.Graph(
                        id='select-graph',
                        figure=plot_timeline(well_df, [well_df.columns[0]], 250)
                    ),
                ], style={'marginTop': 30, 'marginLeft': 35}),
                html.Div([
                    html.Div(className="six columns", children=[
                        dcc.Graph(
                            id='scatter-graph1',
                            figure=plot_scatter(well_df, well_df.columns[0], well_df.columns[0], 250)
                        ),
                        dcc.Graph(
                            id='scatter-graph2',
                            figure=plot_scatter(well_df, well_df.columns[0], well_df.columns[0], 250))
                        ])
                ], style={'marginLeft': 35
                          }),
                html.Div(className="six columns",
                         children=[
                            dcc.Graph(
                                id='timeline-graph',
                                figure=plot_timeline(well_df, [well_df.columns[0]], 500)
                            ),
                            ], style={'marginLeft': 35
                                      })
            ]),
            html.Div([
                html.Div(className="six columns", children=[
                    html.H4('Select axes for Scatter plot 1'),
                    dcc.Dropdown(
                        id='key1-column',
                        options=[{'label': i, 'value': i} for i in well_df.columns],
                        value=well_df.columns[0]
                    ),
                    dcc.Dropdown(
                        id='key2-column',
                        options=[{'label': i, 'value': i} for i in well_df.columns],
                        value=well_df.columns[0]
                    ),
                    dcc.RadioItems(
                        id='radio-working-1',
                        options=[{'label': i, 'value': i} for i in ['all', 'working', 'non working']],
                        value='all',
                        labelStyle={'display': 'inline-block'}
                    ),
                    html.H4('Select axes for Scatter plot 2'),
                    dcc.Dropdown(
                        id='key3-column',
                        options=[{'label': i, 'value': i} for i in well_df.columns],
                        value=well_df.columns[0]
                    ),
                    dcc.Dropdown(
                        id='key4-column',
                        options=[{'label': i, 'value': i} for i in well_df.columns],
                        value=well_df.columns[0]
                    ),
                    dcc.RadioItems(
                        id='radio-working-2',
                        options=[{'label': i, 'value': i} for i in
                                 ['all', 'working', 'non working']],
                        value='all',
                        labelStyle={'display': 'inline-block'}
                    ),
                    html.H4('Select timeline data'),
                    dcc.Dropdown(
                        id='timeline-drop',
                        options=[{'label': i, 'value': i} for i in well_df.columns],
                        value=[well_df.columns[0]],
                        multi=True
                    ),
                    html.H4('Filters'),
                    dash_table.DataTable(
                        id='filter-table',
                        columns=[{'id': c, 'name': c} for c in ['Active', 'Filter', 'Case id']],
                        style_as_list_view=True)
                ]),
                html.Div(className='six columns', children=[
                    html.Div([
                        html.H4('Cases and anomalies'),
                        dash_table.DataTable(
                            id='case-table',
                            columns=[{'id': c, 'name': c} for c in ['Active', 'Case', 'Case id',
                                                                    'Date start', 'Date end']],
                            style_as_list_view=True)], style={'marginBottom': 150}),
                    html.H4('Events'),
                    dash_table.DataTable(
                        id='events-table',
                        columns=[{'id': c, 'name': c} for c in ['Active', 'Event', 'Case id',
                                                                'Date start']],
                        style_as_list_view=True)
                ])
            ])
        ])
        ]),
        dcc.Tab(label='Analysis tool', value='tab-2', children=[
            html.Div([
                html.H3('Tab content 2')
            ])
        ]),
    ]),
    html.Div(id='tabs-content')
])


@app.callback(Output('timeline-graph', 'figure'),
              [Input('select-button', 'n_clicks')],
              [State('timeline-graph', 'selectedData'),
               State('timeline-drop', 'value')])
def update_output1(n_clicks, sel_data, key):
    if sel_data is not None:
        lst = [p['customdata'] for p in sel_data['points']]
        df2 = well_df.loc[lst]
    else:
        df2 = well_df
    return plot_timeline(df2, key, 500)


if __name__ == '__main__':
    app.run_server(debug=True)
