import networkx as nx
import numpy as np
import pandas as pd
from HE2_Pipe import HE2_WaterPipe
import HE2_Vertices as vrtxs


def make_schema_from_OISPipe_dataframes(df_pipes, df_boundaries):
    df = df_pipes[["node_id_start", "node_id_end"]]
    df.columns = ["source", "target"]
    G = nx.from_pandas_edgelist(df, create_using=nx.DiGraph)
    edge_list = list(G.edges())
    edge_set = set(edge_list)
    rev_set = set([(v, u) for u, v in edge_set])
    fin_edge_set = edge_set - rev_set
    G = nx.DiGraph(fin_edge_set)

    cmpnts = nx.algorithms.components.number_connected_components(nx.Graph(fin_edge_set))
    if cmpnts != 1:
        print('Not single component graph!')
        assert False

    pipes = dict()
    for u, v in G.edges():
        df = df_pipes
        df = df[(df.node_id_start == u) & (df.node_id_end == v)]
        d = df.iloc[0].to_dict()

        Ls = [d['L']]
        Hs = [d['altitude_end'] - d['altitude_start']]
        Ds = [d['D'] - 2 * d['S']]
        Rs = [1e-5]

        pipe = HE2_WaterPipe(Ls, Hs, Ds, Rs)
        pipes[(u, v)] = pipe
    nx.set_edge_attributes(G, name='obj', values=pipes)


    df = df_boundaries[['Q', 'P']].fillna(-1e9)
    df_boundaries['value'] = df.max(axis=1)

    nodes = dict()
    id_list = list(df_boundaries.id.values)
    for n in G.nodes():
        df = df_boundaries
        if n in id_list:
            df = df[df.id == n]
            d = df.iloc[0].to_dict()
        else:
            obj = vrtxs.HE2_ABC_GraphVertex()
            nodes[n] = obj
            continue
        # if d['kind']=='P':
        #     print(d)

        if d['is_source']:
            obj = vrtxs.HE2_Source_Vertex(d['kind'], d['value'], 'water', 20)
        elif d['kind']=='Q' and ((d['Q'] is None) or d['Q']==0):
            obj = vrtxs.HE2_ABC_GraphVertex()
        else:
            obj = vrtxs.HE2_Boundary_Vertex(d['kind'], d['value'])
        nodes[n] = obj

    nx.set_node_attributes(G, name='obj', values=nodes)


    for n in G.nodes():
        o = G.nodes[n]['obj']
        assert o is not None

    for u, v in G.edges():
        o = G[u][v]['obj']
        assert o is not None

    return G


def make_multigraph_schema_from_OISPipe_dataframes(df_pipes, df_boundaries):
    df = df_pipes[["node_id_start", "node_id_end"]]
    df.columns = ["source", "target"]
    G = nx.from_pandas_edgelist(df, create_using=nx.MultiDiGraph)

    cmpnts = nx.algorithms.components.number_connected_components(nx.Graph(G))
    if cmpnts != 1:
        print('Not single component graph!')
        assert False

    pipes = dict()
    for u, v, k in G.edges:
        df = df_pipes
        df = df[(df.node_id_start == u) & (df.node_id_end == v)]
        d = df.iloc[k].to_dict()

        Ls = [d['L']]
        Hs = [d['altitude_end'] - d['altitude_start']]
        Ds = [d['D'] - 2 * d['S']]
        Rs = [1e-5]

        pipe = HE2_WaterPipe(Ls, Hs, Ds, Rs)
        pipes[(u, v, k)] = pipe
    nx.set_edge_attributes(G, name='obj', values=pipes)


    df = df_boundaries[['Q', 'P']].fillna(-1e9)
    df_boundaries['value'] = df.max(axis=1)

    nodes = dict()
    id_list = list(df_boundaries.id.values)
    for n in G.nodes():
        df = df_boundaries
        if n in id_list:
            df = df[df.id == n]
            d = df.iloc[0].to_dict()
        else:
            obj = vrtxs.HE2_ABC_GraphVertex()
            nodes[n] = obj
            continue
        # if d['kind']=='P':
        #     print(d)

        if d['is_source']:
            obj = vrtxs.HE2_Source_Vertex(d['kind'], d['value'], 'water', 20)
        elif d['kind']=='Q' and ((d['Q'] is None) or d['Q']==0):
            obj = vrtxs.HE2_ABC_GraphVertex()
        else:
            obj = vrtxs.HE2_Boundary_Vertex(d['kind'], d['value'])
        nodes[n] = obj

    nx.set_node_attributes(G, name='obj', values=nodes)


    for n in G.nodes():
        o = G.nodes[n]['obj']
        assert o is not None

    for u, v, k in G.edges:
        o = G[u][v][k]['obj']
        assert o is not None

    return G
