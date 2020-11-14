import networkx as nx
import numpy as np
from HE2_ABC import Root
import HE2_tools as tools
from HE2_Solver import HE2_Solver


def evalute_network_fluids_wo_root(_G, x_dict):
    assert not (Root in _G.nodes)

    for u, v in _G.edges:
        assert x_dict[(u, v)] >= 0
    G = nx.DiGraph(_G)
    for (u, v) in x_dict:
        if x_dict[(u, v)] == 0:
            G.remove_edge(u, v)
    for node in _G.nodes:
        if len(G[node]) == 0:
            G.remove_node(node)

    nodes = list(G.nodes)
    edges = list(G.edges)
    EN = len(edges)
    N = len(nodes)

    x = np.array([x_dict[e] for e in edges])
    A = -1 * nx.incidence_matrix(G, nodelist=nodes, edgelist=edges, oriented=True).toarray()
    Q = np.matmul(A, x)
    var_idx = {e:i for i, e in enumerate(edges)}
    sinks = [n for i, n in enumerate(nodes) if Q[i] < 0]
    var_idx.update({n:i+EN for i, n in enumerate(sinks)})
    M = len(var_idx)
    mx1 = np.zeros((N, M)) # the first partition of matrix is for 1stCL
    mx2 = np.zeros((M-N, M)) # the second partition is to dictate condition: all fluids leaving one node have to be the same
    i2 = 0
    for i, node in enumerate(nodes):
        these_vars_are_the_same = []
        if Q[i] < 0: # n is sink, so there is an unknown (equation variable) for outbound flow
            j = var_idx[node] # get the unknown index in matrix
            mx1[i, j] = Q[i]
            these_vars_are_the_same += [j] # remember the unknown for a while

        for u, v in G.in_edges(node):  # inlet flows
            j = var_idx[(u, v)]
            mx1[i, j] = x_dict[(u, v)]

        for u, v in G.out_edges(node): # all the outlet flows are the same
            j = var_idx[(u, v)]
            mx1[i, j] = -x_dict[(u, v)]
            these_vars_are_the_same += [j] # so we have to remmeber this unknown too

        if Q[i] > 0:
            mx1[i] /= Q[i] # cause we need only ones and zeros on the right side

        if len(these_vars_are_the_same) > 1: # if there are more than one remembered outlet flow
            for fl1, fl2 in zip(these_vars_are_the_same[1:], these_vars_are_the_same[:-1]):
                mx2[i2, fl1] = 1  # we have to right an equation to matrix, to describe that these outlet fluids are equal
                mx2[i2, fl2] = -1
                i2 += 1
    mx = np.append(mx1, mx2, axis=0)
    assert mx.shape == (M, M)

    mx_inv = np.linalg.inv(mx)
    rez_mx = np.zeros((M, N))
    srcs = []
    S = 0
    for i, n in enumerate(nodes):
        if Q[i] > 0:
            rez_mx[:, S] = mx_inv[:,i]
            srcs += [n]
            S += 1

    rez = {}
    for k, i in var_idx.items():
        rez[k] = rez_mx[i, :S]
        np.testing.assert_almost_equal(abs(sum(rez[k])), 1)
    return rez, srcs



if __name__ == '__main__':
    G0, nodes = tools.generate_random_net_v0(N=7, E=9, SNK=2, randseed=424242)
    solver = HE2_Solver(G0)
    solver.solve()
    x_dict = dict()
    G = nx.DiGraph()
    G.add_nodes_from(G0)
    for u, v in G0.edges:
        x = G0[u][v]['obj'].result['x']
        if x < 0:
            x_dict[(v,u)] = -x
            G.add_edge(v, u)
        else:
            x_dict[(u, v)] = x
            G.add_edge(u, v)
    rez = evalute_network_fluids_wo_root(G, x_dict)
    print(rez)

