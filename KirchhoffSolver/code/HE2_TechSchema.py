import json
import os
import pandas as pd
from collections import Counter


def get_global_id_for(entity_class):
    if 'ID' not in get_global_id_for.__dict__:
        get_global_id_for.ID = 0
    rez = get_global_id_for.ID
    get_global_id_for.ID += 1
    return rez


class HE2_Object():
    def __init__(self, obj_id=None, name='', description='', inlets=None, outlets=None, p_link=None):
        if obj_id is None:
            obj_id = get_global_id_for('Object')
        self.id = obj_id
        self.name = name
        self.description = description
        self.inlets = [get_global_id_for('Inlet')]
        if inlets:
            self.inlets += inlets
        self.outlets = [get_global_id_for('Outlet')]
        if outlets:
            self.outlets += outlets


class HE2_Tech_Schema():
    def __init__(self):
        self.objects_by_id = {}
        self.objects_by_full_name = {}
        self.containing_tree = {}
        self.graph = None
        self.plain_obj_list = []

    def get_object_by_id(self, id):
        return self.objects_by_id.get(id, None)

    def get_object_by_full_name(self, id):
        pass

    def get_containing_tree(self, root=None):
        pass

    def get_objects_graph(self):
        pass

    def get_sub_schema(self, id_list=None):
        pass


class HE2_Computational_Task():
    def __init__(self, schema, fluids, boundaries):
        self.schema = schema
        self.fluids = fluids
        self.boundaries = boundaries


class HE2_Schema_Persister():
    def __init__(self):
        pass

    def build_task_from_file(self, filename):
        name, ext = os.path.splitext(filename)
        if ext[0:4] in ('.xls', '.xlsx'):
            pipes_colnames = ['line_name', 'node1_name', 'node2_name', 'pipe_num', 'L', 'D', 'Wall', 'Rough']
            nodes_colnames = ['node_name', 'H', 'Q_out', 'Q_in', 'P_out', 'P_in', 'Water_gravity', 'Water_viscosity']
            df_pipes = pd.read_excel(filename, sheet_name=0, header=None, names=pipes_colnames, skiprows=range(9))
            df_nodes = pd.read_excel(filename, sheet_name=1, header=None, names=nodes_colnames, skiprows=range(3))
            print(df_pipes.head(20))
            print(df_nodes.head(20))
            rez = self.build_task_from_dataframes(df_nodes, df_pipes)
            return rez
        elif ext in ('.txt', '.json'):
            f = open(filename, 'r', encoding='UTF-8')
            ts_json = json.load(f)
            rez = self.build_task_from_json(ts_json)
            return rez

    def build_task_from_json(self, ts_json):
        pass

    def build_task_from_dataframes(self, nodes_df, pipes_df):
        schema =  HE2_Tech_Schema()
        obj_list = []
        for idx in nodes_df.index:
            name = nodes_df.node_name[idx]
            obj = HE2_Object(obj_id=None, name=name)
            obj_list += [obj]
        schema.plain_obj_list += obj_list
        schema.objects_by_id = {o.id: o for o in obj_list}
        cntr = Counter([o.name for o in obj_list])
        mc = cntr.most_common(1)[0]
        if mc[1] > 1:
            assert(False, 'What I have to do with objects with same names?', mc)
        schema.objects_by_full_name = {o.name: o for o in obj_list}

        by_name = schema.objects_by_full_name
        obj_list = []
        conn_list = []
        for idx in pipes_df.index:
            name = pipes_df.line_name[idx]
            obj1 = by_name[pipes_df.node1_name[idx]]
            obj2 = by_name[pipes_df.node2_name[idx]]
            obj = HE2_Object(obj_id=None, name=name)
            conn_list += [(obj1.outlets[0], obj.inlets[0]), (obj.outlets[0], obj2.inlets[0])]
            obj_list += [obj]

        return schema


    def dump_tech_schema_to_json(self, tech_schema):
        pass

    def dump_tech_schema_to_dataframes(self, tech_schema):
        pass


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    # def_fn = r'..\data\tech_schema.json'
    def_fn = r'..\data\waterpipes.xlsx'
    parser.add_argument("-f", "--filename", type=str, required=False, help="Techshema json file", default=def_fn)
    args = parser.parse_args()
    filename = args.filename
    persist = HE2_Schema_Persister()
    tech_schema = persist.build_tech_shema_from_file(filename)
    print(tech_schema)
