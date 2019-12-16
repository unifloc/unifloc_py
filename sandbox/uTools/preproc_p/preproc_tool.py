import os
import pandas as pd


def combine_multiprocessing_result(path_to_work_dir, dir_name_with_calculated_data):
    filenames_list = []
    for (dirpath, dirnames, filenames) in os.walk(path_to_work_dir + dir_name_with_calculated_data + 'multiprocessing\\'):
        filenames_list.extend(filenames)
        break
    print(filenames_list)
    for i, j in enumerate(filenames_list):
        if i == 0:
            first_result_data = pd.read_csv(path_to_work_dir + dir_name_with_calculated_data + 'multiprocessing\\' + j,
                                            parse_dates=True, index_col='Время')
        else:
            another_result_data = pd.read_csv(path_to_work_dir + dir_name_with_calculated_data + 'multiprocessing\\' + j,
                                              parse_dates=True, index_col='Время')
            first_result_data = first_result_data.append(another_result_data, sort=True)
            del first_result_data['d']
            first_result_data = first_result_data.dropna(subset=['ESP.ESPpump.EffiencyESP_d'])
    first_result_data = first_result_data.sort_index()
    return first_result_data
