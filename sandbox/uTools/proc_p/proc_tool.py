import os
import sys
import time
import xlwings as xw
sys.path.append('../'*4)


def create_directories(vfm_calc_option, app_path, well_name, options, dir_name_with_input_data, time_mark):
    """
    Функция для определения путей и создания директорий
    """
    if not vfm_calc_option:  # создание директорий для результатов расчета
        input_data_filename_str = app_path + '\\data\\' + well_name + '\\' + dir_name_with_input_data + '\\' + well_name + '_adapt_input'
        dir_to_save_calculated_data = app_path + '\\data\\' + well_name + '\\' + 'adaptation_' + time_mark
        try:
            os.mkdir(dir_to_save_calculated_data)
        except:
            pass
    else:
        input_data_filename_str = app_path + '\\data\\' + well_name + '\\' + dir_name_with_input_data + '\\' + well_name + '_restore_input'
        dir_to_save_calculated_data = app_path + '\\data\\' + well_name + '\\' + 'restore_' + time_mark
        try:
            os.mkdir(dir_to_save_calculated_data)
        except:
            pass
    if options.multiprocessing:
        dir_to_save_calculated_data += '\\' + 'multiprocessing'
        try:
            os.mkdir(dir_to_save_calculated_data)
        except:
            pass
    return input_data_filename_str, dir_to_save_calculated_data


def auto_restart(i,options, UniflocVBA, current_path):
    check = i % options.amount_iters_before_restart
    if check == 0 and i != 0:  # защита против подвисаний экселя - не работает в многопотоке
        print('Перезапуск Excel и VBA')
        UniflocVBA.book.close()
        time.sleep(options.sleep_time_sec)
        UniflocVBA.book = xw.Book(current_path + options.addin_name)

