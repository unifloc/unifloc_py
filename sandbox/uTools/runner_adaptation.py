import time
import os
import sys
sys.path.append('../../../')
import pandas as pd
import datetime
from multiprocessing import Pool
import plot_workflow.plotly_option as pltl_opt
import plot_workflow.plotly_workflow as pltl_wf
import unifloc_vba.description_generated.python_api as python_api
from preproc_p import workflow_cs_data
from preproc_p import workflow_chess_data
from preproc_p import preproc_tool
from preproc_p import workflow_calc_data
from preproc_p import workflow_tr_data
from preproc_p import filtration
from proc_p import processor as proc
from ml import calibr_restore as calibr_restore
from postproc_p import result_and_metrics as result_and_metrics




def run_calculation(thread_option_list):
    if __name__ == '__main__':
        with Pool(amount_of_threads) as p:
            p.map(proc.calc,
                  thread_option_list)
            p.close()


well_names = ['1354', '1479', '1509', '1540', '1567', '1602', '1628',
              '202', '252', '326', '353', '507', '540', '569', '570', '601',
              '627', '658', '689', '693']
amount_of_threads = 12
tr_name = "Техрежим, , февраль 2019.xls"


def massive_adaptation(well_name):
    """
    Запуск адаптации скважины
    """
    dir_name_with_input_data = 'adapt_input_'
    start_time = time.time()
    log = 'Запуск восстановления дебитов \n'
    log += ('Время начало расчета: ' + str(datetime.datetime.today()) + '\n')

    log += 'Расчет скважины ' + well_name + '\n'
    try:
        start_time_in_loop = time.time()
        thread_option_list = proc.create_thread_list(well_name, dir_name_with_input_data, tr_name,
                                                     amount_of_threads)
        run_calculation(thread_option_list)
        end_time_in_loop = time.time()
        log += 'Затрачено времени на скважину ' + well_name + '  ' + str(
            end_time_in_loop - start_time_in_loop) + '\n'
    except:
        log += 'Ошибка какая-то в скважине ' + well_name + '\n'
        pass
    end_time = time.time()
    log += 'Время завершения расчета: ' + str(datetime.datetime.today()) + '\n'
    log += 'Затрачено времени всего, часов' + '  ' + str((end_time - start_time)/3600) + '\n'
    text_file = open('log_massive_adaptation.txt', "a")
    text_file.write(log)
    text_file.close()


def general_runner(func, well_names):
    """
    Функция для запуска функций
    """
    for well_name in well_names:
        try:
            print('Работа со скважиной ' + well_name)
            func(well_name)
        except:
            print('Ошибка со скважиной ' + well_name)


general_runner(massive_adaptation, well_names)
