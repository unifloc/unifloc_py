"""
Кобзарь О.С. 14.01.2020

Модуль для запуска работ в цикле. Является перенесенной копией app.ipynb, возможно отставание в плане функционала
"""

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

auto_open_html = False
tr_name = "Техрежим, , февраль 2019.xls"
amount_of_threads = 12
well_names = ['1354', '1479', '1509', '1540', '1567', '1602', '1628',
              '202', '252', '326', '353', '507', '540', '569', '570', '601',
              '627', '658', '689', '693']
#well_names = ['1354']#, '1479']


def massive_adaptation(well_name):
    """
    Запуск адаптации скважины
    """
    dir_name_with_input_data = 'adapt_input_'
    start_time = time.time()
    log = 'Запуск восстановления дебитов \n'
    log += ('Время начало расчета: ' + str(datetime.datetime.today()) + '\n')

    log += 'Расчет скважины ' + well_name  + '\n'
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


def massive_restore_by_calibr(well_name):
    """
    Запуск восстановление дебитов скважины
    """
    dir_name_with_input_data = 'restore_input_'
    start_time = time.time()
    log = 'Запуск восстановления дебитов \n'
    log += 'Время начало расчета: ' + str(datetime.datetime.today()) + '\n'
    log += 'Расчет скважины ' + well_name  + '\n'
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
    text_file = open('log_massive_restore_by_calibr.txt', "a")
    text_file.write(log)
    text_file.close()


def solve_path(well_name):
    """
    Функция для решения проблем с путями и настройками, вызывается во всех других функциях
    """
    chess_file_name = 'Скв. ' + well_name + ' (01.08.2018-28.02.2019).xls'
    left_boundary = [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 29)]
    right_boundary = [datetime.datetime(2018, 11, 5), datetime.datetime(2019, 2, 28)]

    current_path = os.getcwd()
    time_mark = ''  # datetime.datetime.today().strftime('%Y_%m_%d_%H_%M_%S')
    path_to_data = current_path + "\\data\\"
    path_to_work_dir = current_path + "\\data\\" + well_name + "\\"
    save_dir_name = 'init_edit'
    path_to_save = path_to_work_dir + save_dir_name + '\\'
    dirnames_list = []
    for (dirpath, dirnames, filenames) in os.walk(path_to_data):
        dirnames_list.extend(dirnames)
        break
    print(dirnames_list)
    cs_data_filename = path_to_save + well_name + "_first_edit.csv"
    return current_path, path_to_data, path_to_work_dir, save_dir_name, path_to_save, cs_data_filename, time_mark, chess_file_name, left_boundary, right_boundary


def cs_data_init_edit(well_name):
    """
    Функция для первоначальной обработки данных со СУ
    """
    current_path, path_to_data, path_to_work_dir, save_dir_name, path_to_save, cs_data_filename, time_mark, chess_file_name, left_boundary, right_boundary = solve_path(well_name)
    try:
        os.mkdir(path_to_work_dir + save_dir_name)
    except:
        pass
    well_data = workflow_cs_data.read_and_format_good_tm_data(well_name, path_to_work_dir)
    well_data.to_csv(path_to_save + well_name + "_first_edit.csv")
    well_data['ГФ, м3/м3'] = well_data['Объемный дебит газа'] / well_data['Объемный дебит нефти']
    all_banches = pltl_opt.create_banches_for_report(well_data, report_type = 'init_cs_data')
    pltl_wf.create_report_html(well_data, all_banches, path_to_save + well_name + "_first_edit_report.html",  auto_open = auto_open_html)
    del well_data['ГФ, м3/м3']


def chess_data_init_edit(well_name):
    """
    Функция для обработки данных с шахматки (просмотра)
    """
    current_path, path_to_data, path_to_work_dir, save_dir_name, path_to_save, cs_data_filename, time_mark, chess_file_name, left_boundary, right_boundary = solve_path(well_name)
    chess_data = workflow_chess_data.load_and_edit_chess_data(path_to_work_dir + chess_file_name,
                                                              '1d',
                                                              without_changing=True)  # TODO сделать протяжку вверх, для буферного давления

    all_banches = pltl_opt.create_banches_for_report(chess_data, report_type='init_chess_data')
    pltl_wf.create_report_html(chess_data, all_banches, path_to_save + well_name + "_chess_report.html",
                               auto_open=auto_open_html)


def generate_adaptation_input(well_name):
    """
    Функция для генерации данных для адаптации
    """
    current_path, path_to_data, path_to_work_dir, save_dir_name, path_to_save, cs_data_filename, time_mark, chess_file_name, left_boundary, right_boundary = solve_path(well_name)
    time_to_resamle = '3h'
    created_input_data_type = 0
    edited_data_cs = workflow_cs_data.load_and_edit_cs_data(cs_data_filename,
                                                            created_input_data_type,
                                                            time_to_resamle)
    edited_data_cs = filtration.get_filtred_by_sigma(edited_data_cs)  # TODO добавить сюда фильтрацию по времени замера
    chess_data = workflow_chess_data.load_and_edit_chess_data(path_to_work_dir + chess_file_name,
                                                              time_to_resamle)
    input_data_dir_name = 'adapt_input_' + time_mark
    path_to_input_data = path_to_work_dir + input_data_dir_name + '\\'
    try:
        os.mkdir(path_to_work_dir + input_data_dir_name)
    except:
        pass
    created_input_data = edited_data_cs.join(chess_data, how='inner')  # TODO протягивать буферное давление вверх
    created_input_data = created_input_data.dropna(subset=['Объемный дебит жидкости (СУ)'])
    created_input_data = preproc_tool.cut_df(created_input_data, left_boundary, right_boundary)
    created_input_data = preproc_tool.check_input_data(created_input_data)
    created_input_data.to_csv(path_to_input_data + well_name + '_adapt_input.csv')
    plot_file_path = path_to_input_data + well_name + '_adapt_input.html'
    input_data_traces = pltl_wf.create_traces_list_for_all_columms(created_input_data, 'lines+markers', use_gl=True)
    pltl_wf.plot_subplots(input_data_traces, plot_file_path, True, auto_open=auto_open_html)
    all_banches = pltl_opt.create_banches_for_report(created_input_data, report_type='adapt_input')

    pltl_wf.create_report_html(created_input_data, all_banches,
                               path_to_input_data + well_name + '_adapt_input_report.html', auto_open=auto_open_html)


def load_adaptation_data_and_generate_restore_data(well_name): #TODO сделать многоваритвным
    """
    Функция для загрузки данных адаптации и генерации данных для восстановления дебитов
    """
    current_path, path_to_data, path_to_work_dir, save_dir_name, path_to_save, cs_data_filename, time_mark, chess_file_name, left_boundary, right_boundary = solve_path(well_name)
    dir_name_with_input_data = 'adapt_input_' + '\\'
    input_data_file_name = well_name + '_adapt_input'
    dir_name_with_calculated_data = 'adaptation_' + '\\'
    calculated_data_file_name = well_name + '_adapt_1'
    first_result_data = preproc_tool.combine_multiprocessing_result(path_to_work_dir, dir_name_with_calculated_data)
    first_result_data.to_csv(path_to_work_dir + dir_name_with_calculated_data + calculated_data_file_name + '.csv')
    calculated_data = workflow_calc_data.load_calculated_data_from_csv(
        path_to_work_dir + dir_name_with_calculated_data +
        calculated_data_file_name + '.csv')
    input_data = pd.read_csv(path_to_work_dir + dir_name_with_input_data + input_data_file_name + '.csv')
    input_data.index = input_data['Время']
    del input_data['Время']
    all_data = input_data.join(calculated_data, how='outer')
    all_data.to_csv(path_to_work_dir + dir_name_with_calculated_data + well_name + '_calc_and_input' + '.csv')
    adapt_data_traces = pltl_wf.create_traces_list_for_all_columms(all_data, 'lines+markers', use_gl=True)
    plot_file_path = path_to_work_dir + dir_name_with_calculated_data + well_name + '_calc_and_input' + '.html'
    pltl_wf.plot_subplots(adapt_data_traces, plot_file_path, True, auto_open=auto_open_html)
    all_banches = pltl_opt.create_banches_for_report(all_data, report_type='adapt_report')

    pltl_wf.create_report_html(all_data, all_banches, path_to_work_dir + dir_name_with_calculated_data +
                               well_name + '_adapt_report.html', auto_open=auto_open_html)
    calibr_data = calculated_data[['К. калибровки по напору - множитель (Модель)',
                                   'К. калибровки по мощности - множитель (Модель)']]
    calibr_data = preproc_tool.mark_df_columns(calibr_data, 'Подготовленные')
    # calibr_data = calibr_data.resample('3h').mean()
    # calibr_data = calibr_data.interpolate()
    p_out_lol, f_out_lol = calibr_restore.restore_calibr_via_ridge(all_data, calibr_data,,
    time_to_resamle = '3h'
    created_input_data_type = 1  # костыль для 252 и 693 скважины
    edited_data_cs = workflow_cs_data.load_and_edit_cs_data(cs_data_filename, created_input_data_type,
                                                            time_to_resamle
                                                            )
    chess_data = workflow_chess_data.load_and_edit_chess_data(path_to_work_dir + chess_file_name,
                                                              time_to_resamle)
    edited_data_cs = filtration.get_filtred_by_sigma(edited_data_cs)  # TODO добавить сюда фильтрацию по времени замера
    result = preproc_tool.make_gaps_and_interpolate(calibr_data)
    calibr_data = result.copy()
    # all_data = all_data.join(calibr_data, how = 'inner')
    # created_input_data = all_data.copy()
    calibr_data['К. калибровки по мощности - множитель (Модель) (Подготовленные)'] = p_out_lol
    calibr_data['К. калибровки по напору - множитель (Модель) (Подготовленные)'] = f_out_lol
    calibr_data['К. калибровки по мощности - множитель (Модель) (При адаптации)'] = calculated_data[
        'К. калибровки по напору - множитель (Модель)']
    calibr_data['К. калибровки по напору - множитель (Модель) (При адаптации)'] = calculated_data[
        'К. калибровки по мощности - множитель (Модель)']
    calibr_data = calibr_data[calibr_data['К. калибровки по мощности - множитель (Модель) (Подготовленные)'] !=
                              calculated_data['К. калибровки по мощности - множитель (Модель)']]
    input_data_dir_name = 'restore_input_' + time_mark
    path_to_input_data = path_to_work_dir + input_data_dir_name + '\\'
    try:
        os.mkdir(path_to_work_dir + input_data_dir_name)
    except:
        pass
    created_input_data = edited_data_cs.join(chess_data, how='inner')
    created_input_data = created_input_data.join(calibr_data, how='inner')
    created_input_data = preproc_tool.cut_df(created_input_data, left_boundary, right_boundary)
    created_input_data.to_csv(path_to_input_data + well_name + '_restore_input.csv')
    plot_file_path = path_to_input_data + well_name + '_restore_input.html'
    input_data_traces = pltl_wf.create_traces_list_for_all_columms(created_input_data, 'lines+markers', use_gl=True)
    pltl_wf.plot_subplots(input_data_traces, plot_file_path, True, auto_open=auto_open_html)
    all_banches = pltl_opt.create_banches_for_report(created_input_data, report_type='restore_input')

    pltl_wf.create_report_html(created_input_data, all_banches, path_to_input_data +
                               well_name + '_restore_input_report.html', auto_open=auto_open_html)


def final_step(well_name):
    """
    Функция для анализа данных восстановления, сведения адаптации и восстановления, создания общего отчета, метрик
    """
    current_path, path_to_data, path_to_work_dir, save_dir_name, path_to_save, cs_data_filename, time_mark, chess_file_name, left_boundary, right_boundary = solve_path(well_name)
    dir_name_with_input_data = 'restore_input_' + '\\'
    input_data_file_name = well_name + '_restore_input'
    dir_name_with_calculated_data = 'restore_' + '\\'
    calculated_data_file_name = well_name + '_restore_1'

    first_result_data = preproc_tool.combine_multiprocessing_result(path_to_work_dir, dir_name_with_calculated_data)
    first_result_data.to_csv(path_to_work_dir + dir_name_with_calculated_data + calculated_data_file_name + '.csv')
    calculated_data = workflow_calc_data.load_calculated_data_from_csv(path_to_work_dir + dir_name_with_calculated_data +
                                                    calculated_data_file_name +  '.csv')
    input_data = pd.read_csv(path_to_work_dir + dir_name_with_input_data + input_data_file_name +  '.csv', parse_dates = True, index_col = 'Время')
    all_data = input_data.join(calculated_data, how = 'outer')
    all_data.to_csv(path_to_work_dir + dir_name_with_calculated_data + well_name + '_calc_and_input' +  '.csv' )
    input_data_traces = pltl_wf.create_traces_list_for_all_columms(all_data, 'lines+markers', use_gl = True)
    plot_file_path = path_to_work_dir + dir_name_with_calculated_data + well_name + '_calc_and_input' +  '.html'
    pltl_wf.plot_subplots(input_data_traces, plot_file_path, True,  auto_open = auto_open_html)

    path_to_adapt_dir = 'adaptation_' + '\\'
    path_to_restore_dir = 'restore_' + '\\'
    adapt_data_with_input = pd.read_csv(path_to_work_dir + path_to_adapt_dir + well_name + '_calc_and_input' + '.csv' , parse_dates = True, index_col = 'Время')
    adapt_data_with_input = preproc_tool.mark_df_columns(adapt_data_with_input, 'ADAPT')
    restore_data_with_input = pd.read_csv(path_to_work_dir + path_to_restore_dir + well_name + '_calc_and_input' + '.csv' , parse_dates = True, index_col = 'Время')
    restore_data_with_input = preproc_tool.mark_df_columns(restore_data_with_input, 'RESTORE')
    overall_data = adapt_data_with_input.join(restore_data_with_input, how = 'outer')
    q_liq = overall_data[['Объемный дебит жидкости (СУ) (ADAPT)', 'Активная мощность (СУ) (ADAPT)']]
    result = preproc_tool.make_gaps_and_interpolate(q_liq)
    result = preproc_tool.mark_df_columns(result, 'INTERP')
    overall_data = overall_data.join(result)
    overall_data = result_and_metrics.final_edit_overall_data(overall_data)
    all_banches = pltl_opt.create_banches_for_report(overall_data, report_type = 'overall_result')
    plot_file_path = path_to_work_dir + path_to_restore_dir + well_name + '_adapt_and_restore_report' +  '.html'
    pltl_wf.create_report_html(overall_data, all_banches, plot_file_path,  auto_open = auto_open_html)
    calibr_calc_metrics, interp_calc_metrics = result_and_metrics.calc_calibr_interp_metrics(overall_data)
    metrics_text_file_path = path_to_work_dir + path_to_restore_dir + well_name + '_adapt_and_restore_metrics_report' +  '.txt'
    text_file = open(metrics_text_file_path, "w")
    text_file.write(calibr_calc_metrics + '\n' + interp_calc_metrics)
    text_file.close()

    tr_file_full_path = os.getcwd() + '\\data\\tr\\' + tr_name
    tr_data = workflow_tr_data.read_tr_and_get_data(tr_file_full_path, well_name)
    tr_data_df = pd.DataFrame({'Параметры скважины с ТР': list(tr_data.__dict__.values())})
    tr_data_df.index = list(tr_data.__dict__.keys())
    overall_metrics = result_and_metrics.calc_calibr_interp_metrics(overall_data, return_df = True)

    overall_metrics['ЭЦН'] = [tr_data.esp_name_str, tr_data.esp_name_str]
    overall_metrics['ПЭД'] = [tr_data.motor_name_str, tr_data.motor_name_str]
    overall_metrics['Кол-во точек (ADAPT)'] = [len(overall_data['К. калибровки по напору - множитель (Модель) (ADAPT)']),
                                               len(overall_data['К. калибровки по напору - множитель (Модель) (ADAPT)'])]
    overall_metrics['MAE/Q ж, м3/сут (Модель) (ADAPT) mean, %'] = overall_metrics['Mean absolute error'] / overall_metrics['Q ж, м3/сут (Модель) (ADAPT) mean'] * 100
    overall_metrics.index = [well_name + ' (CALIBR)' , well_name + ' (INTERP)']
    overall_metrics.to_csv(path_to_work_dir + path_to_restore_dir + well_name + '_adapt_restore_metrics_tr_report' +  '.csv')
    overall_metrics.head()
    q_esp_nom_m3day = tr_data.esp_nom_rate_m3day
    head_esp_nom_m = tr_data.esp_nom_head_m


    path_to_addin = os.getcwd()
    path_to_addin = path_to_addin.replace('unifloc\\sandbox\\uTools', 'unifloc_vba\\UniflocVBA_7.xlam')
    UniflocVBA = python_api.API(path_to_addin)
    esp_traces = pltl_wf.create_esp_traces(UniflocVBA, q_esp_nom_m3day, head_esp_nom_m)
    overall_data_dimensionless = result_and_metrics.make_dimensionless_df(overall_data)
    filename = path_to_work_dir + path_to_restore_dir + well_name + '_dimless_pump_heatmap_report' + '.html'
    pltl_wf.create_overall_report(overall_data, overall_data_dimensionless, esp_traces, filename,  auto_open = auto_open_html)


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



#cs_data_init_edit, chess_data_init_edit, generate_adaptation_input, massive_adaptation,
# load_adaptation_data_and_generate_restore_data, massive_restore_by_calibr, final_step

#general_runner(generate_adaptation_input, well_names)
#general_runner(massive_adaptation, well_names)
#general_runner(load_adaptation_data_and_generate_restore_data, well_names)
general_runner(massive_restore_by_calibr, well_names)
#general_runner(final_step, well_names)

