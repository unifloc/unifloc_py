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

import app_calc_massive


amount_of_threads = 12

def run_calculation(thread_option_list):
    if __name__ == '__main__':
        with Pool(amount_of_threads) as p:
            p.map(proc.calc,
                  thread_option_list)
            p.close()

well_names = ['1354', '1479', '1509', '1540', '1567', '1602', '1628',
              '202', '252', '326', '353', '507', '540', '569', '570', '601',
              '627', '658', '689', '693']


import subprocess

app_calc_massive.general_runner(app_calc_massive.generate_adaptation_input, well_names)

subprocess.call(['C:\\ProgramData\\Anaconda3\\python.exe', 'C:\\Users\\olegk\\Git\\unifloc\\sandbox\\uTools\\runner_adaptation.py'])

app_calc_massive.general_runner(app_calc_massive.load_adaptation_data_and_generate_restore_data, well_names)

subprocess.call(['C:\\ProgramData\\Anaconda3\\python.exe', 'C:\\Users\\olegk\\Git\\unifloc\\sandbox\\uTools\\app_calc_massive.py'])

app_calc_massive.general_runner(app_calc_massive.final_step, well_names)

