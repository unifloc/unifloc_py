import pandas as pd
import numpy as np
import scipy
from scipy import integrate


import sys
sys.path.append('../')
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

gn = preproc_tool.GlobalNames()

fsolve = scipy.optimize.fsolve
this_time_delta = pd.to_timedelta(1, unit = 'hour')

def calculated_regime_time(this_file, regime_column=gn.i_a_motor_a, return_all=False):
    this_file = this_file.dropna(subset=[regime_column])
    last_time = this_file.index[0]

    work_status = []
    if this_file[regime_column][0] > 0:
        work_status.append(1)
    else:
        work_status.append(0)

    work_timedelta = []
    stop_timedelta = []
    regime_bounds = []
    work_bounds = []
    stop_bounds = []

    for i in range(1, this_file.shape[0]):
        this_time = this_file.index[i]
        this_value = this_file[gn.i_a_motor_a][i]
        if this_value > 0:
            work_status.append(1)
        else:
            work_status.append(0)
        # print(this_value)
        time_delta = this_time - last_time
        # time_delta = this_file.index[i-1] - last_time
        if work_status[-1] != work_status[-2]:

            # print('Переключение')
            if work_status[-1] == 1:
                # print('Включение')
                stop_timedelta.append(time_delta.total_seconds() / 60)
                stop_bounds.append([last_time, this_time])
            else:
                # print('Выключение')
                work_timedelta.append(time_delta.total_seconds() / 60)
                work_bounds.append([last_time, this_time])

            last_time = this_time

    if work_status[-1] == 1:
        # print('Последний интервал работала')
        work_timedelta.append(time_delta.total_seconds() / 60)
        work_bounds.append([last_time, this_time])
    else:
        # print('Последний интервал не работала')
        work_timedelta.append(time_delta.total_seconds() / 60)
        work_bounds.append([last_time, this_time])

    work_time_median = np.median(work_timedelta)
    stop_timedelta_median = np.median(stop_timedelta)
    if not return_all:
        return work_time_median, stop_timedelta_median
    else:
        return work_status, work_timedelta, stop_timedelta, work_bounds, stop_bounds


def calc_integral(series, return_result_series = False):
    this_delta = series.index[-1] - series.index[0] + (series.index[1] - series.index[0])
    this_delta_to_rolling = this_delta
    result_integral = series.rolling(this_delta_to_rolling).apply(integrate.trapz)
    if return_result_series:
        return result_integral
    else:
        return result_integral[-1]


def get_new_df_with_instant_rate(df, nedeed_value = None):
    super_small_df = df.copy()
    def func_for_fsolve_inner(multiply, nedeed_value = None):
        #print(multiply)
        super_small_df['instant_rate'] = super_small_df[gn.work_status_number] * multiply
        new_integral_value = calc_integral(super_small_df['instant_rate'])
        #print(f"new_integral_value: {new_integral_value}")
        super_small_df[gn.q_liq_m3day + ' (created)'] = super_small_df['instant_rate'] * 24 * 60
        return nedeed_value - new_integral_value
    result = fsolve(func_for_fsolve_inner, x0 = 1, args=[nedeed_value])
    return super_small_df, result