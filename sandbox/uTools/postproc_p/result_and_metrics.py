"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль динамического анализа исходных данных и рассчитанных значений
"""
from sklearn import metrics
import pandas as pd
import sys
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool
global_names = preproc_tool.GlobalNames()
from scipy import integrate

def relative_error_perc(y1, y2):
    """
    Расчет относительной ошибки
    :param y1: фактическое значение
    :param y2: прогнозное значения
    :return: относительная ошибка, %
    """
    return (y1 - y2) / y1 * 100


def calc_mertics(y_fact, y_pred): #TODO добавить MAE чтобы было относительное
    """
    Расчет метрик модели
    :param y_fact: фактические значения
    :param y_pred: прогнозные значения
    :return: набор метрик в виде DataFrame
    """
    df = {}
    df['R2'] = [metrics.r2_score(y_fact, y_pred)]
    df['Mean absolute error'] = [metrics.mean_absolute_error(y_fact, y_pred)]
    df['Mean squared error'] = [metrics.mean_squared_error(y_fact, y_pred)]
    df['Root mean squared error'] = [metrics.mean_squared_error(y_fact, y_pred) ** 0.5]
    df['Max error'] = [metrics.max_error(y_fact, y_pred)]
    df['Median absolute error'] = [metrics.median_absolute_error(y_fact, y_pred)]
    df['explained_variance_score'] = [metrics.explained_variance_score(y_fact, y_pred)]
    df['mean_squared_log_error'] = [metrics.mean_squared_log_error(y_fact, y_pred)]
    df = pd.DataFrame(df)
    return df


def add_relative_errors_to_overall_data(df, gn = global_names):
    """
    Финальная обработка сведенных данных адаптации и восстановления
    :param df: DataFrame с адаптацией, восстановлением при помощи калибровок + линейной интерполяцией
    :return: df с поправленными размерностями данных и относительными ошибками по дебиту
            и мощности (для калибровок и интерполяции)
    """
    df[gn.relative_error_active_power_perc] = relative_error_perc(df[gn.active_power_kwt + " (ADAPT)"],
                                                                  df[gn.active_power_kwt + " (PREDICTION)"]).abs()
    df[gn.relative_error_q_liq_perc] = relative_error_perc(df[gn.q_liq_m3day + " (ADAPT)"],
                                                           df[gn.q_liq_m3day + " (PREDICTION)"]).abs()
    df[gn.relative_error_head_calibr_perc] = relative_error_perc(df[gn.c_calibr_head_d + " (ADAPT)"],
                                                                 df[gn.c_calibr_head_d + " (PREDICTION)"]).abs()
    df[gn.relative_error_power_calibr_perc] = relative_error_perc(df[gn.c_calibr_power_d + " (ADAPT)"],
                                                                  df[gn.c_calibr_power_d + " (PREDICTION)"]).abs()
    return df


def calc_cumulative_oil_production_m3(overall_data, production_column_name, time_to_resample, divide_q=24):
    production_series = overall_data[production_column_name]
    production_series_m3h = production_series / divide_q
    production_series_m3h = production_series_m3h.resample(time_to_resample).mean()
    production_series_m3h = production_series_m3h.interpolate('linear')
    cumulative_production = integrate.trapz(production_series_m3h.values)
    return cumulative_production


def make_result(overall_data: pd.DataFrame, gn = global_names, time_to_resample = '1h'):  #TODO оставить только df с методом to_string или excel, удалить str мусор
    """
    Итоговый расчет метрик
    :param overall_data: сведенные данные адаптации и восстановления на весь период.
                         Преобразуется к overall_data_with_calibr_gaps - df только с тестовыми (прогнозными) значениями
    :param return_df: bool - флаг для возврата DataFrame
    :return:
    """
    overall_data_with_calibr_gaps = overall_data.dropna(subset=[gn.q_liq_m3day + " (PREDICTION)"])

    calibr_calc_metrics = calc_mertics(overall_data_with_calibr_gaps[gn.q_liq_m3day + " (ADAPT)"],
                                             overall_data_with_calibr_gaps[gn.q_liq_m3day + " (PREDICTION)"])

    calibr_calc_metrics[gn.relative_error_active_power_perc] = [
        overall_data_with_calibr_gaps[gn.relative_error_active_power_perc].abs().mean()]
    calibr_calc_metrics[gn.relative_error_q_liq_perc] = [
        overall_data_with_calibr_gaps[gn.relative_error_q_liq_perc].abs().mean()]
    calibr_calc_metrics[gn.relative_error_head_calibr_perc] = [
        overall_data_with_calibr_gaps[gn.relative_error_head_calibr_perc].abs().mean()]
    calibr_calc_metrics[gn.relative_error_power_calibr_perc] = [
        overall_data_with_calibr_gaps[gn.relative_error_power_calibr_perc].abs().mean()]
    calibr_calc_metrics[gn.q_liq_m3day + ' (MEAN)'] = [
        overall_data_with_calibr_gaps[gn.q_liq_m3day].abs().mean()]
    calibr_calc_metrics[f"MAE/{gn.q_liq_m3day} (MEAN)"] = calibr_calc_metrics['Mean absolute error'] / \
        calibr_calc_metrics[gn.q_liq_m3day + ' (MEAN)'] * 100
    calibr_calc_metrics["Накопленная добыча жидкости, м3 (ADAPT)"] = calc_cumulative_oil_production_m3(overall_data_with_calibr_gaps,
                                                                                         gn.q_liq_m3day + " (ADAPT)",
                                                                                         time_to_resample)
    calibr_calc_metrics["Накопленная добыча жидкости, м3 (PREDICTION)"] = calc_cumulative_oil_production_m3(overall_data_with_calibr_gaps,
                                                                                         gn.q_liq_m3day + " (PREDICTION)",
                                                                                         time_to_resample)
    calibr_calc_metrics['Относительная ошибка по накопленной добыче жидкости, %'] = relative_error_perc(calibr_calc_metrics["Накопленная добыча жидкости, м3 (ADAPT)"],
                                                                                                         calibr_calc_metrics[
                                                                                                             "Накопленная добыча жидкости, м3 (PREDICTION)"])
    calibr_calc_metrics['Количество точек overall_data, шт'] = len(overall_data[gn.q_liq_m3day + " (ADAPT)"])
    return calibr_calc_metrics


def make_dimensionless_df(df):
    """
    Функция для обезразмеривания DataFrame
    :param df: исходные df
    :return: обезразмеренный df
    """
    result_df_dimensionless = df.copy()
    for i in result_df_dimensionless.columns:
        if i == 'Время':
            del result_df_dimensionless['Время']
        else:
            new_new = result_df_dimensionless[i]/result_df_dimensionless[i].max()
            result_df_dimensionless[i] = new_new
    return result_df_dimensionless
