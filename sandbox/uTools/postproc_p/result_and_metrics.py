"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль динамического анализа исходных данных и рассчитанных значений
"""
from sklearn import metrics
import pandas as pd
def relative_error_perc(y1, y2):
    return (y1 - y2) / y1 * 100


def calc_mertics(y_fact, y_pred, mark_str, return_df = False):
    """
    Расчет метрик успешности работы
    :param y_fact:
    :param y_pred:
    :param mark_str:
    :return:
    """
    if return_df == True:
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
    else:

        answer = mark_str + '\n'
        r2_score = metrics.r2_score(y_fact, y_pred)
        mean_absolute_error = metrics.mean_absolute_error(y_fact, y_pred)
        mean_squared_error = metrics.mean_squared_error(y_fact, y_pred)
        max_error = metrics.max_error(y_fact, y_pred)
        median_absolute_error = metrics.median_absolute_error(y_fact, y_pred)
        explained_variance_score = metrics.explained_variance_score(y_fact, y_pred)
        mean_squared_log_error = metrics.mean_squared_log_error(y_fact, y_pred)
        answer += 'r2_score: ' + str(r2_score) + '\n' + \
                  'mean_absolute_error: ' + str(mean_absolute_error) + '\n' + \
                  'mean_squared_error: ' + str(mean_squared_error) + '\n' + \
                  'max_error: ' + str(max_error) + '\n' + \
                  'median_absolute_error: ' + str(median_absolute_error) + '\n' + \
                  'explained_variance_score: ' + str(explained_variance_score) + '\n' + \
                  'mean_squared_log_error: ' + str(mean_squared_log_error) + '\n'
        return answer


def final_edit_overall_data(overall_data):
    overall_data['Активная мощность (СУ) (ADAPT)'] = overall_data['Активная мощность (СУ) (ADAPT)'] * 1000
    overall_data['Активная мощность (СУ) (ADAPT) (INTERP)'] = overall_data['Активная мощность (СУ) (ADAPT) (INTERP)'] * 1000
    overall_data['Загрузка двигателя (СУ) (ADAPT)'] = overall_data['Загрузка двигателя (СУ) (ADAPT)'] / 100
    overall_data['Относительная ошибка расчетов (Q ж), %'] = relative_error_perc(overall_data['Объемный дебит жидкости (СУ) (ADAPT)'],
                                                                          overall_data['Q ж, м3/сут (Модель) (RESTORE)']).abs()
    overall_data['Относительная ошибка расчетов (N акт), %'] = relative_error_perc(overall_data['Активная мощность (СУ) (ADAPT)'],
                                                                          overall_data['Мощность, передаваемая СУ (Модель) (RESTORE)']).abs()
    overall_data['Относительная ошибка расчетов (Q ж) (INTERP), %'] = relative_error_perc(overall_data['Объемный дебит жидкости (СУ) (ADAPT)'],
                                                                          overall_data['Объемный дебит жидкости (СУ) (ADAPT) (INTERP)']).abs()
    overall_data['Относительная ошибка расчетов (N акт) (INTERP), %'] = relative_error_perc(overall_data['Активная мощность (СУ) (ADAPT) (INTERP)'],
                                                                          overall_data['Мощность, передаваемая СУ (Модель) (RESTORE)']).abs()
    return overall_data


def calc_calibr_interp_metrics(overall_data):
    overall_data_with_calibr_gaps = overall_data[overall_data['К. калибровки по напору - множитель (Модель) (ADAPT)'] !=
                                                overall_data['К. калибровки по напору - множитель (Модель) (RESTORE)']]
    overall_data_with_calibr_gaps = overall_data_with_calibr_gaps.dropna(subset = ['Q ж, м3/сут (Модель) (RESTORE)'])
    calibr_calc_metrics = calc_mertics(overall_data_with_calibr_gaps['Q ж, м3/сут (Модель) (ADAPT)'],
                overall_data_with_calibr_gaps['Q ж, м3/сут (Модель) (RESTORE)'], 'Метрики восстановления Qж с помощью калибровок')
    calibr_calc_metrics += 'Средняя относительная ошибка Q ж, %: ' + \
                           str(overall_data_with_calibr_gaps['Относительная ошибка расчетов (Q ж), %'].abs().mean()) + '\n'
    calibr_calc_metrics += 'Средняя относительная ошибка N акт, %: ' + \
                           str(overall_data_with_calibr_gaps['Относительная ошибка расчетов (N акт), %'].abs().mean()) + '\n'

    interp_calc_metrics = calc_mertics(overall_data_with_calibr_gaps['Объемный дебит жидкости (СУ) (ADAPT)'],
                overall_data_with_calibr_gaps['Объемный дебит жидкости (СУ) (ADAPT) (INTERP)'], 'Метрики восстановления Qж с помощью интерполяции')
    interp_calc_metrics +='Средняя относительная ошибка Q ж (INTERP), %: ' +\
                          str(overall_data_with_calibr_gaps['Относительная ошибка расчетов (Q ж) (INTERP), %'].abs().mean()) + '\n'
    interp_calc_metrics +='Средняя относительная ошибка N акт (INTERP), %: ' + \
                          str(overall_data_with_calibr_gaps['Относительная ошибка расчетов (N акт) (INTERP), %'].abs().mean()) + '\n'
    return (calibr_calc_metrics, interp_calc_metrics)