"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль динамического анализа исходных данных и рассчитанных значений
"""
from sklearn import metrics
import pandas as pd
def relative_error_perc(y1, y2):
    """
    Расчет относительной ошибки
    :param y1: фактическое значение
    :param y2: прогнозное значения
    :return: относительная ошибка, %
    """
    return (y1 - y2) / y1 * 100


def calc_mertics(y_fact, y_pred, mark_str, return_df = False): #TODO добавить MAE чтобы было относительное
    """
    Расчет метрик модели
    :param y_fact: фактические значения
    :param y_pred: прогнозные значения
    :param mark_str: string для пометки результатов в виде текста
    :param return_df: bool - флаг для возврата метрик в виде DataFrame
    :return: набор метрик в виде string или DataFrame
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
    """
    Финальная обработка сведенных данных адаптации и восстановления
    :param overall_data: DataFrame с адаптацией, восстановлением при помощи калибровок + линейной интерполяцией
    :return: overall_data с поправленными размерностями данных и относительными ошибками по дебиту
            и мощности (для калибровок и интерполяции)
    """
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


def calc_calibr_interp_metrics(overall_data, return_df=False):  #TODO оставить только df с методом to_string или excel, удалить str мусор
    """
    Итоговый расчет метрик
    :param overall_data: сведенные данные адаптации и восстановления на весь период.
                         Преобразуется к overall_data_with_calibr_gaps - df только с тестовыми (прогнозными) значениями
    :param return_df: bool - флаг для возврата DataFrame
    :return:
    """
    overall_data_with_calibr_gaps = overall_data[overall_data['К. калибровки по напору - множитель (Модель) (ADAPT)'] !=
                                                overall_data['К. калибровки по напору - множитель (Модель) (RESTORE)']]
    overall_data_with_calibr_gaps = overall_data_with_calibr_gaps.dropna(subset = ['Q ж, м3/сут (Модель) (RESTORE)'])
    if return_df == False:
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
    else:
        calibr_calc_metrics = calc_mertics(overall_data_with_calibr_gaps['Q ж, м3/сут (Модель) (ADAPT)'],
                                                 overall_data_with_calibr_gaps['Q ж, м3/сут (Модель) (RESTORE)'],
                                                 'Метрики восстановления Qж с помощью калибровок', return_df=True)
        interp_calc_metrics = calc_mertics(overall_data_with_calibr_gaps['Объемный дебит жидкости (СУ) (ADAPT)'],
                                                 overall_data_with_calibr_gaps[
                                                     'Объемный дебит жидкости (СУ) (ADAPT) (INTERP)'],
                                                 'Метрики восстановления Qж с помощью интерполяции', return_df=True)
        overall_metrics = calibr_calc_metrics.append(interp_calc_metrics)
        overall_metrics['Средняя относительная ошибка Q ж'] = [
            overall_data_with_calibr_gaps['Относительная ошибка расчетов (Q ж), %'].abs().mean(),
            overall_data_with_calibr_gaps['Относительная ошибка расчетов (Q ж) (INTERP), %'].abs().mean()]
        overall_metrics['Средняя относительная ошибка N акт'] = [
            overall_data_with_calibr_gaps['Относительная ошибка расчетов (N акт), %'].abs().mean(),
            overall_data_with_calibr_gaps['Относительная ошибка расчетов (N акт) (INTERP), %'].abs().mean()]
        overall_metrics['Q ж, м3/сут (Модель) (ADAPT) mean'] = [
            overall_data_with_calibr_gaps['Q ж, м3/сут (Модель) (ADAPT)'].abs().mean(),
            overall_data_with_calibr_gaps['Q ж, м3/сут (Модель) (ADAPT)'].abs().mean()]
        return overall_metrics


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
