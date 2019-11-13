"""
Кобзарь О.С. Хабибуллин Р.А.

Модуль динамического анализа исходных данных и рассчитанных значений
"""
from sklearn import metrics

def relative_error_perc(y1, y2):
    return (y1 - y2) / y1 * 100


def calc_mertics(y_fact, y_pred, mark_str):
    """
    Расчет метрик успешности работы
    :param y_fact:
    :param y_pred:
    :param mark_str:
    :return:
    """
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
