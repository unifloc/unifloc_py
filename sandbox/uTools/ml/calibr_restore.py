from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GridSearchCV
from sklearn import linear_model
import numpy as np
import pandas as pd
import sys
sys.path.append('../'*4)
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool
global_names = preproc_tool.GlobalNames()


def check_ml_input_by_drop(df: pd.DataFrame):
    init_amount_of_rows, init_amount_of_columns = df.shape[0], df.shape[1]
    df = df.dropna()
    new_amount_of_rows, new_amount_of_columns = df.shape[0], df.shape[1]
    print(f"Удалено строк :{init_amount_of_rows - new_amount_of_rows},"
          f" удалено столбцов: {init_amount_of_columns - new_amount_of_columns}")
    return df


def extract_time_from_df(df: pd.DataFrame):
    """
    Разделение DataFrame на отельную колонку с временем и DataFrame без времени
    :param df: исходный df
    :return: df, time_columns
    """
    time_columns = df['Время']
    df = df.drop(columns=['Время'])
    return df, time_columns


def get_test_train_drop_2_points(data: pd.DataFrame, target: pd.Series):
    """
    Разделение DataFrame на train и test через 2
    :param data: исходные данные - фичи
    :param target: исходные данные - ответы
    :return: out_x_train, out_x_test, out_y_train, out_y_test
    """
    out_x_train = data[data.index % 2 == 0]
    out_y_train = target[target.index % 2 == 0]
    out_x_test = data[data.index % 2 == 1]
    out_y_test = target[target.index % 2 == 1]
    return out_x_train, out_x_test, out_y_train, out_y_test


def get_test_train_80_20(data: pd.DataFrame, target: pd.Series):
    """
    Разделение DataFrame на train и test как 80 и 20
    :param data: исходные данные - фичи
    :param target: исходные данные - ответы
    :return: out_x_train, out_x_test, out_y_train, out_y_test
    """
    board_80_20 = int(len(data.index) * 0.8)
    out_x_train = data[data.index <= board_80_20]
    out_y_train = target[target.index <= board_80_20]
    out_x_test = data[data.index > board_80_20]
    out_y_test = target[target.index > board_80_20]
    return out_x_train, out_x_test, out_y_train, out_y_test



def get_joined_2_points_target(y_test, y_train):
    """
    Склейка данных теста и обучения
    :param y_test:
    :param y_train:
    :return: исходный DataFrame с предсказанными значениями через 2
    """
    out = [None] * (len(y_test) + len(y_train))
    y_test = list(y_test)
    y_train = list(y_train)
    for i in range(len(out)):
        if i % 2 == 0:
            out[i] = y_train[int(i/2)]
        else:
            out[i] = y_test[int((i-1)/2)]
    return out


def get_joined_80_20(y_test, y_train):
    """
    Склейка данных теста и обучения
    :param y_test:
    :param y_train:
    :return: исходный DataFrame с предсказанными значениями (20% последних)
    """
    y_test = list(y_test)
    y_train = list(y_train)
    out = y_train + y_test
    return out


def restore_calibr_via_ridge(x_train, x_test, y_train):
    # отмаштабируем данные
    sc = StandardScaler()
    sc.fit(x_train)
    x_train_sc = sc.transform(x_train)
    x_test_sc = sc.transform(x_test)
    # у модели. которой сейчас будем пользоваться есть параметр, задающийся из вне
    # пробежимся на данных для обучения и подберём его
    parameters = {'alpha': np.linspace(1e-7, 30, 1000)}

    r_est_power = linear_model.Ridge()
    clf = GridSearchCV(r_est_power, parameters, cv=5)
    _ = clf.fit(x_train_sc, y_train)
    b_reg = clf.best_estimator_
    y_test = b_reg.predict(x_test_sc)

    return y_test
