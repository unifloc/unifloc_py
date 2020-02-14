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


def extract_time_from_df(df: pd.DataFrame, use_time_as_int_in_column=False):
    """
    Разделение DataFrame на отельную колонку с временем и DataFrame без времени
    :param use_time_as_int_in_column:
    :param df: исходный df с индексом времени
    :return: df, time_columns
    """
    time_index = df.index.copy()
    df = df.reset_index(drop=True)
    if use_time_as_int_in_column:
        df['Время'] = time_index.astype(int)
    return df, time_index


def get_test_train_drop_2_points(feature_data: pd.DataFrame, target_data: pd.Series):
    """
    Разделение DataFrame на train и test через 2
    :param feature_data: исходные данные - фичи
    :param target_data: исходные данные - ответы
    :return: out_x_train, out_x_test, out_y_train, out_y_test
    """
    feature_data_without_index, _ = extract_time_from_df(feature_data,
                                                         use_time_as_int_in_column=True)
    target_data_without_index, _ = extract_time_from_df(target_data,
                                                        use_time_as_int_in_column=True)

    out_x_train = feature_data_without_index[feature_data_without_index.index % 2 == 0]
    out_y_train = target_data_without_index[target_data_without_index.index % 2 == 0]
    out_x_test = feature_data_without_index[feature_data_without_index.index % 2 == 1]
    out_y_test = target_data_without_index[target_data_without_index.index % 2 == 1]

    out_x_train = out_x_train.set_index('Время')
    out_x_train.index = pd.to_datetime(out_x_train.index)

    out_y_train = out_y_train.set_index('Время')
    out_y_train.index = pd.to_datetime(out_y_train.index)

    out_x_test = out_x_test.set_index('Время')
    out_x_test.index = pd.to_datetime(out_x_test.index)

    out_y_test = out_y_test.set_index('Время')
    out_y_test.index = pd.to_datetime(out_y_test.index)

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


def predict_parameter_in_df(x_train, x_test, y_train, use_time_as_int_in_column=False):
    x_train_without_index, x_train_index = extract_time_from_df(x_train, use_time_as_int_in_column=use_time_as_int_in_column)
    x_test_without_index, x_test_index = extract_time_from_df(x_test, use_time_as_int_in_column=use_time_as_int_in_column)
    y_train_without_index, y_train_index = extract_time_from_df(y_train, use_time_as_int_in_column=False)
    y_test_without_index = predict_parameter_via_linear_model(x_train_without_index, x_test_without_index, y_train_without_index)

    if type(y_train) == pd.DataFrame:
        y_test_df = {}
        for i, j in enumerate(y_train.columns):
            y_test_df[j] = y_test_without_index[i]
        y_test_df = pd.DataFrame(y_test_df, index=x_test_index)
        y_test_df.index.name = 'Время'
    else:
        y_test_df = pd.Series(y_test_without_index)
        y_test_df.name = y_train.name
        y_test_df.index = x_test_index
        y_test_df.index.name = 'Время'
    return y_test_df


def predict_parameter_via_linear_model(x_train, x_test, y_train):
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
