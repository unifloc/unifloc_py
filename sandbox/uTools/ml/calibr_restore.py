from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GridSearchCV
from sklearn import linear_model
import numpy as np
import pandas as pd

def restore_calibr_via_ridge(adaptation_all_data_lol, calibr_data): #TODO подозрительно много warnings - надо исправить
    # но всё нам не нужно, возьмём только хорошее
    adaptation_data_lol = adaptation_all_data_lol[['ГФ (СУ)', 'Процент обводненности (СУ)',
                                           'Давление на приеме насоса (пласт. жидкость) (СУ)', 'Рлин ТМ (Ш)', 'Рбуф (Ш)',
                                           'Температура на приеме насоса (пласт. жидкость) (СУ)', 'F вращ ТМ (Ш)', 'Dшт (Ш)',
                                           'Активная мощность (СУ)', 'Напряжение на выходе ТМПН (СУ)', 'Коэффициент мощности (СУ)',
                                           'К. калибровки по напору - множитель (Модель)',
                                           'К. калибровки по мощности - множитель (Модель)'
                                          ]
                                         ]
    # уберем пропуски в существенных данных
    adaptation_data_lol.dropna(inplace=True)
    # и обновляем индексы
    adaptation_data_lol.reset_index(inplace=True)
    del adaptation_data_lol['Время']
    # берём ответы в отдельную переменную
    y_f_lol = adaptation_data_lol['К. калибровки по напору - множитель (Модель)']
    y_p_lol = adaptation_data_lol['К. калибровки по мощности - множитель (Модель)']
    # а в общих данных удаляем их
    adaptation_data_lol.drop(columns=['К. калибровки по напору - множитель (Модель)',
                                  'К. калибровки по мощности - множитель (Модель)'],
                         inplace=True
                        )
    # обозначим функцию для разделения данных на тест и обучение "через одну"

    def get_test_train_drop_2_points(data: pd.DataFrame, target: pd.Series):

        out_x_train = data[data.index % 2 == 0]
        out_y_train = target[target.index % 2 == 0]
        out_x_test = data[data.index % 2 == 1]
        out_y_test = target[target.index % 2 == 1]
        return out_x_train, out_x_test, out_y_train, out_y_test

    # Разделим
    x_train_lol, x_test_lol, y_train_p_lol, y_test_p_lol = get_test_train_drop_2_points(adaptation_data_lol, y_p_lol)
    _, _, y_train_f_lol, y_test_f_lol = get_test_train_drop_2_points(adaptation_data_lol, y_f_lol)
    # отмаштабируем данные
    sc_lol = StandardScaler()
    sc_lol.fit(x_train_lol)
    x_train_sc_lol = sc_lol.transform(x_train_lol)
    x_test_sc_lol = sc_lol.transform(x_test_lol)
    # у модели. которой сейчас будем пользоваться есть параметр, задающийся из вне
    # пробежимся на данных для обучения и подберём его
    parameters_lol = {'alpha': np.linspace(1e-7, 30, 1000)}

    r_est_p_lol = linear_model.Ridge()
    clf_p_lol = GridSearchCV(r_est_p_lol, parameters_lol, cv=5)
    _ = clf_p_lol.fit(x_train_sc_lol, y_train_p_lol)
    b_reg_p_lol = clf_p_lol.best_estimator_
    p_pred_lol = b_reg_p_lol.predict(x_test_sc_lol)

    r_est_f_lol = linear_model.Ridge()
    clf_f_lol = GridSearchCV(r_est_f_lol, parameters_lol, cv=5)
    _ =clf_f_lol.fit(x_train_sc_lol, y_train_f_lol)
    b_reg_f_lol = clf_f_lol.best_estimator_
    f_pred_lol = b_reg_f_lol.predict(x_test_sc_lol)
    # пробежались
    # тепрь нужно сшить предсказания и тест

    def get_joined_2_points_target(y_test, y_train):
        out = [None] * (len(y_test) + len(y_train))
        y_test = list(y_test)
        y_train = list(y_train)
        for i in range(len(out)):
            if i % 2 == 0:
                out[i] = y_train[int(i/2)]
            else:
                out[i] = y_test[int((i-1)/2)]
        return out
    # сшиваем
    p_out_lol = get_joined_2_points_target(y_test=p_pred_lol, y_train=y_train_p_lol)
    f_out_lol = get_joined_2_points_target(y_test=f_pred_lol, y_train=y_train_f_lol)
    # вдруг потерялись точки
    if len(calibr_data['К. калибровки по напору - множитель (Модель) (Подготовленные)']) != len(p_out_lol):
        print("WTFFFFF чтото с размерами, останавливай нафиг всё")
    # ну если не потерялись, то чуть ниже вставим машинно обученные предсказания в модель
    return (p_out_lol, f_out_lol)