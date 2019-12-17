import pandas as pd


def get_test_train_drop_2_points(data: pd.DataFrame, target: pd.Series):
    out_x_train = data[data.index % 2 == 0]
    out_y_train = target[target.index % 2 == 0]
    out_x_test = data[data.index % 2 == 1]
    out_y_test = target[target.index % 2 == 1]
    return out_x_train, out_x_test, out_y_train, out_y_test


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
