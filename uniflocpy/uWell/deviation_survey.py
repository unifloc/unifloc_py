"""
Модуль работы над инклинометрией скважины
Кобзарь О.С. 2019 г.
"""

import pandas as pd
import scipy.interpolate as interpolate


# TODO добавить логику для проверки ошибок - "защиту от дурака"
# TODO добавить построение простой скважины


class well_deviation_survey:
    def __init__(self):
        self.deviation_survey_dataframe = None
        self.h_vert_interpolate_func = None
        self.angle_from_vert_interpolate_func = None

        self.h_mes_m = None
        self.angle_from_vert_grad = None

    def __scip_last_row__(self):
        self.deviation_survey_dataframe = self.deviation_survey_dataframe.iloc[:-1]

    def __change_column_type__(self, column):
        column = column.str.replace(',', '.')
        column = column.astype('float64')
        return column

    def load_deviation_survey(self, path_to_file_str):
        self.deviation_survey_dataframe = pd.read_excel(path_to_file_str)
        self.__scip_last_row__()

    def change_str_to_float(self):
        self.deviation_survey_dataframe['Координата Х (инклинометрия)'] = self.__change_column_type__(
            self.deviation_survey_dataframe['Координата Х (инклинометрия)'])
        self.deviation_survey_dataframe['Координата Y (инклинометрия)'] = self.__change_column_type__(
            self.deviation_survey_dataframe['Координата Y (инклинометрия)'])
        self.deviation_survey_dataframe['Вертикальная отметка'] = self.__change_column_type__(
            self.deviation_survey_dataframe['Вертикальная отметка'])
        self.deviation_survey_dataframe['Глубина конца интервала, м'] = self.__change_column_type__(
            self.deviation_survey_dataframe['Глубина конца интервала, м'])
        self.deviation_survey_dataframe['Угол, гpад'] = self.__change_column_type__(
            self.deviation_survey_dataframe['Угол, гpад'])

    def interpolate_all(self):
        self.h_vert_interpolate_func = interpolate.interp1d(
            self.deviation_survey_dataframe['Глубина конца интервала, м'],
            self.deviation_survey_dataframe['Вертикальная отметка'], kind='cubic')
        self.angle_from_vert_interpolate_func = interpolate.interp1d(
            self.deviation_survey_dataframe['Глубина конца интервала, м'],
            self.deviation_survey_dataframe['Угол, гpад'], kind='cubic')

    def get_h_vert_m(self, h_mes_m):
        self.h_mes_m = self.h_vert_interpolate_func(h_mes_m)
        return self.h_mes_m

    def get_angle_from_vert_grad(self, h_mes_m):
        self.angle_from_vert_grad = self.angle_from_vert_interpolate_func(h_mes_m)
        return self.angle_from_vert_grad

