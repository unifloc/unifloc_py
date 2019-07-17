"""
Модуль работы над инклинометрией скважины
Кобзарь О.С. 2019 г.
"""

import pandas as pd
import numpy as np
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
        """
        Функция удаляет последний ряд в DataFrame - он обычно пустой

        :return: DataFrame без последней строки
        """
        self.deviation_survey_dataframe = self.deviation_survey_dataframe.iloc[:-1]

    def __change_column_type__(self, column):
        """
        Функция меняет формат данных столбца с str на float64

        :param column: столбец из PandasDataFrame в формате str
        :return: столбец из PandasDataFrame в формате float64
        """
        column = column.str.replace(',', '.')
        column = column.astype('float64')
        return column

    def load_deviation_survey(self, path_to_file_str):
        """
        Загрузка данных из Excel и удаление последней строки

        :param path_to_file_str: путь к файлу, str
        :return: None
        """
        self.deviation_survey_dataframe = pd.read_excel(path_to_file_str)
        self.__scip_last_row__()

    def change_str_to_float(self):
        """
        Функция меняет в необходимых столбцах str на float64

        :return: None
        """
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
        """
        Интерполяция вертикальной отметки и угла по измеренной глубине

        :return: None
        """
        self.h_vert_interpolate_func = interpolate.interp1d(
            self.deviation_survey_dataframe['Глубина конца интервала, м'],
            self.deviation_survey_dataframe['Вертикальная отметка'], kind='cubic')
        self.angle_from_vert_interpolate_func = interpolate.interp1d(
            self.deviation_survey_dataframe['Глубина конца интервала, м'],
            self.deviation_survey_dataframe['Угол, гpад'], kind='cubic')

    def get_h_vert_m(self, h_mes_m):
        """
        Функция по интерполированным данным возвращает абсолютную вертикальную отметку в точке по измеренной глубине

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: абсолютная (вертикальная) глубина, м
        """
        self.h_mes_m = self.h_vert_interpolate_func(h_mes_m)
        return self.h_mes_m

    def get_angle_from_vert_grad(self, h_mes_m):
        """
        Функция по интерполированным данным возращает угол наклона от вертикали

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: угол отклонения ствола от вертикали, град
        """
        self.angle_from_vert_grad = self.angle_from_vert_interpolate_func(h_mes_m)
        return self.angle_from_vert_grad


class simple_well_deviation_survey():

    def __init__(self):
        self.h_conductor_mes_m = 500
        self.h_conductor_vert_m = 500
        self.h_conductor_end_mes_m = 600
        self.h_conductor_end_vert_m = 590
        self.h_pump_mes_m = 1200
        self.h_pump_vert_m = 1000
        self.h_bottomhole_mes_m = 2500
        self.h_bottomhole_vert_m = 1500

        self.lenth_of_one_part = 10

        self.h_mes_init_data_for_interpolation_m = None
        self.h_vert_init_data_for_interpolation_m = None
        
        self.interpolation_func_linear_h_vert_by_h_mes = None
        self.interpolation_func_quadratic_h_vert_by_h_mes = None

        self.amounts_of_parts = None

        self.h_mes_m = None
        self.h_vert_m = None
        self.angle_from_vert_grad = None
        self.x_displacement_m = None
        self.y_displacement_m = None
        self.rate_of_curvature_grad10m = None
        self.borehole_extension = None

    def calc_all(self):
        self.h_mes_init_data_for_interpolation_m = [0, self.h_conductor_mes_m, self.h_conductor_end_mes_m,
                                                    self.h_pump_mes_m, self.h_bottomhole_mes_m]
        self.h_vert_init_data_for_interpolation_m = [0, self.h_conductor_vert_m, self.h_conductor_end_vert_m,
                                                     self.h_pump_vert_m, self.h_bottomhole_vert_m]

        self.interpolation_func_linear_h_vert_by_h_mes = interpolate.interp1d(self.h_mes_init_data_for_interpolation_m,
                                                                              self.h_vert_init_data_for_interpolation_m,
                                                                              kind = 'slinear')
        self.interpolation_func_quadratic_h_vert_by_h_mes = interpolate.interp1d(self.h_mes_init_data_for_interpolation_m,
                                                                                 self.h_vert_init_data_for_interpolation_m,
                                                                                 kind='cubic')

        self.amounts_of_parts = int(self.h_bottomhole_mes_m / self.lenth_of_one_part)

        h_mes_m = [0]
        h_vert_m = [0]
        angle_from_vert_grad = [90]
        x_displacement_m = [0]
        rate_of_curvature_grad10m = [0]
        borehole_extension = [0]

        for i in range(self.amounts_of_parts):
            current_h_mes_m = h_mes_m[-1] + self.lenth_of_one_part

            if current_h_mes_m <= self.h_conductor_mes_m:
                current_h_vert_m = float(self.interpolation_func_linear_h_vert_by_h_mes(current_h_mes_m))
            else:
                current_h_vert_m = float(self.interpolation_func_quadratic_h_vert_by_h_mes(current_h_mes_m))

            current_borehole_extension = current_h_mes_m - current_h_vert_m

            delta_h_vert_m = current_h_vert_m - h_vert_m[-1]

            delta_x_displacement_m = (self.lenth_of_one_part ** 2 - delta_h_vert_m ** 2) ** (1 / 2)

            if type(delta_x_displacement_m) == complex:
                delta_x_displacement_m = delta_x_displacement_m.real

            current_x_displacement_m = x_displacement_m[-1] + delta_x_displacement_m

            cos_phi_vert_angle = delta_x_displacement_m / self.lenth_of_one_part

            current_angle_from_vert_grad = np.degrees(np.arccos(float(cos_phi_vert_angle)))

            current_rate_of_curvature_grad10m = (angle_from_vert_grad[
                                                     -1] - current_angle_from_vert_grad) / self.lenth_of_one_part

            h_mes_m.append(current_h_mes_m)
            h_vert_m.append(current_h_vert_m)
            borehole_extension.append(current_borehole_extension)
            x_displacement_m.append(current_x_displacement_m)
            angle_from_vert_grad.append(current_angle_from_vert_grad)
            rate_of_curvature_grad10m.append(current_rate_of_curvature_grad10m)

        self.h_mes_m = np.asarray(h_mes_m)
        self.h_vert_m = np.asarray(h_vert_m)
        self.angle_from_vert_grad = np.asarray(angle_from_vert_grad)
        self.x_displacement_m = np.asarray(x_displacement_m)
        self.y_displacement_m = np.asarray(x_displacement_m) * 0
        self.rate_of_curvature_grad10m = np.asarray(rate_of_curvature_grad10m)
        self.borehole_extension = np.asarray(borehole_extension)

