"""
Модуль работы над инклинометрией скважины

Кобзарь О.С. 18.07.2019 г.
"""

import pandas as pd
import numpy as np
import scipy.interpolate as interpolate


# TODO добавить логику для проверки ошибок - "защиту от дурака"
# TODO проверить методы интерполяции - где уместно линейную, где кубическую?
# TODO проверить простую модель, добавить возможность добавление точек для создания профиля любой сложности
# TODO переделать циклы for на numpy

class well_deviation_survey:
    """
    Класс для задания профиля ствола скважины по инклинометрии, которая загружается из файла excel
    """
    def __init__(self):
        self.deviation_survey_dataframe = None
        self.h_vert_interpolate_func = None
        self.vert_angle_interpolate_func = None
        self.curvature_rate_interpolate_func = None

        self.column_h_mes_m = None
        self.column_curvature_rate_grad10m = None

        self.h_mes_m = None
        self.vert_angle_grad = None
        self.curvature_rate_grad10m = None

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
        Функция меняет в столбцах str на float64

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
        self.vert_angle_interpolate_func = interpolate.interp1d(
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

    def get_vert_angle_grad(self, h_mes_m):
        """
        Функция по интерполированным данным возращает угол наклона от вертикали

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: угол отклонения ствола от вертикали, град
        """
        self.vert_angle_grad = self.vert_angle_interpolate_func(h_mes_m)
        return self.vert_angle_grad

    def get_curvature_rate_grad10m(self, h_mes_m):
        self.curvature_rate_grad10m = self.curvature_rate_interpolate_func(h_mes_m)
        return self.curvature_rate_grad10m

    def calc_curvature(self):
        """
        Функция рассчитывает скорость набора кривизны (изменение угла на 10 м)
        Результатом являются:

            дополнительный столбец в DataFrame с рассчитаными значениями

            отдельный массив с рассчитанными значениями

            интерполяционная функция для нахождения скорости набора кривизны от глубины вдоль ствола скважины

        :return: None
        """
        h_mes_m = [0]
        curvature_rate = [0]
        column_h_mes = self.deviation_survey_dataframe['Глубина конца интервала, м']
        borehole_lenth_m = column_h_mes.max() - column_h_mes.min()
        lenth_of_one_part = 10
        amounts_of_parts = int(
            borehole_lenth_m / lenth_of_one_part - borehole_lenth_m % lenth_of_one_part / lenth_of_one_part)
        for i in range(amounts_of_parts):
            current_h_mes_m = h_mes_m[-1] + lenth_of_one_part
            current_angle_grad = self.get_vert_angle_grad(current_h_mes_m)
            last_angle_grad = self.get_vert_angle_grad(h_mes_m[-1])
            current_curvature_rate = abs(current_angle_grad - last_angle_grad) / lenth_of_one_part
            h_mes_m.append(current_h_mes_m)
            curvature_rate.append(current_curvature_rate)

        if h_mes_m[-1] < column_h_mes.max():
            current_h_mes_m = column_h_mes.max() - lenth_of_one_part
            current_angle_grad = self.get_vert_angle_grad(current_h_mes_m)
            last_angle_grad = self.get_vert_angle_grad(column_h_mes.max())
            current_curvature_rate = abs(current_angle_grad - last_angle_grad) / lenth_of_one_part
            curvature_rate.append(current_curvature_rate)
            h_mes_m.append(column_h_mes.max())

        h_mes_m = np.asarray(h_mes_m)
        curvature_rate = np.asarray(curvature_rate)
        self.curvature_rate_interpolate_func = interpolate.interp1d(h_mes_m, curvature_rate, kind='cubic')
        self.deviation_survey_dataframe['Интенсивность кривизны, град/10 м'] = self.curvature_rate_interpolate_func(
            column_h_mes
        )
        self.column_curvature_rate_grad10m = self.deviation_survey_dataframe['Интенсивность кривизны, град/10 м']

    def calc_all(self):
        """
        Функция выполняет необходимые все необходимые расчеты, после которой класс может быть использован по назначению

        :return: None
        """
        self.change_str_to_float()
        self.interpolate_all()
        self.calc_curvature()
        self.column_h_mes_m = self.deviation_survey_dataframe['Глубина конца интервала, м']


class simple_well_deviation_survey():
    """
    Класс для задания простого профиля скважины по точкам
    """
    def __init__(self):
        self.h_conductor_mes_m = 500
        self.h_conductor_vert_m = 500
        self.h_pump_mes_m = 1200
        self.h_pump_vert_m = 1000
        self.h_bottomhole_mes_m = 2500
        self.h_bottomhole_vert_m = 1500

        self.lenth_of_one_part = 10

        self.h_mes_init_data_for_interpolation_m = None
        self.h_vert_init_data_for_interpolation_m = None

        self.interpolation_func_slinear_h_vert_by_h_mes = None
        self.interpolation_func_cubic_h_vert_by_h_mes = None

        self.amounts_of_parts = None

        self.h_mes_m = None
        self.h_vert_m = None
        self.vert_angle_grad = None
        self.x_displacement_m = None
        self.y_displacement_m = None
        self.curvature_rate_grad10m = None
        self.borehole_extension_m = None

        self.interpolation_x_displacement_by_h_mes = None
        self.interpolation_vert_angle_by_h_mes = None
        self.interpolation_h_vert_by_h_mes = None
        self.interpolation_borehole_extension_by_h_mes = None
        self.interpolation_curvature_rate_by_h_mes = None

    def calc_all(self):
        """
        Функция с помощью интерполяции по нескольким точкам строит профиль скважины.

        Исходными данными являются: точки абсолютной глубины и глубины вдоль ствола скважина кондуктора, приема
        оборудования, забоя.

        :return: None
        """
        self.h_mes_init_data_for_interpolation_m = np.asarray([0, self.h_conductor_mes_m,
                                                               self.h_pump_mes_m, self.h_bottomhole_mes_m])
        self.h_vert_init_data_for_interpolation_m = np.asarray([0, self.h_conductor_vert_m,
                                                               self.h_pump_vert_m, self.h_bottomhole_vert_m])

        self.interpolation_func_slinear_h_vert_by_h_mes = interpolate.interp1d(self.h_mes_init_data_for_interpolation_m,
                                                                               self.h_vert_init_data_for_interpolation_m,
                                                                               kind='linear')
        self.interpolation_func_cubic_h_vert_by_h_mes = interpolate.interp1d(self.h_mes_init_data_for_interpolation_m,
                                                                             self.h_vert_init_data_for_interpolation_m,
                                                                             kind='cubic')

        self.amounts_of_parts = int(self.h_bottomhole_mes_m / self.lenth_of_one_part)

        h_mes_m = [0]
        h_vert_m = [0]
        vert_angle_grad = [90]
        x_displacement_m = [0]
        curvature_rate_grad10m = [0]
        borehole_extension = [0]

        for i in range(self.amounts_of_parts):
            current_h_mes_m = h_mes_m[-1] + self.lenth_of_one_part

            current_h_vert_m = float(self.interpolation_func_slinear_h_vert_by_h_mes(current_h_mes_m))
            if current_h_vert_m < current_h_mes_m:
                current_h_vert_m = float(self.interpolation_func_cubic_h_vert_by_h_mes(current_h_mes_m))

            current_borehole_extension = current_h_mes_m - current_h_vert_m

            delta_h_vert_m = current_h_vert_m - h_vert_m[-1]

            delta_x_displacement_m = (self.lenth_of_one_part ** 2 - delta_h_vert_m ** 2) ** (1 / 2)

            if type(delta_x_displacement_m) == complex:
                delta_x_displacement_m = delta_x_displacement_m.real

            current_x_displacement_m = x_displacement_m[-1] + delta_x_displacement_m

            cos_phi_vert_angle = delta_x_displacement_m / self.lenth_of_one_part

            current_vert_angle_grad = np.degrees(np.arccos(float(cos_phi_vert_angle)))

            current_curvature_rate_grad10m = (vert_angle_grad[
                                                     -1] - current_vert_angle_grad) / self.lenth_of_one_part

            h_mes_m.append(current_h_mes_m)
            h_vert_m.append(current_h_vert_m)
            borehole_extension.append(current_borehole_extension)
            x_displacement_m.append(current_x_displacement_m)
            vert_angle_grad.append(current_vert_angle_grad)
            curvature_rate_grad10m.append(current_curvature_rate_grad10m)

        self.h_mes_m = np.asarray(h_mes_m)
        self.h_vert_m = np.asarray(h_vert_m)
        self.vert_angle_grad = np.asarray(vert_angle_grad)
        self.x_displacement_m = np.asarray(x_displacement_m)
        self.y_displacement_m = np.asarray(x_displacement_m) * 0
        self.curvature_rate_grad10m = np.asarray(curvature_rate_grad10m)
        self.borehole_extension_m = np.asarray(borehole_extension)

        self.interpolation_x_displacement_by_h_mes = interpolate.interp1d(self.h_mes_m,
                                                                          self.x_displacement_m,
                                                                          kind='cubic')

        self.interpolation_h_vert_by_h_mes = interpolate.interp1d(self.h_mes_m,
                                                                  self.h_vert_m,
                                                                  kind='cubic')

        self.interpolation_vert_angle_by_h_mes = interpolate.interp1d(self.h_mes_m,
                                                                      self.vert_angle_grad,
                                                                      kind='cubic')

        self.interpolation_borehole_extension_by_h_mes = interpolate.interp1d(self.h_mes_m,
                                                                              self.borehole_extension_m,
                                                                              kind='cubic')

        self.interpolation_curvature_rate_by_h_mes = interpolate.interp1d(self.h_mes_m,
                                                                          self.curvature_rate_grad10m,
                                                                          kind='cubic')

    def get_x_displacement_m(self, h_mes_m):
        """
        Функция по результатам выполненной ранее интерполяции возвращает горизонтально смещение от устья

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: горизонтальное смещение от устья, м
        """
        return self.interpolation_x_displacement_by_h_mes(h_mes_m)

    def get_h_vert_m(self, h_mes_m):
        """
        Функция по результатам выполненной ранее интерполяции возвращает абсолютную глубину

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: абсолютная вертикальная глубина, м
        """
        return self.interpolation_h_vert_by_h_mes(h_mes_m)

    def get_vert_angle_grad(self, h_mes_m):
        """
        Функция по результатам выполненной ранее интерполяции возвращает угол наклона от вертикали

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: угол наклона от вертикали, м
        """
        return self.interpolation_vert_angle_by_h_mes(h_mes_m)

    def get_borehole_extension_m(self, h_mes_m):
        """
        Функция по результатам выполненной ранее интерполяции возвращает величину удлинения ствола скважины,
        т.е. разницу между измеренной глубиной и абсолютной

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: удлинение ствола скважины, м
        """
        return self.interpolation_borehole_extension_by_h_mes(h_mes_m)

    def get_curvature_rate_grad10m(self, h_mes_m):
        """
        Функция по результатам выполненной ранее интерполяции возвращает величину скорости набора кривизны

        :param h_mes_m: измеренная глубина вдоль ствола скважины, м
        :return: скорость набора кривизны, град / 10 м
        """
        return self.interpolation_curvature_rate_by_h_mes(h_mes_m)



