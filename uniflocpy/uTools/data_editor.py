"""
Модуль для первичной загрузки и редактирования данных со станции управления, сохранения данных в новый файл
.csv, в котором параметры будут разнесены по столбцам и отсортированы по времени

Кобзарь О.С. Хабибуллин Р.А. 29.07.2019
"""
import numpy as np
import pandas as pd


class Editor_initial_cs_data():
    def __init__(self):
        self.init_pandas_df = None
        self.parametrs_list = None
        self.str_to_delete = None

    def load_csv(self, csv_file_name_str):
        """
        Стандартная загрузка через Pandas с учетом особенностей файла со СУ - нет заголовков и разделитель ;
        если при загрузке выявляется ошибка, необходимо загрузить csv стандартным методом pandas, а после
        присвоить атрибуту self.init_pandas_df текущего класса загруженный DataFrame

        :param csv_file_name_str: имя файлика в директории, data.csv
        :return: None
        """
        self.init_pandas_df = pd.read_csv(csv_file_name_str, sep=';', header=None)

    def get_parametrs_list(self, cs_data=-1):
        """
        Определение уникальных параметров

        :param cs_data: DataFrame данных со СУ
        :return: список уникальных параметров
        """
        if type(cs_data) == int:
            cs_data = self.init_pandas_df
        self.parametrs_list = np.unique(cs_data[1])
        return self.parametrs_list

    def get_information(self, cs_data=-1, parametrs_list=-1):
        """
        Получение информации через print о данных

        :param cs_data: DataFrame данных со СУ
        :param parametrs_list: лист имен уникальных параметров в cs_data
        :return: порядковый номер параметра в списке и количество записей в cs_data
        """
        if type(cs_data) == int:
            cs_data = self.init_pandas_df
        if type(parametrs_list) == int:
            parametrs_list = self.parametrs_list
        for i in range(len(parametrs_list)):
            amount_of_parametr_values = len(cs_data[cs_data[1] == parametrs_list[i]])
            print("№" + str(i) + " Кол-во записей: " + str(amount_of_parametr_values) + '  ' + str(parametrs_list[i]))

    def initial_editing(self, str_to_delete, data=-1):
        """
        Первоначальная обработка данных: удаление лишнего столбца, приведение индекса ко времени,
        изменение столбца с названием параметров

        :param str_to_delete: строка для удаления в столбце
        :param data: DataFrame данных со СУ
        :return: измененный DataFrame
        """
        if type(data) == int:
            data = self.init_pandas_df
        del data[0]
        self.str_to_delete = str_to_delete
        data[1] = data[1].str.replace(self.str_to_delete, "")
        data.index = data[2]
        del data[2]
        data.index = pd.to_datetime(data.index)
        return data

    def extract_df_one_parametr_and_edit(self, data, list_of_params, number_of_param_in_list):
        """
        Извлечение из общих данных DataFrame c единственным параметром по его номеру в списке параметров

        :param data: DataFrame
        :param list_of_params: список параметров в данном DataFrame
        :param number_of_param_in_list: номер параметра в списке
        :return: сформированный DataFrame с единственный параметром
        """
        extracted_df_one_param = data[data[1] == list_of_params[number_of_param_in_list]].copy()
        edited_df_one_param = extracted_df_one_param.rename(index=str, columns={3: extracted_df_one_param[1][1]})
        del edited_df_one_param[1]
        return edited_df_one_param

    def create_edited_df(self, prepared_data, parametrs_list_for_prepared_data):
        """
        Итоговая обработка данных после предварительной обработки

        :param prepared_data: подготовленные данные
        :param parametrs_list_for_prepared_data: список параметров в подготовленных данных
        :return: готовый DataFrame
        """
        init_one_parametr_df = self.extract_df_one_parametr_and_edit(prepared_data, parametrs_list_for_prepared_data, 0)
        result = init_one_parametr_df
        for i in range(1, len(parametrs_list_for_prepared_data)):
            new_one_parametr_df = self.extract_df_one_parametr_and_edit(prepared_data, parametrs_list_for_prepared_data, i)
            result = result.join([new_one_parametr_df], sort=True, how="outer")
        return result

    def save_edited_df(self, edited_df, csv_name_str):
        """
        Сохранение итогового отредактированного DataFrame

        :param edited_df: итоговый отредактированный DataFrame
        :param csv_name_str: имя файлика в директории, data.csv
        :return: None
        """
        edited_df.to_csv(csv_name_str, index=True, index_label="Time")
