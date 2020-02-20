"""
Кобзарь О.С. Водопьян А.О. Хабибуллин Р.А. 09.08.2019

Модуль-интерфейс-инструмент для извлечения данных из расчетных классов
"""
#  TODO обеспечить хранение внутренних классов на любом уровне вложенности, пока сохраняется только верхний
#  TODO использоавать встроенную библиотеку collection.defaultdict
#  TODO после полной проработки сохарнения изменить примеры и удалить старые методы
#  TODO пока не сделано, сохранять явно каждый класс
#  TODO обеспечить возможность сохранения данных в pandas.dataframe
#  TODO обеспечить возможность сохранения данных в .csv на лету (чтобы оставались расчеты при ошибке в середине большого расчета)
import pandas as pd
class Data():

    def __init__(self):
        self.saved_dicts = []  # список словарей атрибутов класса
        self.lists = []  # списки для хранения
        self.h_list = []
        self.p_list = []
        self.t_list = []
        self.distance_list = []
        self.amounts_of_param_in_one_banch = None
        self.df_list = []


    def __extract_data(self, this_class, init_class_name):
        all_class_dicts_well = []

        def extract_recursicely_data_from_obj(this_class):
            this_class_dict = this_class.__dict__
            for i, j in this_class_dict.items():
                if "uniflocpy" in str(type(j)):
                    this_class_dict = extract_recursicely_data_from_obj(j)
                    all_class_dicts_well.append({i: this_class_dict})
            return this_class_dict

        ___ = extract_recursicely_data_from_obj(this_class)
        all_class_dicts_well.append({init_class_name: this_class.__dict__})
        return all_class_dicts_well

    def __combine_object_dicts_into_one(self, data):
        super_dict = {}
        casing_pipe_object_not_passed = True
        for i in data:
            if list(i.keys())[0] != 'data':
                object_name = list(i.keys())[0]
                if object_name == "casing_pipe":
                    casing_pipe_object_not_passed = False
                if object_name != "well_profile" and object_name != "well" and casing_pipe_object_not_passed:
                    parameter_mark = f"casing_pipe.{object_name}."
                elif object_name != "well_profile" and object_name != "well" and not casing_pipe_object_not_passed:
                    parameter_mark = f"tube_pipe.{object_name}."
                else:
                    parameter_mark = f"{object_name}."
                for j, k in i[object_name].items():
                    super_dict[parameter_mark + j] = k
        return super_dict

    def __leave_numbers(self, data):
        new_data = {}
        for i, j in data.items():
            if str(type(j)) == "<class 'numpy.ndarray'>":

                if len(j) == 1:
                    new_data[i] = float(j)
                else:
                    pass
            elif type(j) == int or type(j) == float or str(type(j)) == "<class 'numpy.float64'>" or str(
                    type(j)) == "<class 'NoneType'>" or type(j) == bool:
                new_data[i] = j
            else:
                pass
        return new_data

    def __extract_data_from_object(self, this_object, object_name='well', index_to_df=[0]):
        data = self.__extract_data(this_object, object_name)
        data = self.__combine_object_dicts_into_one(data)
        data = self.__leave_numbers(data)
        df = pd.DataFrame(data, index=index_to_df)
        return df

    def get_data(self, method_or_correlation):
        """
        Получение атрибутов (всех данных) из расчетного класса
        """
        this_df = self.__extract_data_from_object(method_or_correlation)
        self.df_list.append(this_df)
        this_data = method_or_correlation.__dict__
        self.saved_dicts.append(this_data)
        for key, value in this_data.items():  # преобразование словаря в итерируемый список
            temp = [key, value]
            self.lists.append(temp)
        self.amounts_of_param_in_one_banch = len(self.saved_dicts[0])


    def clear_data(self):
        """
        Очистка данных
        """
        self.saved_dicts = []
        self.lists = []
        self.h_list = []
        self.p_list = []
        self.t_list = []

    def get_values(self, number):
        """
        Старый метод: использовать save_data_from_class_to_storage

        Получение массива определенного параметра по его номеру
        """
        self.amounts_of_param_in_one_banch = len(self.saved_dicts[0])
        amounts_of_banches = len(self.lists) / self.amounts_of_param_in_one_banch
        one_parametr = []
        k = number
        for i in range(int(amounts_of_banches)):
            one = self.lists[k]
            k = k + self.amounts_of_param_in_one_banch
            one_parametr.append(one)
        values = []
        for i in one_parametr:
            values.append(float(i[-1]))
        return values

    def get_name(self, number):
        """
        Старый метод: использовать get_saved_parameter_name_by_number

        Получение названия параметра по его номеру
        """
        return self.lists[number][0]

    def print_all_names(self):
        """
        Старый метод: использовать print_all_names_of_saved_parameters

        print всех параметров их их номеров для получения
        """
        for i in range(self.amounts_of_param_in_one_banch):
            print('Номер ' + str(i) + ' для получения параметра ' + self.get_name(i))

    def save_data_from_class_to_storage(self, class_obj):
        """
        Получение всех атрибутов расчетного класса в форме словаря. Словари хранятся в списке.

        :param class_obj: расчетный класс
        :return: None
        """
        this_data = class_obj.__dict__.copy()
        self.saved_dicts.append(this_data)
        self.amounts_of_param_in_one_banch = len(self.saved_dicts[0])

    def print_all_names_of_saved_parameters(self):
        """
        Получение названий атрибутов и соответствующих им номеров (индексов) для обращения

        :return: print номеров всех атрибутов(сохраненных параметров)
        """
        for number_of_key, key_str in enumerate(self.saved_dicts[0].keys()):
            print('Номер ' + str(number_of_key) + ' для получения параметра ' + key_str)

    def get_saved_parameter_name_by_number(self, parameter_number):
        """
        Получение имени (ключа) параметра по его номеру(индексу)

        :param parameter_number: номер параметра(атрибута), int
        :return: имя параметра (ключа), str
        """
        dict_keys = self.saved_dicts[0].keys()
        keys_list = list(dict_keys)
        parameter_name = keys_list[parameter_number]
        return parameter_name

    def get_saved_values_by_number(self, parameter_number):
        """
        Получение списка значений только для одного параметра.
        Спискок значений формируется из словарей, сохраненных при расчете

        :param parameter_number: номер параметра(атрибута), int
        :return: список значений параметра, list
        """
        parameter_name = self.get_saved_parameter_name_by_number(parameter_number)
        values = []
        for one_dict in self.saved_dicts:
            one_parameter_value = one_dict[parameter_name]
            values.append(float(one_parameter_value))
        return values



    def combine_df_from_objects(self, df_list):
        for i in df_list:
            try:
                result_df = result_df.append(i, sort=False)
            except:
                result_df = df_list[0]
        return result_df

