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
class Data():

    def __init__(self):
        self.saved_dicts = []  # список словарей атрибутов класса
        self.lists = []  # списки для хранения
        self.h_list = []
        self.p_list = []
        self.t_list = []
        self.distance_list = []
        self.amounts_of_param_in_one_banch = None

    def get_data(self, method_or_correlation):
        """
        Получение атрибутов (всех данных) из расчетного класса
        """

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

