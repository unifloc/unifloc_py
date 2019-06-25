"""
Модуль-интерфейс-инструмент для извлечения данных из расчетов разных корреляций
"""

class Data():

    def __init__(self):
        self.dicts = []  # список словарей атрибутов класса
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
        self.dicts.append(this_data)
        for key, value in this_data.items():  # преобразование словаря в итерируемый список
            temp = [key, value]
            self.lists.append(temp)
        self.amounts_of_param_in_one_banch = len(self.dicts[0])

    def clear_data(self):
        """
        Очистка данных
        """
        self.dicts = []
        self.lists = []
        self.h_list = []
        self.p_list = []
        self.t_list = []

    def get_values(self, number):
        """
        Получение массива определенного параметра по его номеру
        """
        self.amounts_of_param_in_one_banch = len(self.dicts[0])
        amounts_of_banches = len(self.lists) / self.amounts_of_param_in_one_banch
        one_parametr = []
        k = number
        for i in range(int(amounts_of_banches)):
            one = self.lists[k]
            k = k + self.amounts_of_param_in_one_banch
            one_parametr.append(one)
        values = []
        for i in one_parametr:
            values.append(i[-1])
        return values

    def get_name(self, number):
        """
        Получение названия параметра по его номеру
        """
        return self.lists[number][0]

    def print_all_names(self):
        """print всех параметров их их номеров для получения"""
        for i in range(self.amounts_of_param_in_one_banch):
            print('Номер ' + str(i) + ' для получения параметра ' + self.get_name(i))
