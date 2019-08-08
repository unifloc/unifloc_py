"""
Модуль-интерфейс-инструмент для извлечения данных из расчетов разных корреляций
"""

#TODO переделать получение значений: вместо по списка обращаться к словарям по ключам

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
            values.append(float(i[-1]))
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

    def save_data_from_class_to_storage(self, class_obj):
        this_data = class_obj.__dict__.copy()
        self.dicts.append(this_data)
        self.amounts_of_param_in_one_banch = len(self.dicts[0])

    def print_all_names_of_saved_parameters(self):
        for number_of_key, key_str in enumerate(self.dicts[0].keys()):
            print('Номер ' + str(number_of_key) + ' для получения параметра ' + key_str)

    def get_saved_parameter_name_by_number(self, parameter_number):
        dict_keys = self.dicts[0].keys()
        keys_list = list(dict_keys)
        parameter_name = keys_list[parameter_number]
        return parameter_name

    def get_saved_values_by_number(self, parameter_number):
        parameter_name = self.get_saved_parameter_name_by_number(parameter_number)
        values = []
        for one_dict in self.dicts:
            one_parameter_value = one_dict[parameter_name]
            values.append(float(one_parameter_value))
        return values


import uniflocpy.uWell.uPipe as Pipe


pipe = Pipe.Pipe()
pipe_data = Data()
p_initial_mpa = 15
t_initial_c = 90
h_initial_m = 2000
step_m = 10
step_cm = 0.03
amount_of_steps = int(h_initial_m / step_m)

pipe.fluid_flow.qliq_on_surface_m3day = 100
pipe.fluid_flow.fw_on_surface_perc = 20

h_list = [h_initial_m]
p_list = [p_initial_mpa]
t_list = [t_initial_c]


pipe_data.clear_data()

for i in range(amount_of_steps):
    if p_list[-1] > 0.101250:
        p_bar = p_list[-1] * 10
        t_c = t_list[-1]

        grad_pam = pipe.calc_p_grad_pam(p_bar, t_c)

        p = p_list[-1] - step_m * grad_pam / 10 ** 6
        t = t_list[-1] - step_m * step_cm
        h = h_list[-1] - step_m

    if p > 0.1:
        p_list.append(p)
    else:
        p_list.append(p_list[-1])

    t_list.append(t)
    h_list.append(h)

    pipe_data.save_data_from_class_to_storage(pipe)
    pipe_data.h_list.append(h)
    pipe_data.t_list.append(t)
    pipe_data.p_list.append(p)

pipe_data.print_all_names_of_saved_parameters()
pipe_data.get_saved_parameter_name_by_number(9)
print(pipe_data.get_saved_values_by_number(9))

print(str(type(pipe_data.dicts[0]["fluid_flow"])).replace("unifloc","check"))