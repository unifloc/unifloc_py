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
        self.result_df = None

    def __extract_data(self, this_class, init_class_name):
        """
        Рекурсивное извлечение __dict__ из объкта класса
        :param this_class:
        :param init_class_name:
        :return:
        """
        all_class_dicts_well = []

        def rec(this_class, level=1, init_class_name=init_class_name):
            # print(f"Уровень{level}")
            this_class_dict = this_class.__dict__
            all_class_dicts_well.append({init_class_name: this_class_dict})
            for i, j in this_class_dict.items():
                if "uniflocpy" in str(type(j)):
                    # print(f"Спускаемся в {i}")
                    this_class_dict = rec(j, level + 1, init_class_name=i)
                    # print(f"Возвращаемся из {i}")
            return this_class_dict

        ___ = rec(this_class)

        return all_class_dicts_well

    def __combine_object_dicts_into_one(self, data):
        """
        Объединение списка словаре в один большой словарь, с пометками исходных keys
        :param data:
        :return:
        """
        super_dict = {}
        for i in data:
            if list(i.keys())[0] != 'data':
                object_name = list(i.keys())[0]
                parameter_mark = f"{object_name}."
                for j, k in i[object_name].items():
                    super_dict[parameter_mark + j] = k
        return super_dict

    def __leave_numbers(self, data):
        """
        Чистка словаря от ненужного, оставляются только числа
        :param data:
        :return:
        """
        new_data = {}
        for i, j in data.items():
            if str(type(j)) == "<class 'numpy.ndarray'>":
                if type(j.tolist()) == float or type(j.tolist()) == int:
                    new_data[i] = float(j)
                elif len(j) == 1:
                    new_data[i] = float(j)
                else:
                    pass
            elif type(j) == int or type(j) == float or str(type(j)) == "<class 'numpy.float64'>" or str(
                    type(j)) == "<class 'NoneType'>" or type(j) == bool:
                new_data[i] = j
            else:
                pass
        return new_data

    def extract_data_from_object(self, this_object, object_name='well', index_to_df=[0]):
        """
        Глубокое извлечение всех атрибутов и выдача чистого DataFrame
        :param this_object:
        :param object_name:
        :param index_to_df:
        :return:
        """
        data = self.__extract_data(this_object, object_name)
        data = self.__combine_object_dicts_into_one(data)
        data = self.__leave_numbers(data)
        df = pd.DataFrame(data, index=index_to_df)
        return df

    def get_data(self, method_or_correlation, object_name='well'):
        """
        Получение атрибутов (всех данных) из расчетного класса
        """
        this_df = self.extract_data_from_object(method_or_correlation, object_name=object_name)
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
        self.df_list = []
        self.result_df = None

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

    def __combine_df_from_objects(self, df_list):
        """
        Объединение DataFrame-состояний в один DataFrame
        :param df_list:
        :return:
        """
        for i in df_list:
            try:
                result_df = result_df.append(i, sort=False)
            except:
                result_df = df_list[0]
        return result_df

    def get_data_as_df(self):
        self.result_df = self.__combine_df_from_objects(self.df_list)
        return self.result_df


translation_dict = {'Давление, бар': 'p_calculated_bar',
                    'Температура, С': 't_calculated_c',
                    'Температура окр. среды, С': 't_calculated_earth_init',
                    'Расходная доля газа в потоке, д.ед.': 'gas_fraction_d',
                    'Истинная доля газа в потоке, д.ед.': 'gas_fraction_real_d',
                    'Расходная плотность ГЖС, кг/м3': 'rhon_kgm3',
                    'Расходная вязкость ГЖС, Па*сек': 'mun_pas',
                    'Расходная вязкость ГЖС, сПуаз': 'mun_cP',
                    'Составляющая градиента по гравитации, Па/м': 'density_grad_pam',
                    'Доля градиента по гравитации, %': 'density_grad_part_percent',
                    'Калибровочный коэффициент на трение, ед.': 'friction_grad_coef',
                    'Составляющая градиента по трению, Па/м': 'friction_grad_pam',
                    'Доля градиента по трению, %': 'friction_grad_part_percent',
                    'Число Рейнольдса, безр.': 'number_re_n',
                    'Давление насыщения, бар': 'pb_bar',
                    'Давление насыщения для rupvt, бар': 'pb_bar_for_rus_cor',
                    'Приведенная скорость жидкости, м/сек': 'vsl_msec',
                    'Приведенная скорость газа, м/сек': 'vsg_msec',
                    'Приведенная скорость ГЖС, м/сек': 'vm_msec',
                    'Градиент давления, Па/м': 'result_grad_pam',
                    'Калибровочный коэффициент на ускорение, ед.': 'acceleration_grad_coef',
                    'Флаг использования ускорения в градиенте': 'acceleration_grad_using',
                    'Составляющая градиента по ускорению, Па/м': 'acceleration_grad_pam',
                    'Доля градиента по ускорению, %': 'acceleration_grad_part_percent',
                    'Газосодержание при Pнас, м3/т': 'rsb_m3t',
                    'Удельный объем выделившегося газа, м3/т': 'gas_liberated_m3t',
                    'Удельный объем растворенного газа, м3/т': 'gas_dissolved_m3t',
                    'Объемный коэффициент нефти, м3/м3': 'b_oil_m3m3',
                    'Плотность нефти, кг/м3': 'rho_oil_kgm3',
                    'Вязкость нефти, сПуаз': 'mu_oil_cp',
                    'Модель флюида.Коэффициент сверхсжимаемости газа': 'fl.z',
                    'Относительная плотность газа, растворенного в нефти': 'rho_gas_dissolved_relative_d',
                    'Измеренная глубина, м':'h_calculated_mes_m',
                    'Удельный расход газа, м3/м3': 'r_gas_injected_m3m3'

                    }


def rename_columns_by_dict(this_df, columns_name_dict=translation_dict):
    """
    Специальное изменение названий столбцов по словарю
    :param df:
    :param dict: словарь со значениями типа rus:eng
    :return:
    """
    df = this_df.copy()
    for i in df.columns:
        for items in columns_name_dict.items():
            pattern_to_change = items[1]
            if pattern_to_change in i:
                new_name = i.replace(pattern_to_change, items[0])
                df = df.rename(columns={i: new_name})
    return df