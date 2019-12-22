import pandas as pd


def extract_power_from_motor_name(name_str):
    """
    Определение мощности двигателя по его шифру из строки техрежима
    :param name_str: строка из техрежима, например 'ПЭД-100-117"
    :return: мощность двигателя, кВт (100 для данного ПЭД)
    """
    name_str = name_str.upper()
    name_str = name_str.replace('9.8.4ЭДБТ ', '')
    name_str = name_str.replace(' ', '')
    name_str = name_str.replace('ПЭДНС', '')
    name_str = name_str.replace('9ЭДБТК', '')
    name_str = name_str.replace('ПЭДC', '')
    name_str = name_str.replace('ПЭДН', '')
    name_str = name_str.replace('9.8.4ЭДБТ', '')
    name_str = name_str.replace('ПВЭДН', '')
    name_str = name_str.replace('9ЭДБСТ', '')
    name_str = name_str.replace('9ЭДБТ', '')
    name_str = name_str.replace('ЭДБТ', '')
    name_str = name_str.replace('ПЭД', '')
    if name_str[0] == '-':
        name_str = name_str[1:]
    if name_str[3] == '-':
        name_str = name_str[0:3]
    elif name_str[2] == '-':
        name_str = name_str[0:2]
    return float(name_str)


class Tr_data:
    def __init__(self, row):
        """
        Класс-структура для хранения данных о скважине из техрежима
        :param row: строка, извлеченная с техрежима для данной скважины
        """
        self.d_cas_mm = row[('D э/к', 'Unnamed: 9_level_1', 'Unnamed: 9_level_2', 'мм')].values[0]
        self.d_tube_mm = row[('D нкт', 'Unnamed: 10_level_1', 'Unnamed: 10_level_2', 'мм')].values[0]
        self.esp_nom_rate_m3day = \
        row[('Номинальная\nпроизводительность', 'Unnamed: 16_level_1', 'Unnamed: 16_level_2', 'м3/сут')].values[0]
        self.esp_nom_head_m = row[('Номинальный напор', 'Unnamed: 17_level_1', 'Unnamed: 17_level_2', 'м')].values[0]
        self.h_pump_m = row[('Н сп', 'Unnamed: 20_level_1', 'Unnamed: 20_level_2', 'м')].values[0]
        self.esp_name_str = \
        row[('Тип насоса', 'Unnamed: 15_level_1', 'Unnamed: 15_level_2', 'Unnamed: 15_level_3')].values[0]
        self.udl_m = row[('Удл (Нсп)', 'Unnamed: 161_level_1', 'Unnamed: 161_level_2', 'м')].values[0]
        self.i_motor_nom_a = row[('ПЭД', 'Unnamed: 131_level_1', 'I ном', 'А')].values[0]
        self.motor_name_str = row[('ПЭД', 'Unnamed: 128_level_1', 'Марка', 'Unnamed: 128_level_3')].values[0]
        self.power_motor_nom_kwt = extract_power_from_motor_name(
            row[('ПЭД', 'Unnamed: 128_level_1', 'Марка', 'Unnamed: 128_level_3')].values[0])


def read_tr_and_get_data(tr_file_full_path, well_name):
    """
    Чтение техрежима и извлечение данных по скважине
    :param tr_file_full_path: абсолютный путь техрежиму
    :param well_name: номер скважины, str
    :return: класс-структура со всем данными
    """
    tr = pd.read_excel(tr_file_full_path, skiprows=6,
                       header=[0, 1, 2, 3])  # при ошибке файл нужно открыть и сохранить повторно без изменений
    this_well_row = tr[tr[('№\nскв', 'Unnamed: 4_level_1', 'Unnamed: 4_level_2', 'Unnamed: 4_level_3')] == well_name]
    tr_data = Tr_data(this_well_row)
    return tr_data