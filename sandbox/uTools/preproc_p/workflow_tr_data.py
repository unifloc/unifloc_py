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


def to_float(string):
    try:
        string = string.replace(',', '.')
        string = string.replace(' ', '')
        this_float = float(string)
    except:
        this_float = float(string)
    return this_float


class Tr_data:
    def __init__(self):
        """
        Класс-структура для хранения данных о скважине из техрежима
        :param row: строка, извлеченная с техрежима для данной скважины
        """
        self.d_cas_mm = None
        self.d_tube_mm = None
        self.esp_nom_rate_m3day = None
        self.esp_nom_head_m = None
        self.h_pump_m = None
        self.esp_name_str = None
        self.udl_m = None
        self.i_motor_nom_a = None
        self.motor_name_str = None
        self.power_motor_nom_kwt = None

    def fill_by_true_tr(self, row):
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

    def fill_by_grad(self, this_well_row):
        self.d_cas_mm = to_float(this_well_row['Диаметр экспл.колонны'].values[0])
        self.d_tube_mm = to_float(this_well_row['Диаметр НКТ'].values[0])
        self.esp_nom_rate_m3day = to_float(this_well_row['Производительность ЭЦН'].values[0])
        self.esp_nom_head_m = to_float(this_well_row['Напор'].values[0])
        self.h_pump_m = to_float(this_well_row['Глубина спуска'].values[0])
        self.esp_name_str = this_well_row['Тип насоса'].values[0]
        self.udl_m = to_float(this_well_row['Глубина спуска'].values[0]) / \
                     to_float(this_well_row['Глубина верхних дыр перфорации'].values[0]) * \
                     to_float(this_well_row['Удлинение'].values[0])
        self.i_motor_nom_a = to_float(this_well_row['Ток номинальный'].values[0])
        if self.i_motor_nom_a > 0:
            pass
        else:
            self.i_motor_nom_a = to_float(this_well_row['Ток рабочий'].values[0]) * 1.5
            print('Нет значения номинального тока двитателя!')
            if self.i_motor_nom_a > 0:
                pass
            else:
                self.i_motor_nom_a = 30
                print('Нет значения рабочего тока двитателя!')

    def fill_by_pump_base(self, this_well_row):
        self.motor_name_str = this_well_row['Длина'].values[0]
        self.power_motor_nom_kwt = extract_power_from_motor_name(self.motor_name_str)

def read_tr_and_get_data(tr_file_full_path, well_name,
                         pump_base_path='БОМД. Оборудование актуальных паспортов_Филиал Муравленковскнефть ОАО Газпромнефть-ННГ_30.12.2019.xlsx',
                         field='Вынгаяхинское'):
    """
    Чтение техрежима и извлечение данных по скважине
    :param tr_file_full_path: абсолютный путь техрежиму
    :param well_name: номер скважины, str
    :return: класс-структура со всем данными
    """
    try:
        tr = pd.read_excel(tr_file_full_path, skiprows=6,
                           header=[0, 1, 2, 3])  # при ошибке файл нужно открыть и сохранить повторно без изменений
        this_well_row = tr[
            tr[('№\nскв', 'Unnamed: 4_level_1', 'Unnamed: 4_level_2', 'Unnamed: 4_level_3')] == well_name]
        tr_data = Tr_data()
        tr_data.fill_by_true_tr(this_well_row)
    except:
        tr = pd.read_csv(tr_file_full_path, engine='c', delimiter=';', skiprows=2)
        this_well_row = tr[tr['Скважина'] == well_name]
        if this_well_row.shape[0] == 0:
            this_well_row = tr[tr['Скважина'] == well_name + '_1']
        if this_well_row.shape[0] == 2:
            this_well_row = this_well_row[this_well_row.index == this_well_row.index[0]]
        tr_data = Tr_data()
        tr_data.fill_by_grad(this_well_row)

        pump_base = pd.read_excel(pump_base_path, skiprows=3)
        this_well_row = pump_base[(pump_base['№ скважины'] == well_name) & (pump_base['Месторождение'] == field)]
        tr_data.fill_by_pump_base(this_well_row)
    return tr_data