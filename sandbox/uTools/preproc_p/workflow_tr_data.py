import pandas as pd


def extract_power_from_motor_name(name_str):
    """
    Определение мощности двигателя по его шифру из строки техрежима
    :param name_str: строка из техрежима, например 'ПЭД-100-117"
    :return: мощность двигателя, кВт (100 для данного ПЭД)
    """
    try:
        name_str = name_str.upper()
        name_str = name_str.replace('9.8.4ЭДБТ ', '')
        name_str = name_str.replace('9.1ВЭДБТ', '')
        name_str = name_str.replace(' ', '')
        name_str = name_str.replace('ПЭДНС', '')
        name_str = name_str.replace('9ЭДБТК', '')
        name_str = name_str.replace('ПЭДC', '')
        name_str = name_str.replace('ПЭДН', '')
        name_str = name_str.replace('9.8.4ЭДБТ', '')
        name_str = name_str.replace('ПВЭДН', '')
        name_str = name_str.replace('9.11ВЭДБ', '')
        name_str = name_str.replace('9ЭДБСТ', '')
        name_str = name_str.replace('9ЭДБТ', '')
        name_str = name_str.replace('ЭДБТ', '')
        name_str = name_str.replace('9.1ВЭДБТ', '')
        name_str = name_str.replace('ПЭД', '')
        if name_str[0] == '-':
            name_str = name_str[1:]
        if name_str[3] == '-':
            name_str = name_str[0:3]
        elif name_str[2] == '-':
            name_str = name_str[0:2]
        return float(name_str)
    except:
        float(100)


def to_float(string):
    try:
        string = string.replace(',', '.')
        string = string.replace(' ', '')
        this_float = float(string)
    except:
        this_float = float(string)
    return this_float


class Static_data:
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
        self.esp_id = None
        self.udl_m = None
        self.i_motor_nom_a = None
        self.motor_name_str = None
        self.power_motor_nom_kwt = None

        self.gamma_oil = None
        self.gamma_gas = None
        self.gamma_wat = None
        self.rsb_m3m3 = None
        self.tres_c = None
        self.pb_atm = None
        self.bob_m3m3 = None
        self.muob_cp = None
        self.rp_m3m3 = None

        self.qliq_m3day_initial_guess = None
        self.c_calibr_head_d_initial_guess = None
        self.c_calibr_power_d_initial_guess = None

        self.c_calibr_head_d_max_limit = None
        self.c_calibr_head_d_min_limit = None
        self.c_calibr_power_d_max_limit = None
        self.c_calibr_power_d_min_limit = None

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
                         field='Восточно-Пякутинское'):
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
        tr_data = Static_data()
        tr_data.fill_by_true_tr(this_well_row)
    except:
        tr = pd.read_csv(tr_file_full_path, engine='c', delimiter=';', skiprows=2)
        this_well_row = tr[tr['Скважина'] == well_name]
        if this_well_row.shape[0] == 0:
            this_well_row = tr[tr['Скважина'] == well_name + '_1']
        if this_well_row.shape[0] == 2:
            this_well_row = this_well_row[this_well_row.index == this_well_row.index[0]]
        tr_data = Static_data()
        tr_data.fill_by_grad(this_well_row)

        pump_base = pd.read_excel(pump_base_path, skiprows=3)
        this_well_row = pump_base[(pump_base['№ скважины'] == well_name) & (pump_base['Месторождение'] == field)]
        tr_data.fill_by_pump_base(this_well_row)
    return tr_data


def read_pvt_file_and_fill_tr_data(pvt_file_full_path: str, well_name: str, tr_data: Static_data):
    pvt_file = pd.read_excel(pvt_file_full_path)
    try:
        this_well_row = pvt_file[pvt_file['пласт/скважина'] == well_name]
        if this_well_row.shape[0] == 0:
            this_well_row = pvt_file[pvt_file['пласт/скважина'] == 'м']
    except:
        this_well_row = pvt_file[pvt_file['пласт/скважина'] == 'м']
    tr_data.gamma_oil = to_float(this_well_row['gamma_oil'].values[0])
    tr_data.gamma_gas = to_float(this_well_row['gamma_gas'].values[0])
    tr_data.gamma_wat = to_float(this_well_row['gamma_wat'].values[0])
    tr_data.rsb_m3m3 = to_float(this_well_row['rsb_m3m3'].values[0])
    tr_data.tres_c = to_float(this_well_row['tres_c'].values[0])
    tr_data.pb_atm = to_float(this_well_row['pb_atm'].values[0])
    tr_data.bob_m3m3 = to_float(this_well_row['bob_m3m3'].values[0])
    tr_data.muob_cp = to_float(this_well_row['muob_cp'].values[0])
    tr_data.rp_m3m3 = to_float(this_well_row['rp_m3m3'].values[0])
    return tr_data


def fill_static_data_structure_by_df(static_data: Static_data,
                                     static_data_df: pd.DataFrame,
                                     chosen_column_name: str):
    this_static_data_series = static_data_df.set_index(static_data_df['Параметр'])[chosen_column_name]
    static_data.d_cas_mm = this_static_data_series.d_cas_mm
    static_data.d_tube_mm = this_static_data_series.d_tube_mm
    static_data.udl_m = this_static_data_series.udl_m

    static_data.esp_nom_rate_m3day = this_static_data_series.esp_nom_rate_m3day
    static_data.esp_nom_head_m = this_static_data_series.esp_nom_head_m
    static_data.h_pump_m = this_static_data_series.h_pump_m
    static_data.esp_name_str = this_static_data_series.esp_name_str
    static_data.esp_id = this_static_data_series.esp_id

    static_data.i_motor_nom_a = this_static_data_series.i_motor_nom_a
    static_data.motor_name_str = this_static_data_series.motor_name_str
    static_data.power_motor_nom_kwt = this_static_data_series.power_motor_nom_kwt

    static_data.gamma_oil = this_static_data_series.gamma_oil
    static_data.gamma_gas = this_static_data_series.gamma_gas
    static_data.gamma_wat = this_static_data_series.gamma_wat
    static_data.rsb_m3m3 = this_static_data_series.rsb_m3m3
    static_data.tres_c = this_static_data_series.tres_c
    static_data.pb_atm = this_static_data_series.pb_atm
    static_data.bob_m3m3 = this_static_data_series.bob_m3m3
    static_data.muob_cp = this_static_data_series.muob_cp
    static_data.rp_m3m3 = this_static_data_series.rp_m3m3

    static_data.qliq_m3day_initial_guess = this_static_data_series.qliq_m3day_initial_guess
    static_data.c_calibr_head_d_initial_guess = this_static_data_series.c_calibr_head_d_initial_guess
    static_data.c_calibr_power_d_initial_guess = this_static_data_series.c_calibr_power_d_initial_guess
    static_data.c_calibr_head_d_max_limit = this_static_data_series.c_calibr_head_d_max_limit
    static_data.c_calibr_head_d_min_limit = this_static_data_series.c_calibr_head_d_min_limit
    static_data.c_calibr_power_d_max_limit = this_static_data_series.c_calibr_power_d_max_limit
    static_data.c_calibr_power_d_min_limit = this_static_data_series.c_calibr_power_d_min_limit

    return static_data


