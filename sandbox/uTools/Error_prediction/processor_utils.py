import pandas as pd


def get_fragmentation(length: int, slaves: int) -> list:
    """
    with length of jobs and amount of slaves returns list with turples.
    job_list[out[slave][0]: out[slave][1]] is a sub-list of jobs for this slave. slave in [0, slaves-1]
    :param length: number of jobs
    :param slaves: amount of slaves
    :return: turples to distribute jobs
    """
    # defining step size
    step = int(length / slaves)
    # but int-like devision returns a bit less
    r = length - step * slaves
    # preparing out list with turples. r will be distributed over first slaves
    out = []
    cur_point = 0
    for i in range(slaves):
        if r > 0:
            # if we have non dealed r, push a piece
            out_el = (cur_point, cur_point + step + 1)
            r -= 1
        else:
            # if all the r already distributed then ok
            out_el = (cur_point, cur_point + step)
        cur_point = out_el[1]
        out.append(out_el)
    return out


def divide_prepared_data(prepared_data, options):
    """
    Разбивка всех исходных данных на равные части для реализации многопоточности
    :param prepared_data: подготовленные входные данные
    :param options: класс настроек расчета
    :return: определенная часть исходных данных, которая будет считаться данным потоком
    """
    fragmentation = get_fragmentation(prepared_data.shape[0], options.amount_of_threads)
    out = prepared_data[fragmentation[options.number_of_thread - 1][0]: fragmentation[options.number_of_thread - 1][1]]
    return out


def create_new_result_df(this_result, this_state, prepared_data, i):
    """
    Объединение всех результатов для данной итерации в один DataFrame
    :param this_result: список с результатами UniflocVBA
    :param this_state: класс-состояние скважины для данной итерации (входные данные)
    :param prepared_data: DataFrame входных данных
    :param i: номер строки в prepared_data для текущей итерации
    :return: out - сводный результат расчета
    """
    new_dict = {}
    for j in range(len(this_result[1])):
        new_dict[this_result[1][j]] = [this_result[0][j]]
        print(str(this_result[1][j]) + " -  " + str(this_result[0][j]))
    new_dict['ГФ'] = [this_state.rp_m3m3]
    new_dict['Значение функции ошибки'] = [this_state.error_in_step]
    new_dict['Время'] = [prepared_data.index[i]]
    out = pd.DataFrame(new_dict)
    out.index = out['Время']
    return out


def transfer_data_from_row_to_state(this_state, row_in_prepared_data, vfm_calc_option):
    """
    заполнение класса-состояния скважины с ЭЦН текущим набором входных данных (для данного момента времени)
    :param this_state: состояние скважины со всеми параметрами
    :param row_in_prepared_data: набора данных - строка входного DataFrame
    :param vfm_calc_option: флаг восстановления дебитов - если False - адаптация
    :return: заполненное состояние this_state
    """
    this_state.watercut_perc = row_in_prepared_data['Процент обводненности (СУ)']  # заполнение структуры данными
    this_state.rp_m3m3 = row_in_prepared_data['ГФ (СУ)']
    this_state.p_buf_data_atm = row_in_prepared_data['Рбуф (Ш)']
    this_state.p_wellhead_data_atm = row_in_prepared_data['Линейное давление (СУ)'] * 10
    this_state.tsep_c = row_in_prepared_data['Температура на приеме насоса (пласт. жидкость) (СУ)']
    this_state.p_intake_data_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
    this_state.psep_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
    this_state.p_wf_atm = row_in_prepared_data['Давление на приеме насоса (пласт. жидкость) (СУ)'] * 10
    this_state.d_choke_mm = row_in_prepared_data['Dшт (Ш)']
    this_state.ESP_freq = row_in_prepared_data['F вращ ТМ (Ш)']
    this_state.active_power_cs_data_kwt = row_in_prepared_data['Активная мощность (СУ)'] * 1000
    this_state.u_motor_data_v = row_in_prepared_data['Напряжение на выходе ТМПН (СУ)']
    this_state.cos_phi_data_d = row_in_prepared_data['Коэффициент мощности (СУ)']
    if vfm_calc_option:
        this_state.c_calibr_head_d = 1  # row_in_prepared_data["К. калибровки по напору - множитель (Модель) (Подготовленные)"]
        this_state.c_calibr_power_d = 1  # row_in_prepared_data["К. калибровки по мощности - множитель (Модель) (Подготовленные)"]
    else:
        this_state.qliq_m3day = row_in_prepared_data['Объемный дебит жидкости (СУ)']
    return this_state

