import pandas as pd
import sys
import os

sys.path.append('../' * 4)
sys.path.append('../' * 3)
sys.path.append('../' * 2)
import sandbox.uTools.preproc_p.preproc_tool as preproc_tool
import unifloc.sandbox.uTools.preproc_p.preproc_tool as preproc_tool

global_names = preproc_tool.GlobalNames()


def create_banches_for_report(well_data, report_type):
    """
    Создание списка словарей для построения графиков в форме report
    :param well_data: DataFrame с данными по скважине
    :param report_type: string c типом report: init_cs_data, init_chess_data, adapt_input, adapt_report, restore_input,
            overall_result
    :return:
    """
    if report_type == 'second_edit_data':
        qliq = {global_names.q_liq_m3day: [global_names.q_liq_m3day]}
        gor = {global_names.gor_m3m3: [global_names.gor_m3m3]}
        wc = {global_names.watercut_perc: [global_names.watercut_perc]}
        pressure_intake = {global_names.p_intake_atm: [global_names.p_intake_atm]}
        pressure_wh = {global_names.p_buf_atm: [global_names.p_buf_atm]}
        temp_intake = {global_names.t_intake_c: [global_names.t_intake_c]}
        frequencies = {global_names.freq_hz: [global_names.freq_hz]}
        load = {global_names.motor_load_perc: [global_names.motor_load_perc]}
        power = {global_names.active_power_kwt: [global_names.active_power_kwt]}
        voltage = {global_names.u_motor_v: [global_names.u_motor_v]}
        cos = {global_names.cos_phi_d: [global_names.cos_phi_d]}
        current = {global_names.i_a_motor_a: [global_names.i_a_motor_a]}
        all_banches = [qliq, gor, wc, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load, power, voltage, cos, current]
        return all_banches
    if report_type == 'init_cs_data':
        qliq = {'Объемный дебит жидкости': ['Объемный дебит жидкости']}
        gor = {'ГФ, м3/м3': ['ГФ, м3/м3']}
        wc = {'Процент обводненности': ['Процент обводненности']}
        pressure_intake = {'Давление на приеме': ['Давление на приеме насоса (пласт. жидкость)']}
        pressure_wh = {'Линейное давление': ['Линейное давление']}
        temp_intake = {'Температура на приеме насоса': ['Температура на приеме насоса (пласт. жидкость)']}
        frequencies = {'Выходная частота ПЧ':
                       ['Выходная частота ПЧ']}
        load = {'Загрузка двигателя': ['Загрузка двигателя']}
        power = {'Активная мощность': ['Активная мощность']}
        voltage = {'Напряжение на выходе ТМПН':['Напряжение на выходе ТМПН']}
        cos = {'Коэффициент мощности': ['Коэффициент мощности']}
        time = {'Время замеров': ['Время замера фактическое', 'Время замера плановое']}
        current = {'Ток фазы А': ['Ток фазы А']}
        all_banches = [qliq, gor, wc, time, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load,power, voltage,cos,current]
        return all_banches
    if report_type == 'init_chess_data':
        qliq = {'Qж ТМ': ['Qж ТМ']}
        wc = {'Обв ТМ': ['Обв ТМ']}
        pressure_intake = {'Рэцн ТМ': ['Рэцн ТМ']}
        pressure_wh = {'Рлин ТМ': ['Рлин ТМ']}
        pressure_buf = {'Р буферное': ['Рбуф ТМ', 'Рбуф']}
        temp_intake = {'Темп. ж. ТМ ': ['Темп. ж. ТМ']}
        frequencies = {'F вращ ТМ': ['F вращ ТМ']}
        load = {'Загр. ПЭД (ТМ)': ['Загр. ПЭД (ТМ)']}
        cos = {'Cos ф (ТМ)': ['Cos ф (ТМ)']}
        choke = {'Dшт': ['Dшт']}
        current = {'I A ТМ': ['I A ТМ']}
        all_banches = [qliq, wc, pressure_intake, pressure_wh, pressure_buf,
                       temp_intake, frequencies, load, cos, choke, current]
        return all_banches
    if report_type == 'adapt_input':
        well_data['Линейное давление (СУ)'] = well_data['Линейное давление (СУ)'] * 10
        qliq = {'Объемный дебит жидкости (СУ)': ['Объемный дебит жидкости (СУ)']}
        pressure_intake = {'Давление на приеме': ['Давление на приеме насоса (пласт. жидкость) (СУ)']}
        pressure_wh = {'Рлин ТМ (Ш)': ['Рлин ТМ (Ш)', 'Линейное давление (СУ)']}
        pressure_bf = {'Рбуф (Ш)': ['Рбуф (Ш)']}
        temp_intake = {'Температура на приеме насоса (пласт. жидкость) (СУ)': [
            'Температура на приеме насоса (пласт. жидкость) (СУ)']}
        frequencies = {'Частота, Гц':
                           ['F вращ ТМ (Ш)', 'Выходная частота ПЧ (СУ)']}
        choke = {'Размер штуцера, мм': ['Dшт (Ш)']}
        power = {'Активная мощность (СУ)': ['Активная мощность (СУ)']}
        voltage = {'Напряжение на выходе ТМПН (СУ)': ['Напряжение на выходе ТМПН (СУ)']}
        cos = {'Коэффициент мощности (СУ)': ['Коэффициент мощности (СУ)']}
        gor = {'ГФ (СУ)': ['ГФ (СУ)']}
        wc = {'Процент обводненности (СУ)': ['Процент обводненности (СУ)']}
        all_banches = [qliq, gor, wc, pressure_intake, pressure_wh, pressure_bf,
                       temp_intake, frequencies, choke, power, voltage, cos]
        return all_banches
    if report_type == 'adapt_report':
        qliq = {'Объемный дебит жидкости': ['Объемный дебит жидкости (СУ)', 'Q ж, м3/сут (Модель)']}
        calibrs = {'Калибровки по напору и мощности':
                       ['К. калибровки по напору - множитель (Модель)',
                        'К. калибровки по мощности - множитель (Модель)']}
        metrics = {'Метрики расчета':
                       ['Значение функции ошибки (Модель)']}
        gor = {'ГФ (Модель)': ['ГФ (Модель)']}
        wc = {'Обводненность, %': ['Обв, % (Модель)']}
        pressure_intake = {
            'Давление на приеме': ['Давление на приеме насоса (пласт. жидкость) (СУ)', 'P прием ЭЦН, атм (Модель)']}
        pressure_wh = {'Рлин': ['Рлин ТМ (Ш)', 'P лин., атм (Модель)']}
        pressure_bf = {'Рбуф': ['Рбуф (Ш)', 'P буф., атм (Модель)']}
        temp_intake = {'Температура на приеме насоса': ['Температура на приеме насоса (пласт. жидкость) (СУ)',
                                                        'T прием ЭЦН, C (Модель)']}
        frequencies = {'Частота, Гц':
                           ['F вращ ТМ (Ш)', 'Выходная частота ПЧ (СУ)', 'F тока, ГЦ (Модель)']}
        choke = {'Размер штуцера, мм': ['Dшт (Ш)']}
        power = {'Активная мощность (СУ)': ['Активная мощность (СУ)']}
        true_power = {'Мощность, передаваемая СУ (Модель)': ['Мощность, передаваемая СУ (Модель)']}
        voltage = {'Напряжение на выходе ТМПН (СУ)': ['Напряжение на выходе ТМПН (СУ)']}
        cos = {'Коэффициент мощности (СУ)': ['Коэффициент мощности (СУ)']}

        efficiency = {'КПД ЭЦН, д.ед.':
                          ['КПД ЭЦН, д.ед. (Модель)']}
        dif_pressure = {'Перепад давления в ЭЦН, атм':
                            ['Перепад давления в ЭЦН, атм (Модель)']}
        all_banches = [qliq, calibrs, metrics, gor, wc, pressure_intake, pressure_wh, pressure_bf,
                       temp_intake, frequencies, choke, power, true_power, voltage, cos, efficiency, dif_pressure]
        return all_banches
    if report_type == 'restore_input':
        qliq = {'Объемный дебит жидкости': ['Объемный дебит жидкости (СУ)']}
        calibrs = {'Калибровки по напору и мощности':
                       ['К. калибровки по напору - множитель (Модель) (Подготовленные)',
                        'К. калибровки по мощности - множитель (Модель) (Подготовленные)',
                        'К. калибровки по напору - множитель (Модель) (При адаптации)',
                        'К. калибровки по мощности - множитель (Модель) (При адаптации)'
                        ]}
        gor = {'ГФ (СУ)': ['ГФ (СУ)']}
        wc = {'Процент обводненности (СУ)': ['Процент обводненности (СУ)']}
        pressure_intake = {'Давление на приеме': ['Давление на приеме насоса (пласт. жидкость) (СУ)']}
        pressure_wh = {'Рлин': ['Рлин ТМ (Ш)']}
        pressure_bf = {'Рбуф': ['Рбуф (Ш)']}
        temp_intake = {'Температура на приеме насоса': ['Температура на приеме насоса (пласт. жидкость) (СУ)']}
        frequencies = {'Частота, Гц':
                           ['F вращ ТМ (Ш)', 'Выходная частота ПЧ (СУ)']}
        choke = {'Размер штуцера, мм': ['Dшт (Ш)']}
        power = {'Активная мощность (СУ)': ['Активная мощность (СУ)']}
        voltage = {'Напряжение на выходе ТМПН (СУ)': ['Напряжение на выходе ТМПН (СУ)']}
        cos = {'Коэффициент мощности (СУ)': ['Коэффициент мощности (СУ)']}

        all_banches = [qliq, calibrs, gor, wc, pressure_intake, pressure_wh, pressure_bf,
                       temp_intake, frequencies, choke, power, voltage, cos]
        return all_banches

    if report_type == 'overall_result':
        liquid_rates = {'Дебиты':
                            ['Qж ТМ (Ш) (ADAPT)', 'Объемный дебит жидкости (СУ) (ADAPT)',
                             'Q ж, м3/сут (Модель) (ADAPT)',
                             'Объемный дебит жидкости (СУ) (ADAPT) (INTERP)', 'Q ж, м3/сут (Модель) (RESTORE)']}
        essential_mertics = {'Сравнение методов расчета':
                                 ['Относительная ошибка расчетов (Q ж), %',
                                  'Относительная ошибка расчетов (Q ж) (INTERP), %']}

        calibrs = {'Калибровки по напору и мощности':
                       ['К. калибровки по напору - множитель (Модель) (ADAPT)',
                        'К. калибровки по мощности - множитель (Модель) (ADAPT)',
                        'К. калибровки по напору - множитель (Модель) (RESTORE)',
                        'К. калибровки по мощности - множитель (Модель) (RESTORE)']}

        gor_wc = {'ГФ, м3/м3 и Обводненность, %':
                      ['ГФ (Модель) (ADAPT)', 'Обв, % (Модель) (ADAPT)',
                       'ГФ (Модель) (RESTORE)', 'Обв, % (Модель) (RESTORE)']}

        temperatures = {'Температура, С':
                            ['T устья, С (Модель) (ADAPT)', 'T устья, С (Модель) (RESTORE)',
                             'T прием ЭЦН, C (Модель) (ADAPT)', 'T прием ЭЦН, C (Модель) (RESTORE)']}

        pressures_up = {'Давления (Устьевое и буферное), атм':
                            ['Рбуф (Ш) (ADAPT)', 'P буф., атм (Модель) (ADAPT)', 'P буф., атм (Модель) (RESTORE)',
                             'Рлин ТМ (Ш) (ADAPT)', 'P лин., атм (Модель) (ADAPT)', 'P лин., атм (Модель) (RESTORE)']}

        pressures_down = {'Давления (На приеме), атм':
                              ['P прием ЭЦН, атм (Модель) (ADAPT)', 'P прием ЭЦН, атм (Модель) (RESTORE)']}

        frequencies = {'Частота, Гц':
                           ['F вращ ТМ (Ш) (ADAPT)', 'Выходная частота ПЧ (СУ) (ADAPT)',
                            'F тока, ГЦ (Модель) (ADAPT)', 'F тока, ГЦ (Модель) (RESTORE)']}

        beam_sizes = {'Размер штуцера, мм':
                          ['Dшт (Ш) (ADAPT)', 'Dшт (Ш) (RESTORE)']}

        powers = {'Активная мощность, Вт':
                      ['Активная мощность (СУ) (ADAPT)', 'Мощность, передаваемая СУ (Модель) (ADAPT)',
                       'Активная мощность (СУ) (ADAPT) (INTERP)', 'Мощность, передаваемая СУ (Модель) (RESTORE)']}

        currents = {'Токи, А':
                        ['Ток фазы А (СУ) (ADAPT)', 'I, А (Модель) (ADAPT)', 'I, А (Модель) (RESTORE)']}

        loads = {'Загрузка, д.ед.':
                     ['Загрузка двигателя (Модель) (ADAPT)', 'Загрузка двигателя (Модель) (RESTORE)']}

        efficiency = {'КПД ЭЦН, д.ед.':
                          ['КПД ЭЦН, д.ед. (Модель) (ADAPT)', 'КПД ЭЦН, д.ед. (Модель) (RESTORE)']}

        metrics = {'Метрики расчета':
                       ['Значение функции ошибки (Модель) (ADAPT)',
                        'Значение функции ошибки (Модель) (RESTORE)',
                        'Относительная ошибка расчетов (N акт), %',
                        'Относительная ошибка расчетов (N акт) (INTERP), %']}

        dif_pressure = {'Перепад давления в ЭЦН, атм':
                            ['Перепад давления в ЭЦН, атм (Модель) (ADAPT)',
                             'Перепад давления в ЭЦН, атм (Модель) (RESTORE)']}
        all_banches = [liquid_rates, essential_mertics, calibrs, metrics, gor_wc, temperatures, pressures_up,
                       pressures_down, frequencies, beam_sizes, powers, currents, loads, efficiency, dif_pressure]
        return all_banches