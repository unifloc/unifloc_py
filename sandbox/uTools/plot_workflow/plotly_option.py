
def create_banches_for_report(well_data, report_type):
    if report_type == 0:
        well_data['ГФ, м3/м3'] = well_data['Объемный дебит газа'] / well_data['Объемный дебит нефти']
        qliq = {'Объемный дебит жидкости':['Объемный дебит жидкости']}
        gor = {'ГФ, м3/м3':['ГФ, м3/м3']}
        wc = {'Процент обводненности':['Процент обводненности']}
        pressure_intake = {'Давление на приеме': ['Давление на приеме насоса (пласт. жидкость)']}
        pressure_wh = {'Линейное давление': ['Линейное давление']}
        temp_intake = {'Температура на приеме насоса': ['Температура на приеме насоса (пласт. жидкость)']}
        frequencies = {'Выходная частота ПЧ':
                       ['Выходная частота ПЧ']}
        load = {'Загрузка двигателя': ['Загрузка двигателя']}
        power = {'Активная мощность':['Активная мощность']}
        voltage = {'Напряжение на выходе ТМПН':['Напряжение на выходе ТМПН']}
        cos = {'Коэффициент мощности':['Коэффициент мощности']}
        time = {'Время замеров':['Время замера фактическое','Время замера плановое']}
        current =  {'Ток фазы А':['Ток фазы А']}
        all_banches = [qliq,gor, wc, time, pressure_intake, pressure_wh,
                       temp_intake, frequencies, load,power, voltage,cos,current]
        return all_banches
    if report_type == 1:
        qliq = {'Qж ТМ': ['Qж ТМ']}
        wc = {'Обв ТМ': ['Обв ТМ']}
        pressure_intake = {'Рэцн ТМ': ['Рэцн ТМ']}
        pressure_wh = {'Рлин ТМ': ['Рлин ТМ']}
        pressure_buf = {'Р буферное': ['Рбуф ТМ', 'Рбуф']}
        temp_intake = {'Темп. ж. ТМ ': ['Темп. ж. ТМ']}
        frequencies = {'F вращ ТМ':
                           ['F вращ ТМ']}
        load = {'Загр. ПЭД (ТМ)': ['Загр. ПЭД (ТМ)']}
        cos = {'Cos ф (ТМ)': ['Cos ф (ТМ)']}
        choke = {'Dшт': ['Dшт']}
        current = {'I A ТМ': ['I A ТМ']}
        all_banches = [qliq, wc, pressure_intake, pressure_wh, pressure_buf,
                       temp_intake, frequencies, load, cos, choke, current]
        return all_banches