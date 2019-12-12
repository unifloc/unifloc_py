from processor import calc
from CalcOptions import CalcOptions
from multiprocessing import Pool

threads_num = 8

first_thread = CalcOptions(addin_name="UniflocVBA_7.xlam", number_of_thread=1, amount_of_threads=threads_num)
second_thread = CalcOptions(addin_name="UniflocVBA_7_1.xlam", number_of_thread=2, amount_of_threads=threads_num)
third_thread = CalcOptions(addin_name="UniflocVBA_7_2.xlam", number_of_thread=3, amount_of_threads=threads_num)
fourth_thread = CalcOptions(addin_name="UniflocVBA_7_3.xlam", number_of_thread=4, amount_of_threads=threads_num)
fifth_thread = CalcOptions(addin_name="UniflocVBA_7_4.xlam", number_of_thread=5, amount_of_threads=threads_num)
sixth_thread = CalcOptions(addin_name="UniflocVBA_7_5.xlam", number_of_thread=6, amount_of_threads=threads_num)
seventh_thread = CalcOptions(addin_name="UniflocVBA_7_6.xlam", number_of_thread=7, amount_of_threads=threads_num)
eighth_thread = CalcOptions(addin_name="UniflocVBA_7_7.xlam", number_of_thread=8, amount_of_threads=threads_num)


# TODO добавить расчет для одного ядра


def run_calculation(threads):
    """
    Функция запускает многоточный расчет при прямом запуске из модуля, при импорте в app.ipynb не работает
    :param threads: спиской настроек для каждого потока
    :return:
    """
    if __name__ == '__main__':
        with Pool(len(threads)) as p:
            p.map(calc,
                  threads)


th = [first_thread, second_thread, third_thread, fourth_thread, fifth_thread,
      sixth_thread, seventh_thread, eighth_thread]
run_calculation(th)
