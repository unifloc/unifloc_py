from Error_prediction.CalcOptions import CalcOptions
from multiprocessing import Pool
from Error_prediction.processor import calc

def create_thread_list(well, dir_name_with_input_data, tr_name,
                       amount_of_threads):
    thread_list = []
    for number_of_thread in range(amount_of_threads):
        addin_name = 'UniflocVBA_7_%s.xlam' % str(number_of_thread)
        this_thread = CalcOptions(addin_name=addin_name, number_of_thread=number_of_thread,
                                  amount_of_threads=amount_of_threads, well=well)
        thread_list.append(this_thread)
    return thread_list


def run_calculation(func, thread_option_list):
    if __name__ == '__main__':
        with Pool(len(thread_option_list)) as p:
            p.map(func, thread_option_list)


thread_list = create_thread_list(well='252', dir_name_with_input_data='restore_input_',
                                 tr_name="Техрежим, , февраль 2019.xls", amount_of_threads=8)
run_calculation(calc, thread_list)
