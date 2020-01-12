import time


import sys
sys.path.append('../../../')

from multiprocessing import Pool

from proc_p import processor as proc

amount_of_threads = 12
def run_calculation(thread_option_list):
    if __name__ == '__main__':
        with Pool(amount_of_threads) as p:
            p.map(proc.calc,
                  thread_option_list)

tr_name = "Техрежим, , февраль 2019.xls"
amount_of_threads = 12
dir_name_with_input_data = 'adapt_input_'

start_time = time.time()
for well_name in ['1628', '570']:
    start_time_in_loop = time.time()
    thread_option_list =proc.create_thread_list(well_name, dir_name_with_input_data, tr_name,
                           amount_of_threads)
    run_calculation(thread_option_list)
    end_time_in_loop = time.time()
    print('Затрачено времени на скважину ' + well_name + '  ' + str(end_time_in_loop - start_time_in_loop))
end_time = time.time()
print('Затрачено времени всего ' + well_name + '  ' + str(end_time_in_loop - start_time_in_loop))