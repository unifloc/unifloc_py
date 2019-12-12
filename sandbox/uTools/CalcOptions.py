class CalcOptions:  # TODO сделать класс-структуру со всем (настройки расчета отдельно здесь, алгоритм отдельно)
    def __init__(self, well='570',  # менять тут для адаптации/восстановления
                 dir_name_with_input_data='restore_input',  # менять тут для адаптации/восстановления
                 multiprocessing=True,
                 addin_name="UniflocVBA_7.xlam",
                 tr_name="Техрежим, , февраль 2019.xls",
                 number_of_thread=1,
                 amount_of_threads=8,
                 debug_mode=False,
                 amount_iters_before_restart=100,
                 sleep_time_sec=25,
                 vfm_calc_option=True,  # менять тут для адаптации/восстановления
                 restore_q_liq_only=True,  # менять тут для адаптации/восстановления
                 calc_option=True,
                 use_pwh_in_loss=False,
                 hydr_part_weight_in_error_coeff=0.5):  # TODO добавлять насосы в UniflocVBA
        """
        класс для сбора всех настроек, необходимых для расчета
        :param well: имя скважины
        :param dir_name_with_input_data: название директории с входными данными (adaptation_input или restore_input)
        :param multiprocessing: флаг для расчета в многопотоке(предварительно нужно размножить unifloc_vba.xlam
        :param addin_name: название надстройки
        :param number_of_thread: порядковый номер этого потока
        :param amount_of_threads: общее число потоков
        :param calc_option: флаг расчета, если True - начала итераций по строкам в df
        :param debug_mode: флаг отладки, если True - онлайн вывод значений функции ошибки и других важных параметров
        :param vfm_calc_option: флаг метода расчета, если True - восстановление, если - False - адаптация
        :param restore_q_liq_only: флаг метода восстановления, если True - только дебита жидкости
        :param amount_iters_before_restart: количество итераций перед перезапуском экселя
        :param sleep_time_sec: время отдыха после закрытия экселя
        :param hydr_part_weight_in_error_coeff: гиперпараметр на гидравлическую часть в функции ошибки
        """
        self.use_pwh_in_loss = use_pwh_in_loss
        self.calc_option = calc_option
        self.vfm_calc_option = vfm_calc_option
        self.restore_q_liq_only = restore_q_liq_only
        self.well = well
        self.dir_name_with_input_data = dir_name_with_input_data
        self.multiprocessing = multiprocessing
        self.addin_name = addin_name
        self.number_of_thread = number_of_thread
        self.amount_of_threads = amount_of_threads
        self.tr_name = tr_name
        self.debug_mode = debug_mode
        self.amount_iters_before_restart = amount_iters_before_restart
        self.sleep_time_sec = sleep_time_sec
        self.hydr_part_weight_in_error_coeff = hydr_part_weight_in_error_coeff
