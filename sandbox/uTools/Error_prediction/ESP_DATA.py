class AllEspData:  # класс, в котором хранятся данные
    def __init__(self, unifloc_vba, tr_data):
        """
        класс для хранение и доступа ко всем данным скважины - входным, выходным
        :param unifloc_vba: текущая надстройка UniflocVBA API
        :param tr_data: данные техрежима
        """
        self.ESP_rate_nom = tr_data.esp_nom_rate_m3day
        self.esp_id = unifloc_vba.calc_ESP_id_by_rate(self.ESP_rate_nom)
        self.ESP_head_nom = tr_data.esp_nom_head_m
        self.dcas_mm = tr_data.d_cas_mm
        self.h_pump_m = tr_data.h_pump_m
        self.d_tube_mm = tr_data.d_tube_mm
        self.p_cas_data_atm = -1  # нет расчета затрубного пространства - он долгий и немножко бесполезный

        self.eff_motor_d = 0.89
        self.i_motor_nom_a = tr_data.i_motor_nom_a
        self.power_motor_nom_kwt = tr_data.power_motor_nom_kwt
        self.h_tube_m = self.h_pump_m  # ТР
        self.h_perf_m = self.h_pump_m + 1  # ТР
        self.udl_m = tr_data.udl_m  # ТР

        self.c_calibr_rate_d = 1

        self.ksep_d = 0.7  # ТР
        self.KsepGS_fr = 0.7  # ТР
        self.hydr_corr = 1  # 0 - BB, 1 - Ansari
        self.gamma_oil = 0.945
        self.gamma_gas = 0.9
        self.gamma_wat = 1.011
        self.rsb_m3m3 = 29.25
        self.tres_c = 16
        self.pb_atm = 40
        self.bob_m3m3 = 1.045
        self.muob_cp = 100
        self.rp_m3m3 = 30

        self.psep_atm = None
        self.tsep_c = None

        self.d_choke_mm = None
        self.ESP_freq = None
        self.p_intake_data_atm = None
        self.p_wellhead_data_atm = None
        self.p_buf_data_atm = None
        self.p_wf_atm = None
        self.cos_phi_data_d = None
        self.u_motor_data_v = None
        self.active_power_cs_data_kwt = None
        self.qliq_m3day = 100  # initial guess
        self.watercut_perc = None
        self.p_buf_data_atm = None
        self.c_calibr_head_d = 1  # initial guess
        self.c_calibr_power_d = 1  # initial guess

        self.result = None
        self.error_in_step = None
        self.p_buf_data_max_atm = None
        self.active_power_cs_data_max_kwt = None
        self.p_wellhead_data_max_atm = None
        self.qliq_max_m3day = None
