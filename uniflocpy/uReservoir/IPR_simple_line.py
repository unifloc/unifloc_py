import uniflocpy.uTools.uconst as uc


class IPRSimpleLine():
    def __init__(self):
        self.p_reservoir_bar = None
        self.pi_m3daybar = None
        self.p_bottomhole_test_bar = None
        self.q_liq_test_m3day = None
        self.dp_test_bar = None

        self.q_liq_m3day = None
        self.dp_bar = None
        self.p_bottomhole_bar = None

    def calc_pi_m3daybar(self, q_liq_test_m3day, p_bottomhole_test_bar, p_reservouir_bar):
        self.q_liq_test_m3day = q_liq_test_m3day
        self.p_bottomhole_test_bar = p_bottomhole_test_bar
        self.p_reservoir_bar = p_reservouir_bar
        self.dp_test_bar = p_reservouir_bar - p_bottomhole_test_bar
        self.pi_m3daybar = self.q_liq_test_m3day / self.dp_test_bar
        return self.pi_m3daybar

    def calc_p_bottomhole_bar(self, q_liq_m3day):
        self.q_liq_m3day = q_liq_m3day
        self.dp_bar = self.q_liq_m3day / self.pi_m3daybar
        self.p_bottomhole_bar = self.p_reservoir_bar - self.dp_bar
        return self.p_bottomhole_bar

