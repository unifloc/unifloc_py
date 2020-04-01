import uniflocpy.uTools.uconst
import uniflocpy.uTools.data_workflow


class IPR_Vogel():

    def __init__(self):
        self.p_res_bar = None
        self.watercut_percent = None
        self.q_liq_m3day = None
        self.p_bubble_bar = None
        self.p_bhp_bar = None
        self.pi_m3daybar = None

    def calc_q_liq_m3day(self, p_bhp_bar, p_res_bar,
                         p_bubble_bar, pi_m3daybar, watercut_percent):
        self.p_bhp_bar = p_bhp_bar
        self.p_res_bar = p_res_bar
        self.p_bubble_bar = p_bubble_bar
        self.pi_m3daybar = pi_m3daybar
        self.watercut_percent = watercut_percent

        if self.watercut_percent == 100 or self.p_bhp_bar > self.p_bubble_bar:
            self.q_liq_m3day = self.pi_m3daybar * (self.p_res_bar - self.p_bhp_bar)
            return self.q_liq_m3day
        else:
            self.q_bubble_m3day = self.pi_m3daybar * (self.p_res_bar - self.p_bubble_bar)
            self.q_max_m3day = self.q_bubble_m3day + self.pi_m3daybar * self.p_bubble_bar / 1.8
            self.q_liq_m3day = (1 - 0.2 * self.p_bhp_bar / self.p_res_bar -
                                0.8 * (self.p_bhp_bar / self.p_res_bar) ** 2) * self.q_max_m3day
            return self.q_liq_m3day

test = IPR_Vogel()

q_liq_m3day = test.calc_q_liq_m3day(30,250,100,0.1,50)
print(q_liq_m3day)



