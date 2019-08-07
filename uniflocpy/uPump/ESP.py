"""
Модуль расчета ЭЦН, основанные на модели ЭЦН в UniflocVBA

Кобзарь О.С. Хабибуллин Р.А. 27.07.2019
"""
import uniflocpy.uTools.uconst as uc
import uniflocpy.uPVT.PVT_fluids as PVT_fluids
import numpy as np
import matplotlib.pyplot as plt

const_convert_m3day_gpm = 0.183452813362193
class ESP_polynom():
    def __init__(self):
        self.coef0 = None
        self.coef1 = None
        self.coef2 = None
        self.coef3 = None
        self.coef4 = None
        self.coef5 = None
        self.arg = None
        self.result = None
    def calc(self, arg):
        self.result = self.coef0 + \
                        self.coef1 * arg + \
                        self.coef2 * arg ** 2 + \
                        self.coef3 * arg ** 3 + \
                        self.coef4 * arg ** 4 + \
                        self.coef5 * arg ** 5
        return self.result

class ESP_structure():
    def __init__(self, ID_ESP=737,
                 stage_number=324,
                 power_ESP_nom_kwt=60,
                 freq_hz=50,
                 freq_nom_hz=50,
                 q_max_ESP_nom_m3day=300,
                 q_ESP_nom_m3day=250):
        self.ID_ESP = ID_ESP
        self.stage_number = stage_number
        self.power_ESP_nom_kwt = power_ESP_nom_kwt
        self.freq_hz = freq_hz
        self.freq_nom_hz = freq_nom_hz
        self.q_max_ESP_nom_m3day = q_max_ESP_nom_m3day
        self.q_ESP_nom_m3day = q_ESP_nom_m3day

class ESP():

    def __init__(self, ESP_structure, MultiPhaseFlow = PVT_fluids.FluidFlow(), fluid = PVT_fluids.FluidStanding()):
        self.stage_number = ESP_structure.stage_number
        self.power_ESP_nom_kwt = ESP_structure.power_ESP_nom_kwt
        self.freq_hz = ESP_structure.freq_hz
        self.freq_nom_hz = ESP_structure.freq_nom_hz
        self.q_max_ESP_nom_m3day = ESP_structure.q_max_ESP_nom_m3day
        self.q_ESP_nom_m3day = ESP_structure.q_ESP_nom_m3day

        self.polynom_ESP = None

        self.stage_height_m = 0.05
        self.ESP_lenth = None

        self.k_sep_total_d = 0.7

        self.fluid_flow = MultiPhaseFlow
        self.fluid = fluid

        self.option_correct_viscosity = True

        self.k_rate_degradation_d = 0
        self.k_pressure_degradation_d = 0
        self.stage_number_in_calc = 1

        self.p_intake_bar = None
        self.t_intake_bar = None
        self.power_fluid_stage_kwt = None
        self.power_fluid_total_kwt = None
        self.power_ESP_stage_kwt = None
        self.power_ESP_total_kwt = None
        self.dt_stage_c = None
        self.dt_total_c = None
        self.dp_stage_bar = None
        self.dp_total_bar = None
        self.p_bar = None
        self.t_c = None

        self.gas_fraction_d = None
        self.q_mix_m3day = None
        self.q_mix_degr_m3day = None
        self.stage_head_m = None
        self.ESP_head_total_m = None
        self.mu_mix_cP = None
        self.rho_mix_kgm3 = None
        self.mu_mix_cSt = None
        self.q_max_ESP_m3day = None

        self.check = None

    def correct_viscosity_by_petroleum_institute(self, q_m3day, mu_cSt):
        if mu_cSt < 5:
            return -1
        else:
            self.q_wat_BEP_100gpm = self.q_ESP_nom_m3day * self.freq_hz / self.freq_nom_hz * const_convert_m3day_gpm
            self.q_ESP_nom_freq_m3day = self.q_ESP_nom_m3day * self.freq_hz / self.freq_nom_hz
            self.head_wat_BEP_ft = uc.m2ft() # TODO
            self.gamma = -7.5946 + 6.6504 * np.log(self.head_wat_BEP_ft) + 12.8429 * np.log(self.q_wat_BEP_100gpm)
            self.q_star = np.exp((39.5276 + 26.5606 * np.log(mu_cSt) - self.gamma) / 51.6565)
            self.k_q_corr_visc = 1 - 4.0327 * 10 ** (-3) * self.q_star - 1.724 * 10 ** (-4) * self.q_star ** 2
            if self.k_q_corr_visc < 0:
                self.k_q_corr_visc = 0
            self.k_efficency_corr_visc = 1 - 3.3075 * 10 ** (-2) * self.q_star - 2.8875 * 10 ** (-4) * self.q_star ** 2

            Q0 = 0
            Q1_0 = self.q_ESP_nom_freq_m3day * self.k_q_corr_visc
            H1_0 = 1 - 7.00763 * 10 ** (-3) * self.q_star - 1.41 * 10 ^ (
                -5) * self.q_star ^ 2
            Q0_8 = Q1_0 * 0.8
            H0_8 = 1 - 4.4726 * 10 ** (-3) * self.q_star - 4.18 * 10 ** (-5) * self.q_star ** 2
            Q0_6 = Q1_0 * 0.6
            H0_6 = 1 - 3.68 * 10 ** (-3) * self.q_star - 4.36 * 10 ** (-5) * self.q_star ** 2
            Q1_2 = Q1_0 * 1.2
            H1_2 = 1 - 9.01 * 10 ** (-3) * self.q_star + 1.31 * 10 ** (-5) * self.q_star ** 2
            q_max = self.q_max_ESP_nom_m3day * self.k_q_corr_visc
            h_max = H1_2
            if q_max < Q1_2:
                print("Ошибка, что-то не так с характеристикой насоса")

            return





    def get_ESP_head_m(self, q_mix_degr_m3day, stage_number_in_calc, mu_mix_cP):
        if q_mix_degr_m3day < 0:
            print("Ошибка. Расчет характеристики насоса с отрицательным дебитом. Напор установлен нулю.")
            self.stage_head_m = 0

        self.mu_mix_cSt = mu_mix_cP / (self.rho_mix_kgm3 / 1000)
        self.q_max_ESP_m3day = self.freq_hz / self.freq_nom_hz * self.q_max_ESP_nom_m3day
        if q_mix_degr_m3day > self.q_max_ESP_m3day:
            print("Ошибка. Текущий дебит превышает максимальный для данной частоты. Напор установлен нулю.")
            self.stage_head_m = 0
        if self.option_correct_viscosity:
            self.check = self.correct_viscosity_by_petroleum_institute(q_mix_degr_m3day, self.mu_mix_cSt)
        b = self.freq_hz / self.freq_nom_hz
        self.ESP_head_calculated_m = b ** (-2) * stage_number_in_calc * self.polynom_ESP.calc(b * q_mix_degr_m3day)
        return self.ESP_head_calculated_m





    def calc(self, p_bar, t_c):
        self.p_intake_bar = p_bar
        self.t_intake_bar = t_c

        self.p_bar = p_bar
        self.t_c = t_c

        self.power_fluid_stage_kwt = 0
        self.power_fluid_total_kwt = 0
        self.power_ESP_stage_kwt = 0
        self.power_ESP_total_kwt = 0
        self.dt_stage_c = 0
        self.dt_total_c = 0
        self.dp_stage_bar = 0
        self.dp_total_bar = 0
        self.stage_head_m = 0
        self.ESP_head_total_m = 0


        for i in range(self.stage_number):
            self.fluid.calc(self.p_bar, self.t_c)
            self.fluid_flow.calc(self.p_bar, self.t_c)

            self.gas_fraction_d = (self.fluid_flow.qgas_m3day * (1 - self.k_sep_total_d) /
                                   self.fluid_flow.qliq_m3day + self.fluid_flow.qgas_m3day * (1 - self.k_sep_total_d))

            self.q_mix_m3day = self.fluid_flow.qliq_m3day + self.fluid_flow.qgas_m3day * (1 - self.k_sep_total_d)
            self.q_mix_degr_m3day = self.q_mix_m3day * (1 + self.k_rate_degradation_d)
            self.mu_mix_cP = self.fluid_flow.mu_liq_cP * (1 - self.gas_fraction_d) + \
                             self.fluid_flow.fl.mu_gas_cP * self.gas_fraction_d
            self.rho_mix_kgm3 = self.fluid_flow.rho_liq_kgm3 * (1 - self.gas_fraction_d) + \
                                self.fluid_flow.fl.rho_gas_kgm3 * self.gas_fraction_d
            self.stage_head_m = self.get_ESP_head_m(self.q_mix_degr_m3day, self.stage_number_in_calc, self.mu_mix_cP)



ESP_polynom_obj = ESP_polynom()

ESP_polynom_obj.coef0 = 6.73238871864004
ESP_polynom_obj.coef1 = -1.15209366895305E-02
ESP_polynom_obj.coef2 = 4.45392114072489E-04
ESP_polynom_obj.coef3 = -4.97923165135159E-06
ESP_polynom_obj.coef4 = 1.4546401075876E-08
ESP_polynom_obj.coef5 = -1.19688044019184E-11
ESP_data = ESP_structure()

ESP_obj = ESP(ESP_data)
ESP_obj.polynom_ESP = ESP_polynom_obj
p_bar = 30
t_c = 30

ESP_obj.fluid.calc(p_bar, t_c)
ESP_obj.mu_mix_cP = ESP_obj.fluid.mu_oil_cP
ESP_obj.rho_mix_kgm3 = ESP_obj.fluid.rho_oil_kgm3
q_mix_degr_m3day = 100
stage_number_in_calc = 324
mu_mix_cP = 1
head_m = ESP_obj.get_ESP_head_m(q_mix_degr_m3day, stage_number_in_calc, mu_mix_cP)
print(head_m)
head_m_list = []
q_m3day_list = []

for q_m3day in range(1, 230):
    head_m = ESP_obj.get_ESP_head_m(q_m3day, stage_number_in_calc, mu_mix_cP)
    head_m_list.append(head_m)
    q_m3day_list.append(q_m3day)
    print(head_m)


plt.plot(q_m3day_list,head_m_list)
plt.show()