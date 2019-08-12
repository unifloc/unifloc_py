"""
Модуль расчета ЭЦН, основанные на модели ЭЦН в UniflocVBA

Кобзарь О.С. Хабибуллин Р.А. 27.07.2019
"""
import uniflocpy.uTools.uconst as uc
import uniflocpy.uPVT.PVT_fluids as PVT_fluids
import numpy as np
import uniflocpy.uTools.data_workflow as data_workflow
import pandas as pd
import time
import plotly.graph_objs as go
from plotly.offline import download_plotlyjs, init_notebook_mode, plot


# TODO отдельный метод/класс для проверки исходных данных

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

        self.esp_calc_data = data_workflow.Data()
        self.fluid_flow_calc_data = data_workflow.Data()
        self.fluid_calc_data = data_workflow.Data()

        self.save_calculated_data = True

        self.polynom_esp_head_wt = None
        self.polynom_esp_efficiency_d = None
        self.polynom_esp_power_wt = None

        self.stage_height_m = 0.05
        self.ESP_lenth = None

        self.k_sep_total_d = 0.9

        self.fluid_flow = MultiPhaseFlow
        self.fluid = fluid

        self.option_correct_viscosity = False

        self.k_rate_degradation_d = 0
        self.k_pressure_degradation_d = 0
        self.stage_number_in_calc = 1

        self.p_intake_bar = None
        self.t_intake_bar = None
        self.power_fluid_stage_wt = None
        self.power_fluid_total_wt = None
        self.power_ESP_stage_wt = None
        self.power_ESP_total_wt = None
        self.dt_stage_c = None
        self.dt_total_c = None
        self.dp_stage_bar = None
        self.dp_total_bar = None
        self.p_bar = None
        self.t_c = None
        self.ESP_efficiency_total_d = None
        self.ESP_efficiency_stage_d = None

        self.gas_fraction_d = None
        self.q_mix_m3day = None
        self.q_mix_degr_m3day = None
        self.stage_head_m = None
        self.ESP_head_total_m = None
        self.mu_mix_cP = None
        self.rho_mix_kgm3 = None
        self.mu_mix_cSt = None
        self.q_max_ESP_m3day = None

        self.check = -1

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
        self.ESP_head_calculated_m = b ** (-2) * stage_number_in_calc * self.polynom_esp_head_wt.calc(b * q_mix_degr_m3day)
        return self.ESP_head_calculated_m

    def get_ESP_efficiency_d(self, q_m3day, mu_cP):
        self.mu_mix_cSt = mu_cP / (self.rho_mix_kgm3 / 1000)
        b = self.freq_hz / self.freq_nom_hz
        self.ESP_efficiency_calculated_d = self.polynom_esp_efficiency_d.calc(b * q_m3day)
        return self.ESP_efficiency_calculated_d

    def get_ESP_power_Wt(self, q_m3day, stage_number_in_calc, mu_cP):
        b = self.freq_hz / self.freq_nom_hz
        self.power_ESP_calculated_Wt = 1000 * b ** (-3) * stage_number_in_calc * self.polynom_esp_power_wt.calc(b * q_m3day)
        return self.power_ESP_calculated_Wt

    def calc(self, p_bar, t_c):
        self.p_intake_bar = p_bar
        self.t_intake_bar = t_c

        self.p_bar = p_bar
        self.t_c = t_c

        self.power_fluid_stage_wt = 0
        self.power_fluid_total_wt = 0
        self.power_ESP_stage_wt = 0
        self.power_ESP_total_wt = 0
        self.dt_stage_c = 0
        self.dt_total_c = 0
        self.dp_stage_bar = 0
        self.dp_total_bar = 0
        self.stage_head_m = 0
        self.ESP_head_total_m = 0

        self.esp_calc_data.clear_data()
        self.fluid_flow_calc_data.clear_data()
        self.fluid_calc_data.clear_data()

        for i in range(self.stage_number):
            self.fluid.calc(self.p_bar, self.t_c)
            self.fluid_flow.calc(self.p_bar, self.t_c)

            self.gas_fraction_d = (self.fluid_flow.qgas_m3day * (1 - self.k_sep_total_d) /
                                   (self.fluid_flow.qliq_m3day + self.fluid_flow.qgas_m3day * (1 - self.k_sep_total_d)))

            self.q_mix_m3day = self.fluid_flow.qliq_m3day + self.fluid_flow.qgas_m3day * (1 - self.k_sep_total_d)
            self.q_mix_degr_m3day = self.q_mix_m3day * (1 + self.k_rate_degradation_d)
            self.mu_mix_cP = self.fluid_flow.mu_liq_cP * (1 - self.gas_fraction_d) + \
                             self.fluid_flow.fl.mu_gas_cP * self.gas_fraction_d
            self.rho_mix_kgm3 = self.fluid_flow.rho_liq_kgm3 * (1 - self.gas_fraction_d) + \
                                self.fluid_flow.fl.rho_gas_kgm3 * self.gas_fraction_d
            self.stage_head_m = self.get_ESP_head_m(self.q_mix_degr_m3day, self.stage_number_in_calc, self.mu_mix_cP)
            self.ESP_head_total_m += self.stage_head_m
            self.dp_stage_bar = uc.Pa2bar(self.stage_head_m * self.rho_mix_kgm3 * uc.g)
            self.dp_total_bar += self.dp_stage_bar

            self.power_fluid_stage_wt = uc.m3day2m3sec(self.q_mix_degr_m3day) * uc.bar2Pa(self.dp_stage_bar)
            self.power_fluid_total_wt += self.power_fluid_stage_wt

            self.power_ESP_stage_wt = self.get_ESP_power_Wt(self.q_mix_degr_m3day,
                                                            self.stage_number_in_calc,
                                                            self.mu_mix_cP)
            self.power_ESP_total_wt += self.power_ESP_stage_wt

            if self.power_ESP_total_wt > 0:
                self.ESP_efficiency_total_d = self.power_fluid_total_wt / self.power_ESP_total_wt
            if self.power_ESP_stage_wt > 0:
                self.ESP_efficiency_stage_d = self.power_fluid_stage_wt / self.power_ESP_stage_wt

            if self.ESP_efficiency_stage_d > 0:
                self.dt_stage_c = (uc.g * self.stage_head_m / self.fluid_flow.heatcapn_jkgc *
                                  (1 - self.ESP_efficiency_stage_d) / self.ESP_efficiency_stage_d)

            self.dt_total_c += self.dt_stage_c

            if self.save_calculated_data:
                self.esp_calc_data.save_data_from_class_to_storage(self)
                self.fluid_flow_calc_data.save_data_from_class_to_storage(self.fluid_flow)
                self.fluid_calc_data.save_data_from_class_to_storage(self.fluid)

            self.p_bar += self.dp_stage_bar
            self.t_c += self.dt_stage_c

# TODO функции для построения графиков вынести в модуль plot_workflow

def trace(data_x, data_y, namexy):
    tracep = go.Scattergl(
        x=data_x,
        y=data_y,
        name=namexy,
        mode='lines'
    )
    return tracep


def plot_func(plot_title_str, filename_str):
    layout = dict(title=plot_title_str)

    fig = dict(data=data, layout=layout)

    plot(fig, filename=filename_str)
    #plot()


def create_traces_list_by_num(data_x_values, data_y, num_y_list):
    trace_list = []
    for i in num_y_list:
        namexy = data_y.get_saved_parameter_name_by_number(i)
        this_trace = trace(data_x_values, data_y.get_saved_values_by_number(i), namexy)
        trace_list.append(this_trace)
    return trace_list


def connect_traces(traces1, trace2):
    connected_traces = []
    for i in traces1:
        connected_traces.append(i)
    for j in trace2:
        connected_traces.append(j)
    return connected_traces


ESP_polynom_head_obj = ESP_polynom()
ESP_polynom_head_obj.coef0 = 6.73238871864004
ESP_polynom_head_obj.coef1 = -1.15209366895305E-02
ESP_polynom_head_obj.coef2 = 4.45392114072489E-04
ESP_polynom_head_obj.coef3 = -4.97923165135159E-06
ESP_polynom_head_obj.coef4 = 1.4546401075876E-08
ESP_polynom_head_obj.coef5 = -1.19688044019184E-11

ESP_polynom_efficency_obj = ESP_polynom()
ESP_polynom_efficency_obj.coef0 = 4.1990495766025E-03
ESP_polynom_efficency_obj.coef1 = 5.99377724541058E-03
ESP_polynom_efficency_obj.coef2 = 1.6897242340386E-05
ESP_polynom_efficency_obj.coef3 = -2.73132096997276E-07
ESP_polynom_efficency_obj.coef4 = 9.14180916880957E-11
ESP_polynom_efficency_obj.coef5 = 1.24234461289717E-12

ESP_polynom_power_obj = ESP_polynom()
ESP_polynom_power_obj.coef0 = 0.106170031712193
ESP_polynom_power_obj.coef1 = 2.74596766297403E-04
ESP_polynom_power_obj.coef2 = 3.45749447827218E-06
ESP_polynom_power_obj.coef3 = -5.9698533809646E-08
ESP_polynom_power_obj.coef4 = 4.02396243631364E-10
ESP_polynom_power_obj.coef5 = -9.16011251716958E-13

ESP_data = ESP_structure()
ESP_obj = ESP(ESP_data)
ESP_obj.polynom_esp_head_wt = ESP_polynom_head_obj
ESP_obj.polynom_esp_efficiency_d = ESP_polynom_efficency_obj
ESP_obj.polynom_esp_power_wt = ESP_polynom_power_obj

p_bar = 30
t_c = 30
"""
ESP_obj.q_mix_m3day = 100
ESP_obj.fluid_flow.fw_on_surface_perc = 0
ESP_obj.calc(p_bar, t_c)

ESP_obj.esp_calc_data.print_all_names_of_saved_parameters()
ESP_obj.fluid_calc_data.print_all_names_of_saved_parameters()
ESP_obj.fluid_flow_calc_data.print_all_names_of_saved_parameters()

parameter_numbers_ESP_list = list(range(19, 45))
data_trace_ESP = create_traces_list_by_num(np.asarray(range(ESP_obj.stage_number)), ESP_obj.esp_calc_data, parameter_numbers_ESP_list)
data = data_trace_ESP
plot_func("Расчет насоса по ступеням - esp_calc_data", "Расчет насоса по ступеням - esp_calc_data.html")

parameter_numbers_fluid_flow_list =  list(range(6, 30))
data_trace_fluid_flow = create_traces_list_by_num(np.asarray(range(ESP_obj.stage_number)), ESP_obj.fluid_flow_calc_data, parameter_numbers_fluid_flow_list)
data = data_trace_fluid_flow
plot_func("Расчет насоса по ступеням - fluid_flow_calc_data", "Расчет насоса по ступеням - fluid_flow_calc_data.html")


parameter_numbers_fluid_list = list(range(0, 43))
data_trace_fluid = create_traces_list_by_num(np.asarray(range(ESP_obj.stage_number)), ESP_obj.fluid_calc_data, parameter_numbers_fluid_list)
data = data_trace_fluid
plot_func("Расчет насоса по ступеням - fluid_calc_data", "Расчет насоса по ступеням - fluid_calc_data.html")

"""

data_ESP_perfomance = data_workflow.Data()
data_fluid_perfomance = data_workflow.Data()
data_fluid_flow_perfomance = data_workflow.Data()

data_ESP_perfomance.clear_data()
data_fluid_perfomance.clear_data()
data_fluid_flow_perfomance.clear_data()

start = time.time()

for q_m3day in range(1, 240, 10):
    start_in_loop = time.time()

    ESP_obj.fluid.bobcal_m3m3 = 0
    ESP_obj.fluid.muobcal_cP = 0
    ESP_obj.fluid_flow.fl.bobcal_m3m3 = 0
    ESP_obj.fluid_flow.fl.muobcal_cP= 0
    ESP_obj.fluid_flow.fw_on_surface_perc = 0
    ESP_obj.save_calculated_data = False
    ESP_obj.fluid_flow.qliq_on_surface_m3day = q_m3day
    ESP_obj.calc(p_bar, t_c)
    data_ESP_perfomance.save_data_from_class_to_storage(ESP_obj)
    data_fluid_perfomance.save_data_from_class_to_storage(ESP_obj.fluid)
    data_fluid_flow_perfomance.save_data_from_class_to_storage(ESP_obj.fluid_flow)

    end_in_loop = time.time()
    print("Рассчитан ЭЦН для q_m3day=" + str(q_m3day))
    print("Время расчета для одного значения расхода = " + str(end_in_loop - start_in_loop))

end = time.time()
print("Полный расчет характеристики")
print(end - start)

numbers_ESP_perfomance_list = list(range(19, 45))
data_trace_ESP = create_traces_list_by_num(np.asarray(range(1, 230, 10)), data_ESP_perfomance,  numbers_ESP_perfomance_list)
data = data_trace_ESP
plot_func("Расчет ЭЦН на разных режимах работы - ESP", "Расчет ЭЦН на разных режимах работы - ESP.html")

numbers_fluid_perfomance_list = list(range(0, 43))
data_trace_fluid = create_traces_list_by_num(np.asarray(range(1, 230, 10)), data_fluid_perfomance,  numbers_fluid_perfomance_list)
data = data_trace_fluid
plot_func("Расчет ЭЦН на разных режимах работы - fluid", "Расчет ЭЦН на разных режимах работы - fluid.html")

numbers_fluid_flow_perfomance_list = list(range(6, 30))
data_trace_fluid_flow = create_traces_list_by_num(np.asarray(range(1, 230, 10)), data_fluid_flow_perfomance,  numbers_fluid_flow_perfomance_list)
data = data_trace_fluid_flow
plot_func("Расчет ЭЦН на разных режимах работы - fluid_flow", "Расчет ЭЦН на разных режимах работы - fluid_flow.html")
