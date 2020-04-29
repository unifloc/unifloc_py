"""
Модуль-интерфейс описания работы фонтанирующей скважины

Кобзарь О.С. Хабибуллин Р.А. 20.07.2019 г
"""

# TODO добавить возможноть извлечения всех доступных данных через единственный self.data
# TODO добавить расчет методом снизу вверх
# TODO добавить возможность добавления нескольких колонн НКТ и ОК
# TODO добавить конструкцию трубы - толщины, диаметры внешние и внутренние

import uniflocpy.uTools.uconst as uc
import uniflocpy.uTools.data_workflow as data_workflow
import uniflocpy.uWell.uPipe as uPipe
import uniflocpy.uWell.deviation_survey as deviation_survey
import uniflocpy.uPVT.PVT_fluids as PVT_fluids
import uniflocpy.uPVT.BlackOil_model as BlackOil_model

import numpy as np
import uniflocpy.uReservoir.IPR_simple_line as IPR_simple_line
import uniflocpy.uTemperature as uTemperature
import uniflocpy.uMultiphaseFlow as uMultiphaseFlow

from scipy.integrate import solve_ivp
import time

class self_flow_well():
    def __init__(self, fluid=0,
                 well_profile=0,
                 hydr_corr=0,
                 temp_corr=0,
                 pipe=0,
                 reservoir=-1,

                 gamma_oil=0.86, gamma_gas=0.6, gamma_wat=1.0, rsb_m3m3=200.0,

                 h_conductor_mes_m=500, h_conductor_vert_m=500,
                 h_intake_mes_m=1000, h_intake_vert_m=1000,
                 h_bottomhole_mes_m=1500, h_bottomhole_vert_m=1500,
                 d_casing_inner_m=0.120, d_tube_inner_m=0.062,

                 qliq_on_surface_m3day=100, fw_on_surface_perc=10,
                 p_bottomhole_bar=200, t_bottomhole_c=92,
                 p_wellhead_bar=20, t_wellhead_c=20,
                 t_earth_init_on_surface_c=3, t_earth_init_in_reservoir_c=90, geothermal_grad_cm=0.03,
                 p_reservoir_bar=None,
                 well_work_time_sec=60 * 24 * 60 * 60,

                 step_lenth_in_calc_along_wellbore_m=10,
                 without_annulus_space=False,
                 save_all=True,
                 solver_using=0,

                activate_rus_mode=0,

                 pb_bar=90):
        """
        При создании модели скважины необходимо задать ее конструкцию, PVT свойства флюидов и режим работы
        вместе с граничными условиями. Кроме параметров, которые предлагается задать при
        инициализации, можно изменить и другие, входящие в состав модели, путем обращения к необходимым
        модулям. На что стоит обрать внимание: некоторые параметры выставлены по умолчанию и изменение
        всех интересующих параметров необходимо выполнить до процесса расчета.

        :param h_conductor_mes_m: измеренная глубина конца кондуктора, м
        :param h_conductor_vert_m: вертикальная глубина конца кондуктора, м
        :param h_intake_mes_m: измеренная глубина конца колонны НКТ (спуска НКТ), м
        :param h_intake_vert_m: вертикальная глубина конца колонны НКТ (спуска НКТ), м
        :param h_bottomhole_mes_m: измеренная глубина забоя, м
        :param h_bottomhole_vert_m: вертикальная глубина забоя, м
        :param qliq_on_surface_m3day: дебит жидкости на поверхности, м3/сутки
        :param fw_on_surface_perc: обводненность продукции на поверхности, %
        :param d_casing_inner_m: внутренний диаметр обсадной колонны, м
        :param d_tube_inner_m: внутренни диаметр НКТ
        :param p_bottomhole_bar: давление на забое, бар
        :param t_bottomhole_c: температура на забое, С
        :param p_wellhead_bar: давление на устье, бар
        :param t_wellhead_c: температура на устье, С
        :param t_earth_init_on_surface_c: начальная температура земли на поверхности (нейтрального слоя), С
        :param t_earth_init_in_reservoir_c: начальная температура пласта, С
        :param geothermal_grad_cm: геотермический градиент, С/м
        :param well_work_time_sec: время работы скважины, сек
        :param step_lenth_in_calc_along_wellbore_m: длина шага вдоль ствола скважины в расчете, м
        """

        self.h_conductor_mes_m = h_conductor_mes_m
        self.h_conductor_vert_m = h_conductor_vert_m
        self.h_intake_mes_m = h_intake_mes_m
        self.h_intake_vert_m = h_intake_vert_m
        self.h_bottomhole_mes_m = h_bottomhole_mes_m
        self.h_bottomhole_vert_m = h_bottomhole_vert_m
        self.d_casing_inner_m = d_casing_inner_m
        self.d_tube_inner_m = d_tube_inner_m

        self.p_bottomhole_bar = p_bottomhole_bar
        self.t_bottomhole_c = t_bottomhole_c
        self.p_wellhead_bar = p_wellhead_bar
        self.t_wellhead_c = t_wellhead_c

        self.well_work_time_sec = well_work_time_sec

        self.step_lenth_in_calc_along_wellbore_m = step_lenth_in_calc_along_wellbore_m

        self.t_earth_init_on_surface_c = t_earth_init_on_surface_c
        self.t_earth_init_in_reservoir_c = t_earth_init_in_reservoir_c
        self.geothermal_grad_cm = geothermal_grad_cm
        self.p_reservoir_bar = p_reservoir_bar

        if well_profile == 0:
            self.well_profile = deviation_survey.simple_well_deviation_survey()
        elif well_profile == 1:
            self.well_profile = deviation_survey.well_deviation_survey()

        if pipe == 0:
            self.pipe = uPipe.Pipe()

        if hydr_corr == 0:
            self.pipe.hydr_cor = uMultiphaseFlow.hydr_cor_Beggs_Brill.Beggs_Brill_cor()

        if temp_corr == 0:
            self.pipe.temp_cor = uTemperature.temp_cor_Hasan_Kabir.Hasan_Kabir_cor()
        elif temp_corr == 1:
            self.pipe.temp_cor = uTemperature.temp_cor_simple_line.SimpleLineCor()

        if fluid == 0:
            self.pipe.fluid_flow.fl = PVT_fluids.FluidStanding(gamma_oil=gamma_oil, gamma_gas=gamma_gas,
                                                               gamma_wat=gamma_wat, rsb_m3m3=rsb_m3m3)
        elif fluid == 1:
            self.pipe.fluid_flow.fl = BlackOil_model.Fluid(gamma_oil=gamma_oil, gamma_gas=gamma_gas,
                                                               gamma_wat=gamma_wat, rsb_m3m3=rsb_m3m3,
                                                           t_res_c=t_bottomhole_c, pb_bar=pb_bar)
        if activate_rus_mode:
            self.pipe.fluid_flow.fl = BlackOil_model.Fluid(gamma_oil=gamma_oil, gamma_gas=gamma_gas,
                                                               gamma_wat=gamma_wat, rsb_m3m3=rsb_m3m3,
                                                       t_res_c=t_bottomhole_c, pb_bar=pb_bar, activate_rus_cor=1)

        self.data = data_workflow.Data()

        self.qliq_on_surface_m3day = qliq_on_surface_m3day
        self.fw_on_surface_perc = fw_on_surface_perc

        self.h_calculated_vert_m = None
        self.h_calculated_mes_m = None
        self.p_calculated_bar = None
        self.t_calculated_c = None
        self.t_calculated_earth_init = None
        self.t_grad_calculated_cm = None
        self.p_grad_calculated_barm = None

        self.without_annulus_space = without_annulus_space
        self.save_all = save_all

        if reservoir == -1:
            self.ipr = None
        elif reservoir == 0:
            self.ipr = IPR_simple_line.IPRSimpleLine()
            if self.p_reservoir_bar == None:
                self.p_reservoir_bar = 1000 * uc.g * self.h_bottomhole_vert_m / 100000
            self.ipr.pi_m3daybar = self.ipr.calc_pi_m3daybar(self.qliq_on_surface_m3day, self.p_bottomhole_bar,
                                                      self.p_reservoir_bar)

        self.direction_up = None
        self.solver_using = solver_using

        self.time_calculated_sec = None
        self.calculation_number_in_one_step = None

    def __transfer_data_to_pipe__(self, pipe_object, section_casing,  d_inner_pipe_m):
        """
        Происходит изменение параметров в используемом подмодуле - трубе -
        используя данные, заданые в классе self_flow_well

        :param pipe_object: экземпляр класса Pipe - НКТ или ОК
        :param section_casing: определение типа Pipe: True - ОК, False - НКТ
        :param d_inner_pipe_m: внутренний диаметр трубы, м
        :return: None
        """
        pipe_object.section_casing = section_casing
        pipe_object.fluid_flow.qliq_on_surface_m3day = self.qliq_on_surface_m3day
        pipe_object.fluid_flow.fw_on_surface_perc = self.fw_on_surface_perc
        pipe_object.time_sec = self.well_work_time_sec
        pipe_object.fluid_flow.d_m = d_inner_pipe_m
        pipe_object.t_in_c = self.t_bottomhole_c
        pipe_object.t_out_c = self.t_wellhead_c
        pipe_object.h_mes_in_m = self.h_bottomhole_mes_m
        pipe_object.h_mes_out_m = 0

    def __init_construction__(self):
        self.well_profile.h_conductor_mes_m = self.h_conductor_mes_m
        self.well_profile.h_conductor_vert_m = self.h_conductor_vert_m
        self.well_profile.h_pump_mes_m = self.h_intake_mes_m
        self.well_profile.h_pump_vert_m = self.h_intake_vert_m
        self.well_profile.h_bottomhole_mes_m = self.h_bottomhole_mes_m
        self.well_profile.h_bottomhole_vert_m = self.h_bottomhole_vert_m
        self.well_profile.lenth_of_one_part = self.step_lenth_in_calc_along_wellbore_m
        self.well_profile.calc_all()
        return None


    def calc_p_grad_pam_for_scipy(self, h_m, p_bar, t_c, pipe_object):
        p_bar = float(p_bar)
        t_c = float(t_c)
        p_grad_pam = pipe_object.calc_p_grad_pam(p_bar, t_c)
        return uc.Pa2bar(p_grad_pam)

    def __calc_pipe__(self, pipe_object, option_last_calc_boolean=False):
        """
        Расчет трубы (НКТ или ОК) в текущей точке всех параметров, сохранение их в атрибуты класса и в хранилище
        data_workflow - self.data, а после вычисление параметров в следующей точке.

        :param pipe_object: экзмепляр класс Pipe - НКТ или ОК
        :param option_last_calc_boolean: опция последнего расчета - не вычисляются параметры в следующей точке
        :return: None
        """
        #print(f"В начале расчета\n"
        #      f"Давление: {self.p_calculated_bar} и температура {self.t_calculated_c} "
        #      f"на измеренной глубине {self.h_calculated_mes_m}"
        #      f"\n")
        start_calculation_time = time.time()
        if self.direction_up:
            sign = 1
        else:
            sign = - 1
        pipe_object.t_earth_init_c = self.t_calculated_earth_init
        pipe_object.angle_to_horizontal_grad = self.well_profile.get_angle_to_horizontal_grad(self.h_calculated_mes_m)

        self.p_grad_calculated_barm = uc.Pa2bar(pipe_object.calc_p_grad_pam(self.p_calculated_bar,
                                                                                 self.t_calculated_c))
        self.t_grad_calculated_cm = pipe_object.calc_t_grad_cm(self.p_calculated_bar, self.t_calculated_c)
        if self.save_all:
            self.data.get_data(self)
        if not option_last_calc_boolean:
            self.step_lenth_calculated_along_vert_m = np.abs(self.well_profile.get_h_vert_m(self.h_calculated_mes_m -
                                                                                            self.step_lenth_in_calc_along_wellbore_m) -
                                                             self.well_profile.get_h_vert_m(self.h_calculated_mes_m))

            if self.solver_using == 1:
                new_p_calculated_bar_solve_output = solve_ivp(self.calc_p_grad_pam_for_scipy,
                                                              t_span=(self.h_calculated_mes_m,
                                                                      self.h_calculated_mes_m - self.step_lenth_in_calc_along_wellbore_m * sign),
                                                              y0=[self.p_calculated_bar],
                                                              args=(self.t_calculated_c, pipe_object),
                                                              rtol=0.001, atol=0.001
                                                              )
                #print(new_p_calculated_bar_solve_output)
                #print('\n')
                new_p_calculated_bar = new_p_calculated_bar_solve_output.y[-1][-1]
                #print(f"new_p_calculated_bar = {new_p_calculated_bar}")
                self.p_calculated_bar = new_p_calculated_bar
                self.calculation_number_in_one_step = new_p_calculated_bar_solve_output.nfev
            else:
                self.p_calculated_bar -= self.p_grad_calculated_barm * self.step_lenth_in_calc_along_wellbore_m * sign
                self.calculation_number_in_one_step = 1
            #if self.p_calculated_bar < 1:
            #    self.p_calculated_bar = 1
            self.t_calculated_c -= self.t_grad_calculated_cm * self.step_lenth_in_calc_along_wellbore_m * sign
            self.h_calculated_mes_m -= self.step_lenth_in_calc_along_wellbore_m * sign
            self.h_calculated_vert_m = self.well_profile.get_h_vert_m(self.h_calculated_mes_m)
            self.t_calculated_earth_init -= self.geothermal_grad_cm * self.step_lenth_calculated_along_vert_m * sign
            #print(f"Давление: {self.p_calculated_bar} и температура {self.t_calculated_c} "
            #      f"на измеренной глубине {self.h_calculated_mes_m}")

        self.time_calculated_sec = time.time() - start_calculation_time


    def calc_all_from_down_to_up(self):
        """
        Расчет фонтанирующей скважины методом снизу-вверх

        :return: None
        """
        self.direction_up = True

        self.well_profile.h_conductor_mes_m = self.h_conductor_mes_m
        self.well_profile.h_conductor_vert_m = self.h_conductor_vert_m
        self.well_profile.h_pump_mes_m = self.h_intake_mes_m
        self.well_profile.h_pump_vert_m = self.h_intake_vert_m
        self.well_profile.h_bottomhole_mes_m = self.h_bottomhole_mes_m
        self.well_profile.h_bottomhole_vert_m = self.h_bottomhole_vert_m
        self.well_profile.lenth_of_one_part = self.step_lenth_in_calc_along_wellbore_m
        self.well_profile.calc_all()

        self.h_calculated_mes_m = self.h_bottomhole_mes_m
        self.h_calculated_vert_m = self.h_bottomhole_vert_m
        if self.ipr != None:
            self.p_calculated_bar = self.ipr.calc_p_bottomhole_bar(self.qliq_on_surface_m3day)
            self.p_bottomhole_bar = self.p_calculated_bar
        else:
            self.p_calculated_bar = self.p_bottomhole_bar
        self.t_calculated_c = self.t_bottomhole_c
        self.t_calculated_earth_init = self.t_earth_init_in_reservoir_c
        self.step_lenth_calculated_along_vert_m = (self.well_profile.get_h_vert_m(self.h_calculated_mes_m -
                                                                                 self.step_lenth_in_calc_along_wellbore_m) -
                                                   self.well_profile.get_h_vert_m(self.h_calculated_mes_m))
        self.data.clear_data()

        self.__transfer_data_to_pipe__(self.pipe, section_casing=True, d_inner_pipe_m=self.d_casing_inner_m)
        # casing calc
        while self.h_calculated_mes_m >= self.h_intake_mes_m + self.step_lenth_in_calc_along_wellbore_m:
            self.__calc_pipe__(self.pipe)
        # last calc in casing
        step_lenth_in_calc_along_wellbore_m = self.step_lenth_in_calc_along_wellbore_m
        self.step_lenth_in_calc_along_wellbore_m = self.h_calculated_mes_m-self.h_intake_mes_m * 0.9999
        self.__calc_pipe__(self.pipe)
        self.step_lenth_in_calc_along_wellbore_m = step_lenth_in_calc_along_wellbore_m

        if self.without_annulus_space:
            self.__transfer_data_to_pipe__(self.pipe, section_casing=True, d_inner_pipe_m=self.d_tube_inner_m)
        else:
            self.__transfer_data_to_pipe__(self.pipe, section_casing=False, d_inner_pipe_m=self.d_tube_inner_m)
        # tubing calc
        while self.h_intake_mes_m > self.h_calculated_mes_m >= self.step_lenth_in_calc_along_wellbore_m:
            self.__calc_pipe__(self.pipe)

        # last step in tubing before 0 point
        step_lenth_in_calc_along_wellbore_m = self.step_lenth_in_calc_along_wellbore_m
        self.step_lenth_in_calc_along_wellbore_m = self.h_calculated_mes_m
        self.__calc_pipe__(self.pipe, option_last_calc_boolean=False)
        self.step_lenth_in_calc_along_wellbore_m = step_lenth_in_calc_along_wellbore_m
        # calc grad in 0 point and save
        self.__calc_pipe__(self.pipe, option_last_calc_boolean=True)
        if not self.save_all:
            self.data.get_data(self)
        self.p_wellhead_bar = self.p_calculated_bar
        self.t_wellhead_c = self.t_calculated_c

    def calc_all_from_up_to_down(self):
        """
        Расчет фонтанирующей скважины методом снизу-вверх

        :return: None
        """
        self.direction_up = False

        self.__init_construction__()

        self.h_calculated_mes_m = 0
        self.h_calculated_vert_m = 0

        self.p_calculated_bar = self.p_wellhead_bar
        self.t_calculated_c = self.t_wellhead_c

        self.t_calculated_earth_init = self.t_wellhead_c  # TODO

        self.step_lenth_calculated_along_vert_m = (self.well_profile.get_h_vert_m(self.h_calculated_mes_m +
                                                                                 self.step_lenth_in_calc_along_wellbore_m) -
                                                   self.well_profile.get_h_vert_m(self.h_calculated_mes_m))
        self.data.clear_data()

        # tubing calc
        if self.without_annulus_space:
            self.__transfer_data_to_pipe__(self.pipe, section_casing=True, d_inner_pipe_m=self.d_tube_inner_m)
        else:
            self.__transfer_data_to_pipe__(self.pipe, section_casing=False, d_inner_pipe_m=self.d_tube_inner_m)

        while self.h_calculated_mes_m <= self.h_intake_mes_m - self.step_lenth_in_calc_along_wellbore_m:
            self.__calc_pipe__(self.pipe)

        # last calc in tubing
        step_lenth_in_calc_along_wellbore_m = self.step_lenth_in_calc_along_wellbore_m
        self.step_lenth_in_calc_along_wellbore_m = self.h_intake_mes_m - self.h_calculated_mes_m * 0.9999
        self.__calc_pipe__(self.pipe)
        self.step_lenth_in_calc_along_wellbore_m = step_lenth_in_calc_along_wellbore_m

        # casing calc
        self.__transfer_data_to_pipe__(self.pipe, section_casing=True, d_inner_pipe_m=self.d_casing_inner_m)
        while self.h_calculated_mes_m <= self.h_bottomhole_mes_m - self.step_lenth_in_calc_along_wellbore_m:
            self.__calc_pipe__(self.pipe)

        # last step in casing before 0 point
        step_lenth_in_calc_along_wellbore_m = self.step_lenth_in_calc_along_wellbore_m
        self.step_lenth_in_calc_along_wellbore_m = self.h_bottomhole_mes_m - self.h_calculated_mes_m
        self.__calc_pipe__(self.pipe, option_last_calc_boolean=False)
        self.step_lenth_in_calc_along_wellbore_m = step_lenth_in_calc_along_wellbore_m
        # calc grad in 0 point and save
        self.__calc_pipe__(self.pipe, option_last_calc_boolean=True)
        if not self.save_all:
            self.data.get_data(self)
        self.p_bottomhole_bar = self.p_calculated_bar
        self.t_bottomhole_c = self.t_calculated_c


import pandas as pd
import uniflocpy.uTemperature.temp_cor_simple_line as temp_cor_simple_line




if __name__ == "__main__":
    calc_options ={"step_lenth_in_calc_along_wellbore_m": 100,
                    "without_annulus_space": False,
                   "save_all": True}

    rsb_m3m3 = 56
    pb_bar = 9 * 10 ** 5
    gamma_oil = 0.86
    gamma_gas = 1.45

    well_data = {"h_intake_mes_m": 1205,
                 "h_intake_vert_m": 1205,
                 "h_bottomhole_mes_m": 1607,  # 1756.8
                 "h_bottomhole_vert_m": 1607,

                 "geothermal_grad_cm": 0.02,
                 "t_bottomhole_c": 40,
                 "t_earth_init_in_reservoir_c": 40,
                 'p_bottomhole_bar': 114.35,
                 "d_casing_inner_m": 0.133,
                 "d_tube_inner_m": 0.0503,
                 "qliq_on_surface_m3day": 40,
                 "fw_on_surface_perc": 0}

    real_measurements = pd.DataFrame(
        {'p_survey_mpa': [0.975, 1.12, 1.83, 2.957, 4.355, 5.785, 7.3, 8.953, 9.863, 10.176, 11.435],
         'h_mes_survey_m': [0, 105, 305, 505, 705, 905, 1105, 1305, 1405, 1505, 1605]})


    well_data['qliq_on_surface_m3day'] = 20

    fluid = BlackOil_model.Fluid()
    simple_well = self_flow_well(fluid=1, reservoir=0,
                                 temp_corr=1, gamma_oil=gamma_oil, gamma_gas=gamma_gas, rsb_m3m3=rsb_m3m3,
                                 **well_data, **calc_options,
                                 solver_using=1)
    simple_well.well_work_time_sec = 1

    simple_well.p_wellhead_bar = 20
    simple_well.t_wellhead_c = 20

    import time
    start = time.time()
    simple_well.calc_all_from_down_to_up()
    stop = time.time()
    print(f"stop-start={stop-start}")
    print(f"simple_well.p_wellhead_bar={simple_well.p_wellhead_bar}")
    print(f"simple_well.p_bottomhole_bar={simple_well.p_bottomhole_bar}")
    print(f"simple_well.p_calculated_bar={simple_well.p_calculated_bar}")
    print(f"simple_well.calculation_number_in_one_step={simple_well.calculation_number_in_one_step}")



    print('\nРазворот\n')
    start = time.time()

    simple_well.calc_all_from_up_to_down()
    stop = time.time()
    print(f"stop-start={stop-start}")
    print(f"simple_well.p_wellhead_bar={simple_well.p_wellhead_bar}")
    print(f"simple_well.p_bottomhole_bar={simple_well.p_bottomhole_bar}")
    print(f"simple_well.p_calculated_bar={simple_well.p_calculated_bar}")
    print(f"simple_well.calculation_number_in_one_step={simple_well.calculation_number_in_one_step}")

