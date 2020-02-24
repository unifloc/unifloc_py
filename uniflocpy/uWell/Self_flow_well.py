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
import uniflocpy.uMultiphaseFlow.hydr_cor_Beggs_Brill as hydr_cor_BB
import uniflocpy.uTemperature.temp_cor_Hasan_Kabir as temp_cor_HK
import numpy as np

class self_flow_well():
    def __init__(self, fluid=PVT_fluids.FluidStanding(),
                 well_profile=deviation_survey.simple_well_deviation_survey(),
                 data=data_workflow.Data(),
                 pipe=uPipe.Pipe(),
                 h_conductor_mes_m=500, h_conductor_vert_m=500,
                 h_intake_mes_m=1000, h_intake_vert_m=1000,
                 h_bottomhole_mes_m=1500, h_bottomhole_vert_m=1500,
                 qliq_on_surface_m3day=100, fw_perc=10,
                 d_casing_inner_m=0.120, d_tube_inner_m=0.062,
                 p_bottomhole_bar=200, t_bottomhole_c=92,
                 p_wellhead_bar=20, t_wellhead_c=20,
                 t_earth_init_on_surface_c=3, t_earth_init_in_reservoir_c=90, geothermal_grad_cm=0.03,
                 well_work_time_sec=60 * 24 * 60 * 60, step_lenth_in_calc_along_wellbore_m=10, without_annulus_space=False):
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
        :param fw_perc: обводненность продукции на поверхности, %
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

        self.well_profile = well_profile
        self.pipe = pipe
        self.pipe.fluid_flow.fl = fluid
        self.data = data

        self.qliq_on_surface_m3day = qliq_on_surface_m3day
        self.fw_perc = fw_perc

        self.h_calculated_vert_m = None
        self.h_calculated_mes_m = None
        self.p_calculated_bar = None
        self.t_calculated_c = None
        self.t_calculated_earth_init = None
        self.t_grad_calculated_cm = None
        self.p_grad_calculated_barm = None

        self.without_annulus_space = without_annulus_space

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
        pipe_object.fluid_flow.fw_perc = self.fw_perc
        pipe_object.time_sec = self.well_work_time_sec
        pipe_object.fluid_flow.d_m = d_inner_pipe_m
        pipe_object.t_in_c = self.t_bottomhole_c
        pipe_object.t_out_c = self.t_wellhead_c
        pipe_object.h_mes_in_m = self.h_bottomhole_mes_m
        pipe_object.h_mes_out_m = 0

    def __calc_pipe__(self, pipe_object, option_last_calc_boolean = False):
        """
        Расчет трубы (НКТ или ОК) в текущей точке всех параметров, сохранение их в атрибуты класса и в хранилище
        data_workflow - self.data, а после вычисление параметров в следующей точке.

        :param pipe_object: экзмепляр класс Pipe - НКТ или ОК
        :param option_last_calc_boolean: опция последнего расчета - не вычисляются параметры в следующей точке
        :return: None
        """
        pipe_object.t_earth_init_c = self.t_calculated_earth_init
        pipe_object.angle_to_horizontal_grad = self.well_profile.get_angle_to_horizontal_grad(self.h_calculated_mes_m)

        self.p_grad_calculated_barm = uc.Pa2bar(pipe_object.calc_p_grad_pam(self.p_calculated_bar,
                                                                                 self.t_calculated_c))
        self.t_grad_calculated_cm = pipe_object.calc_t_grad_cm(self.p_calculated_bar, self.t_calculated_c)

        self.data.get_data(self)
        if not option_last_calc_boolean:
            self.step_lenth_calculated_along_vert_m = np.abs(self.well_profile.get_h_vert_m(self.h_calculated_mes_m -
                                                                                            self.step_lenth_in_calc_along_wellbore_m) -
                                                             self.well_profile.get_h_vert_m(self.h_calculated_mes_m))
            self.p_calculated_bar -= self.p_grad_calculated_barm * self.step_lenth_in_calc_along_wellbore_m
            self.t_calculated_c -= self.t_grad_calculated_cm * self.step_lenth_in_calc_along_wellbore_m
            self.h_calculated_mes_m -= self.step_lenth_in_calc_along_wellbore_m
            self.h_calculated_vert_m = self.well_profile.get_h_vert_m(self.h_calculated_mes_m)
            self.t_calculated_earth_init -= self.geothermal_grad_cm * self.step_lenth_calculated_along_vert_m
    def calc_all_from_down_to_up(self):
        """
        Расчет фонтанирующей скважины методом снизу-вверх

        :return: None
        """

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

import uniflocpy.uPVT.BlackOil_model as BlackOil_model
import pandas as pd
import uniflocpy.uTemperature.temp_cor_simple_line as temp_cor_simple_line

calc_options ={"step_lenth_in_calc_along_wellbore_m":33,
                "without_annulus_space":False}

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
             "fw_perc": 0}

real_measurements = pd.DataFrame(
    {'p_survey_mpa': [0.975, 1.12, 1.83, 2.957, 4.355, 5.785, 7.3, 8.953, 9.863, 10.176, 11.435],
     'h_mes_survey_m': [0, 105, 305, 505, 705, 905, 1105, 1305, 1405, 1505, 1605]})
fluid = BlackOil_model.Fluid(gamma_oil=gamma_oil, gamma_gas=gamma_gas, rsb_m3m3=rsb_m3m3)
simple_well = self_flow_well(fluid=fluid,
                             pipe=uPipe.Pipe(temp_cor=temp_cor_simple_line.SimpleLineCor()), **well_data, **calc_options)
simple_well.well_work_time_sec = 1

simple_well.calc_all_from_down_to_up()