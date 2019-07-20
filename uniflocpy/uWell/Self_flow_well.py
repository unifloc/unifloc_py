import uniflocpy.uTools.uconst as uc
import uniflocpy.uTools.data_workflow as data_workflow
import uniflocpy.uWell.uPipe as pipe
import uniflocpy.uWell.deviation_survey as well_profile
import numpy as np

class self_flow_well():
    def __init__(self, h_conductor_mes_m=500, h_conductor_vert_m=500,
                 h_intake_mes_m=1000, h_intake_vert_m=1000,
                 h_bottomhole_mes_m=1500, h_bottomhole_vert_m=1500,
                 qliq_on_surface_m3day = 100, fw_perc = 10,
                 d_casing_inner_m = 0.062, d_tube_inner_m = 0.120,
                 p_bottomhole_bar = 200, t_bottomhole_c = 92):

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

        self.p_intake_bar = None
        self.t_intake_c = None

        self.p_wellhead_bar = None
        self.t_wellhead_c = None

        self.well_profile = well_profile.simple_well_deviation_survey()

        self.casing_pipe = pipe.Pipe()
        self.tube_pipe = pipe.Pipe()

        self.data = data_workflow.Data()

        self.qliq_on_surface_m3day = qliq_on_surface_m3day
        self.fw_perc = fw_perc

        self.well_work_time_sec = 60 * 24 * 60 * 60

        self.step_lenth_in_calc_along_wellbore_m = 10

        self.t_earth_init_on_surface_c = 3
        self.t_earth_init_in_reservoir_c = 90
        self.geothermal_grad_cm = 0.03

        self.h_calculated_vert_m = None
        self.h_calculated_mes_m = None
        self.p_calculated_bar = None
        self.t_calculated_c = None
        self.t_calculated_earth_init = None
        self.t_grad_calculated_cm = None
        self.p_grad_calculated_barm = None

    def __transfer_data_to_pipe__(self, pipe_object, section_casing,  d_inner_pipe_m):
        pipe_object.section_casing = section_casing
        pipe_object.fluid_flow.qliq_on_surface_m3day = self.qliq_on_surface_m3day
        pipe_object.fluid_flow.fw_perc = self.fw_perc
        pipe_object.time_sec = self.well_work_time_sec
        pipe_object.fluid_flow.d_m = d_inner_pipe_m

    def __calc_pipe__(self, pipe_object):
        pipe_object.t_earth_init_c = self.t_calculated_earth_init

        self.p_grad_calculated_barm = uc.Pa2bar(pipe_object.calc_p_grad_pam(self.p_calculated_bar,
                                                                                 self.t_calculated_c))
        self.t_grad_calculated_cm = pipe_object.calc_t_grad_cm(self.p_calculated_bar, self.t_calculated_c)

        self.step_lenth_calculated_along_vert_m = np.abs(self.well_profile.get_h_vert_m(self.h_calculated_mes_m -
                                                                                        self.step_lenth_in_calc_along_wellbore_m) -
                                                         self.well_profile.get_h_vert_m(self.h_calculated_mes_m))

        self.p_calculated_bar -= self.p_grad_calculated_barm * self.step_lenth_in_calc_along_wellbore_m
        self.t_calculated_c -= self.t_grad_calculated_cm * self.step_lenth_in_calc_along_wellbore_m
        self.h_calculated_mes_m -= self.step_lenth_in_calc_along_wellbore_m
        self.h_calculated_vert_m = self.well_profile.get_h_vert_m(self.h_calculated_mes_m)

        self.t_calculated_earth_init -= self.geothermal_grad_cm * self.step_lenth_calculated_along_vert_m
        self.data.get_data(self)


    def calc_all_from_down_to_up(self):

        self.well_profile.h_conductor_mes_m = self.h_conductor_mes_m
        self.well_profile.h_conductor_vert_m = self.h_conductor_vert_m
        self.well_profile.h_pump_mes_m = self.h_intake_mes_m
        self.well_profile.h_pump_vert_m = self.h_intake_vert_m
        self.well_profile.h_bottomhole_mes_m = self.h_bottomhole_mes_m
        self.well_profile.h_bottomhole_vert_m = self.h_bottomhole_vert_m
        self.well_profile.calc_all()

        self.__transfer_data_to_pipe__(self.casing_pipe, True, self.d_casing_inner_m)

        self.h_calculated_mes_m = self.h_bottomhole_mes_m
        self.h_calculated_vert_m = self.h_bottomhole_vert_m
        self.p_calculated_bar = self.p_bottomhole_bar
        self.t_calculated_c = self.t_bottomhole_c
        self.t_calculated_earth_init = self.t_earth_init_in_reservoir_c
        self.step_lenth_calculated_along_vert_m = (self.well_profile.get_h_vert_m(self.h_calculated_mes_m -
                                                                                 self.step_lenth_in_calc_along_wellbore_m) -
                                                   self.well_profile.get_h_vert_m(self.h_calculated_mes_m))

        self.data.clear_data()
        self.data.get_data(self)
        while self.h_calculated_mes_m >= self.h_intake_mes_m:
            self.__calc_pipe__(self.casing_pipe)

        self.__transfer_data_to_pipe__(self.tube_pipe, False, self.d_tube_inner_m)
        while self.h_calculated_mes_m < self.h_intake_mes_m and self.h_calculated_mes_m > self.step_lenth_in_calc_along_wellbore_m:
            self.__calc_pipe__(self.tube_pipe)




        self.tube_pipe.section_casing = False


#simple_well = self_flow_well()
#print("kek")
#simple_well.well_work_time_sec = 24 * 60 * 60
#simple_well.calc_all_from_down_to_up()
#print(simple_well.p_calculated_bar)