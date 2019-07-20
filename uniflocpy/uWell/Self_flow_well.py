import uniflocpy.uTools.uconst as uc
import uniflocpy.uTools.data_workflow as data_workflow
import uniflocpy.uWell.uPipe as pipe
import uniflocpy.uWell.deviation_survey as deviation_survey

class self_flow_well():
    def __init__(self):
        self.h_intake_mes_m = None
        self.h_intake_vert_m = None

        self.h_bottomhole_mes_m = None
        self.h_bottomhole_vert_m = None

        self.d_casing_m = None
        self.d_tube_m = None

        self.p_bottomhole_bar = None
        self.t_bottomhole_c = None

        self.p_intake_bar = None
        self.t_intake_c = None

        self.p_wellhead_bar = None
        self.t_wellhead_c = None

        self.deviation_survey = deviation_survey()

    def calc_all_from_down_to_up(self):

