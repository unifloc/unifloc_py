"""Класс-интерфейс для расчета потока через трубу,
объединяет расчеты: PVT, многофазный поток, гидравлический и
температурный расчет"""
import uniflocpy.uPVT.PVT_fluids as PVT
import uniflocpy.uMultiphaseFlow.hydr_cor_Beggs_Brill as hydr_cor_BB
import uniflocpy.uTemperature.temp_cor_Hasan_Kabir as temp_cor_HK
import uniflocpy.uTools.uconst as uc


class Pipe():
    """
    Расчет параметров течения ГЖС вдоль трубы
    """
    def __init__(self, fluid=PVT.FluidStanding(),
                 hydr_cor=hydr_cor_BB.Beggs_Brill_cor(),
                 temp_cor=temp_cor_HK.Hasan_Kabir_cor()):
        """
        Инизицализация подъемника газожидкостной смеси
        :param fluid: модель флюида, класс
        :param hydr_cor: гидравлическая корреляция, класс
        :param temp_cor: температурная корреляция, класс
        """
        self.fluid_flow = PVT.FluidFlow(fluid=fluid)
        self.hydr_cor = hydr_cor
        self.temp_cor = temp_cor

        self.section_casing = False  # если True, будет считать ОК

        self.time_sec = 100 * 24 * 60 * 60

        self.t_earth_init_c = 45

        self.angle_to_horizontal_grad = 90

        self.p_bar = None
        self.t_c = None

        self.p_grad_pam = None
        self.t_grad_cm = None

        self.t_out_c = None
        self.t_in_c = None
        self.h_mes_out_m = None
        self.h_mes_in_m = None

        self.rho_slip_kgm3 = None

    def calc_p_grad_pam(self, p_bar, t_c):
        """расчет градиента давления"""
        self.p_bar = p_bar
        self.t_c = t_c
        self.fluid_flow.calc(self.p_bar, self.t_c)

        self.hydr_cor.angle_grad = self.angle_to_horizontal_grad
        self.hydr_cor.angle_rad = uc.grad2rad(self.angle_to_horizontal_grad)
        self.hydr_cor.d_m = self.fluid_flow.d_m

        self.hydr_cor.vsl_msec = self.fluid_flow.vsl_msec
        self.hydr_cor.vsg_msec = self.fluid_flow.vsg_msec
        self.hydr_cor.vm_msec = self.fluid_flow.vm_msec

        self.hydr_cor.liquid_content = self.fluid_flow.liquid_content

        self.hydr_cor.rho_liq_kgm3 = self.fluid_flow.rho_liq_kgm3
        self.hydr_cor.rho_gas_kgm3 = self.fluid_flow.fl.rho_gas_kgm3

        self.hydr_cor.mun_pas = self.fluid_flow.mun_cP / 10 ** 3
        self.hydr_cor.rhon_kgm3 = self.fluid_flow.rhon_kgm3

        self.hydr_cor.sigma_liq_Nm = self.fluid_flow.sigma_liq_Nm

        self.p_grad_pam = self.hydr_cor.calc_grad(self.p_bar, self.t_c)

        self.rho_slip_kgm3 = self.hydr_cor.rhos_kgm3

        return self.p_grad_pam

    def calc_t_grad_cm(self, p_bar, t_c):
        """расчет градиента температуры"""
        self.p_bar = p_bar
        self.t_c = t_c
        self.fluid_flow.calc(self.p_bar, self.t_c)

        self.temp_cor.t_out_c = self.t_out_c
        self.temp_cor.t_in_c = self.t_in_c
        self.temp_cor.h_mes_out = self.h_mes_out_m
        self.temp_cor.h_mes_in_c = self.h_mes_in_m

        self.temp_cor.angle_rad = uc.grad2rad(self.angle_to_horizontal_grad)
        self.temp_cor.section_casing = self.section_casing

        self.temp_cor.time_sec = self.time_sec

        self.temp_cor.t_earth_init_c = self.t_earth_init_c

        self.temp_cor.p_bar = self.p_bar
        self.temp_cor.t_c = self.t_c

        # конструкция скважины
        self.temp_cor.angle_rad = uc.grad2rad(90)
        self.temp_cor.d_m = self.fluid_flow.d_m

        self.temp_cor.r_tube_in_m = self.temp_cor.d_m / 2
        self.temp_cor.r_tube_out_m = self.temp_cor.r_tube_in_m + 0.005
        self.temp_cor.r_cas_in_m = self.temp_cor.r_tube_out_m + 0.03
        self.temp_cor.r_cas_out_m = self.temp_cor.r_cas_in_m + 0.005
        self.temp_cor.r_well_m = self.temp_cor.r_cas_out_m + 0.15

        # свойства многофазного потока
        self.temp_cor.liquid_content = self.fluid_flow.liquid_content
        self.temp_cor.mass_flowraten_kgsec = self.fluid_flow.mass_flowraten_kgsec
        self.temp_cor.vm_msec = self.fluid_flow.vm_msec

        self.temp_cor.sigma_liq_Nm = self.fluid_flow.sigma_liq_Nm
        self.temp_cor.rhon_kgm3 = self.fluid_flow.rhon_kgm3
        self.temp_cor.mun_cP = self.fluid_flow.mun_cP
        self.temp_cor.heatcapn_jkgc = self.fluid_flow.heatcapn_jkgc
        self.temp_cor.thermal_conductn_wmk = self.fluid_flow.thermal_conductn_wmk

        self.temp_cor.number_re_n = self.fluid_flow.number_re_n
        self.temp_cor.number_pr_n = self.fluid_flow.number_pr_n

        # свойства флюида, находящегося в затрубном пространстве  # TODO какое давление? газ или жидкость?
        self.temp_cor.mu_an_cP = 0.0001 * 1000
        self.temp_cor.heatcap_an_jkgc = 1004.81
        self.temp_cor.thermal_conduct_an_wmk = 0.865
        self.temp_cor.rho_an_kgm3 = 36.92
        self.temp_cor.heat_expansion_an_1c = 0.004824

        # параметры, входящие в градиент
        self.temp_cor.Joule_Thompson_coef_cpa = self.fluid_flow.Joule_Thompson_coef_cpa
        self.temp_cor.grad_p_pam = self.calc_p_grad_pam(self.p_bar, self.t_c)
        self.temp_cor.grad_v_msecm = self.fluid_flow.dvdp_msecpam * self.temp_cor.grad_p_pam

        self.t_grad_cm = self.temp_cor.calc_grad_t_cm(self.p_bar, self.t_c)
        return self.t_grad_cm

if __name__ == "__main__":
    import uniflocpy.uTools.data_workflow as data_workflow
    import pandas as pd
    from uniflocpy.uTools import plotly_workflow
    import uniflocpy.uPVT.BlackOil_model as BlackOil_model
    import uniflocpy.uValidation.python_api as python_api
    import uniflocpy.uTools.uconst as uc
    import uniflocpy.uPVT.PVT_fluids as PVT_fluids
    import xlwings as xw
    import uniflocpy.uMultiphaseFlow.hydr_cor_Beggs_Brill as BB
    pipe = Pipe(fluid=BlackOil_model.Fluid())
    uniflocvba = python_api.API('E:\\Git\\unifloc_vba\\UniflocVBA_7.xlam')
    uniflocvba.f_bb = uniflocvba.book.macro('unf_BegsBrillGradient')


    def convert_result_to_df(result):
        this_dict = {
            "p_grad_pam": uc.atm2Pa(result[0]),
            "density_part_pam": uc.atm2Pa(result[1]),
            "friction_part_pam": uc.atm2Pa(result[2]),
            "acceleration_part_pam": uc.atm2Pa(result[3]),
            "vsl_msec": result[4],
            "vsg_msec": result[5],
            "liquid_content_with_pains_corr": result[6],
            "flow_regime_number": result[7]
        }
        this_df = pd.DataFrame(this_dict, index=[0])
        return this_df


    p_bar = 8




    vba_result_df = None
    t_c = 40
    pipe.fluid_flow.qliq_on_surface_m3day = 40
    pipe.fluid_flow.fl.rsb_m3m3 = 56

    pipe.calc_p_grad_pam(p_bar, t_c)

    arr_d_m = pipe.fluid_flow.d_m
    arr_theta_deg = pipe.angle_to_horizontal_grad
    eps_m = pipe.hydr_cor.epsilon_friction_m
    Ql_rc_m3day = pipe.fluid_flow.qliq_m3day
    Qg_rc_m3day = pipe.fluid_flow.qgas_m3day
    Mul_rc_cP = pipe.fluid_flow.mu_liq_cP
    Mug_rc_cP = pipe.fluid_flow.fl.mu_gas_cp
    sigma_l_Nm = pipe.fluid_flow.sigma_liq_Nm
    rho_lrc_kgm3 = pipe.fluid_flow.rho_liq_kgm3
    rho_grc_kgm3 = pipe.fluid_flow.fl.rho_gas_kgm3
    Payne_et_all_holdup = 1,
    Payne_et_all_friction = 1
    c_calibr_grav = 1
    c_calibr_fric = 1

    vba_result = uniflocvba.f_bb(arr_d_m,
                                 arr_theta_deg, eps_m,
                                 Ql_rc_m3day, Qg_rc_m3day,
                                 Mul_rc_cP, Mug_rc_cP,
                                 sigma_l_Nm,
                                 rho_lrc_kgm3,
                                 rho_grc_kgm3,
                                 # Payne_et_all_holdup
                                 # Payne_et_all_friction ,
                                 # c_calibr_grav   ,
                                 # c_calibr_fric
                                 )
    this_vba_result_df = convert_result_to_df(vba_result)
    for i in this_vba_result_df.columns:
        print(i, this_vba_result_df[i][0])
    print(this_vba_result_df["liquid_content_with_pains_corr"][0])
    print(pipe.hydr_cor.liquid_holdup_d)
    print(pipe.hydr_cor.liquid_content)
    print(this_vba_result_df)
    for i in pipe.hydr_cor.__dict__.items():
        print(i)