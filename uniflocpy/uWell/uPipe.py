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

        self.t_out_bar = None
        self.t_in_c = None
        self.h_mes_out = None
        self.h_mes_in_c = None

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

        return  self.p_grad_pam

    def calc_t_grad_cm(self, p_bar, t_c):
        """расчет градиента температуры"""
        self.p_bar = p_bar
        self.t_c = t_c
        self.fluid_flow.calc(self.p_bar, self.t_c)

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

        self.temp_cor.sigma_liq_Nm = self.temp_cor.sigma_liq_Nm
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
