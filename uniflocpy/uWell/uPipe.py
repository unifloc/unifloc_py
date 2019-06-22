"""Класс-интерфейс для расчета потока через трубу,
объединяет расчеты: PVT, многофазный поток, гидравлический и
температурный расчет"""
import uniflocpy.uPVT.PVT_fluids as PVT
import uniflocpy.uMultiphaseFlow.hydr_cor_Beggs_Brill as hydr_cor_BB
import uniflocpy.uTemperature.temp_cor_Hasan_Kabir as temp_cor_HK

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
        self.fluid_flow = PVT.FluidFlow(fluid = fluid)
        self.hydr_cor = hydr_cor
        self.temp_cor = temp_cor

        self.p_bar = None
        self.t_c = None

        self.p_grad_pam = None
        self.t_grad_cm = None

    def calc_p_grad_pam(self, p_bar, t_c):
        self.p_bar = p_bar
        self.t_c = t_c
        self.fluid_flow.calc(self.p_bar, self.t_c)

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
        self.p_bar = p_bar
        self.t_c = t_c
        self.fluid_flow.calc(self.p_bar, self.t_c)

        self.temp_cor.d_m = self.fluid_flow.d_m

        self.temp_cor.liquid_content = self.fluid_flow.liquid_content

        self.temp_cor.rhon_kgm3 = self.fluid_flow.rhon_kgm3
        self.temp_cor.mun_cP = self.fluid_flow.mun_cP
        self.temp_cor.heatcapn_jkgc = self.fluid_flow.heatcapn_jkgc
        self.temp_cor.thermal_conductn_wmk = self.fluid_flow.thermal_conductn_wmk
        self.temp_cor.mass_flowraten_kgsec = self.fluid_flow.mass_flowraten_kgsec

        self.temp_cor.number_re_n = self.fluid_flow.number_re_n
        self.temp_cor.number_pr_n = self.fluid_flow.number_pr_n

        self.t_grad_cm = self.temp_cor.calc_grad_t_cm(self.p_bar, self.t_c)
        return self.t_grad_cm

p_bar, t_c = 50, 50

pipe = Pipe()
answer = pipe.calc_t_grad_cm(p_bar, t_c)
print(answer)

