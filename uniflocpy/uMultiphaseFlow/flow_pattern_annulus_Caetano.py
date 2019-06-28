import uniflocpy.uTools.uconst as uc
import math
import scipy.optimize as sp

class flow_pattern_annulus_Caetano(object):
    """Класс для определения режимов потока в затрубном пространстве
    по Caetano (1992)"""

    def __init__(self):
        self.surface_tension_gl_Nm = 0.020
        self.rho_liq_kgm3 = 800
        self.rho_gas_kgm3 = 50

        self.d_cas_in_m = 0.155
        self.d_tube_out_m = 0.80

        self.rho_mix_kgm3 = self.rho_liq_kgm3
        self.mu_mix_pasec = 0.01

        self.vs_liq_msec = 0.5
        self.vs_gas_msec = 0.9
        self.vm_msec = self.vs_liq_msec + self.vs_gas_msec

        self.concentric_annulus = True

        # рассчитанные параметры
        self.equation_part = None
        self.v_infinite_z_msec = None
        self.v_Taylor_bubble_msec = None
        self.d_equi_periphery_m = None
        self.d_hydr_m = None

        self.flow_pattern = None

        self.v_slip_m3sec = None

        self.const_k = None

        self.friction_coeff = None

        self.d_max_bubble_m = None

        self.d_crit_bubble_m = None

        self.number_Re = None

        self.k_ratio_d = None

        self.Fca = None

        self.vs_gas_bubble2slug_msec = None
        self.vs_gas_dispbubble2slug_msec = None
        self.vs_gas_2annular_msec = None

    def __friction_coefficient_Gunn_Darling(self, initial_f):
        right_part = (4 * math.log(self.number_Re * (initial_f *
                                                (16 / self.Fca) ** (0.45 * math.exp(-(self.number_Re - 3000) / 10 ** 6))
                                                ) ** 0.5) - 0.4)
        left_part = 1 / (initial_f * (16 / self.Fca) ** (0.45 * math.exp(-(self.number_Re - 3000) / 10 ** 6))) ** 0.5

        return right_part - left_part


    def calc_pattern(self):
        # Bubble Flow Region Existence
        # Harmanty (1960)
        self.equation_part = (self.surface_tension_gl_Nm *
                                    (self.rho_liq_kgm3 - self.rho_gas_kgm3) * uc.g /
                                    self.rho_liq_kgm3 ** 2) * 0.25
        self.v_infinite_z_msec = 1.53 * self.equation_part
        self.d_equi_periphery_m = self.d_cas_in_m + self.d_tube_out_m
        # Nicklin (1962)
        self.v_Taylor_bubble_msec = 0.35 * (uc.g * self.d_equi_periphery_m) ** 0.5
        if self.v_Taylor_bubble_msec >= self.v_infinite_z_msec:
            self.flow_pattern = 0  # bubble flow pattern

        # Bubble to Slug Flow Transition
        if self.concentric_annulus:
            self.vs_gas_bubble2slug_msec = self.vs_liq_msec / 4 + 0.306 * self.equation_part
        else:
            self.vs_gas_bubble2slug_msec = self.vs_liq_msec / 5.67 + 0.230 * self.equation_part

        # bubble or slug to dispersed bubble flow transition
        # Calderbank (1958)
        self.const_k = 0.725 + 4.15 * (self.vs_gas_msec / self.vm_msec) ** 0.5
        self.d_hydr_m = self.d_cas_in_m - self.d_tube_out_m
        # Broodkey (1967)
        self.number_Re = self.rho_mix_kgm3 * self.vm_msec * self.d_hydr_m / self.mu_mix_pasec
        self.k_ratio_d = self.d_tube_out_m / self.d_cas_in_m
        # Friction coefficient
        # TODO дописать формулы для эксентричного положения трубы (сложные какие-то) - или не надо?
        self.Fca = (16 * (1 - self.k_ratio_d) ** 2 /
                    ((1 - self.k_ratio_d ** 4) / (1 - self.k_ratio_d ** 2) -
                     (1 - self.k_ratio_d ** 2) / math.log(1 / self.k_ratio_d)))
        if self.number_Re < 3000:  # laminar flow
            if self.concentric_annulus:
                self.friction_coeff = self.Fca / self.number_Re
            #else:
            #    self.friction_coeff = (1 / self.number_Re * 4 * (1 - self.k_ratio_d) ** 2 * (1 - self.k_ratio_d ** 2) /
            #                           self.o1o / math.sinh(self.eno) ** 4)
        else:  # turbulent flow
            if self.concentric_annulus:
                self.friction_coeff = float(sp.fsolve(self.__friction_coefficient_Gunn_Darling, 0.000005))
        # Hinze (1955)
        self.d_max_bubble_m = (self.const_k * (self.surface_tension_gl_Nm / self.rho_liq_kgm3) ** (0.6) *
                               (2 * self.vm_msec ** 3 / self.d_hydr_m * self.friction_coeff) ** (-0.4))

        self.d_crit_bubble_m = (0.4 * self.surface_tension_gl_Nm / (self.rho_liq_kgm3 -
                                                                    self.rho_gas_kgm3) / uc.g) ** 0.5

        # Dispersed bubble to slug flow transition
        self.vs_gas_dispbubble2slug_msec = 1.083 * self.vs_liq_msec + 0.796 * self.equation_part

        # Transition to Annular flow
        self.vs_gas_2annular_msec = 3.1 * self.equation_part

        #  определение режима потока
        if self.d_crit_bubble_m >= self.d_max_bubble_m:
            self.flow_pattern = 1  # dispersed bubble flow pattern
        else:
            if self.vs_gas_msec < self.vs_gas_bubble2slug_msec:
                self.flow_pattern = 0  # bubble flow pattern
            else:
                if self.vs_gas_msec > self.vs_gas_2annular_msec:
                    self.flow_pattern = 3  # annular flow
                else:
                    if self.vs_gas_msec < self.vs_gas_dispbubble2slug_msec:
                        self.flow_pattern = 1  # dispersed bubble flow pattern
                    else:
                        self.flow_pattern = 3  # slug flow pattern

annulus = flow_pattern_annulus_Caetano()
annulus.calc_pattern()


import uniflocpy.uPVT.PVT_fluids as PVT

annular = flow_pattern_annulus_Caetano()
fluid_flow = PVT.FluidFlow()

p_bar = 40
t_c = 60

fluid_flow.calc(p_bar, t_c)

annular.surface_tension_gl_Nm = fluid_flow.sigma_liq_Nm
annular.rho_liq_kgm3 = fluid_flow.rho_liq_kgm3
annular.rho_gas_kgm3 = fluid_flow.fl.rho_gas_kgm3
annular.rho_mix_kgm3 = fluid_flow.rhon_kgm3
annular.mu_mix_pasec = fluid_flow.mun_cP / 10 ** 3




annular.d_cas_in_m = 0.150
annular.d_tube_out_m = 0.100

annular.vs_gas_msec = fluid_flow.vsg_msec
annular.vs_liq_msec = fluid_flow.vsl_msec

annular.calc_pattern()