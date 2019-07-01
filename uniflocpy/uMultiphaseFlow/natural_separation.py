import uniflocpy.uMultiphaseFlow.flow_pattern_annulus_Caetano as FPA
import uniflocpy.uTools.uconst as uc


class new_correlation_Marquez(object):
    def __init__(self):
        self.vs_liq_z_msec = 10
        self.v_infinite_z_msec = 8

        self.ratio = None
        self.M = None
        self.natural_sepatarion_d = None

    def calc(self):
        a = - 0.0093
        b = 57.758
        c = 34.4
        d = 1.308
        ST = 273
        backst = 1 / 273
        self.ratio = self.vs_liq_z_msec / self.v_infinite_z_msec
        self.M =  ((a * b + c * self.ratio ** d) / (b + self.ratio ** d))
        self.natural_sepatarion_d = 1 - (1 - ((1 + self.M) ** 272 + self.ratio ** 272)**(1/272) +
                                         self.ratio)
        """if self.M > 13:
            self.natural_separation = 0
            return self.natural_separation
        else:
            self.natural_separation  = ((1 + (a * b + c * self.M ** d) / (b + self.M ** d)) ** ST + self.M ** ST) ** backst - self.M
            return self.natural_separation"""
        return self.natural_sepatarion_d

import uniflocpy.uMultiphaseFlow.flow_pattern_annulus_Caetano as FPA
import uniflocpy.uPVT.PVT_fluids as PVT
import uniflocpy.uTools.data_workflow as tool

annular = FPA.flow_pattern_annulus_Caetano()
fluid_flow = PVT.FluidFlow()
flow_data = tool.Data()
pattern_data = tool.Data()

p_bar = 40
t_c = 60
fluid_flow.qliq_on_surface_m3day = 0.1
fluid_flow.calc(p_bar, t_c)

annular.surface_tension_gl_Nm = fluid_flow.sigma_liq_Nm
annular.rho_liq_kgm3 = fluid_flow.rho_liq_kgm3
annular.rho_gas_kgm3 = fluid_flow.fl.rho_gas_kgm3
annular.rho_mix_kgm3 = fluid_flow.rhon_kgm3
annular.mu_mix_pasec = fluid_flow.mun_cP / 10 ** 3

annular.d_cas_in_m = 0.140
annular.d_tube_out_m = 0.100


Ap = uc.pi / 4 * (annular.d_cas_in_m ** 2 - annular.d_tube_out_m ** 2)

vs_gas_msec = fluid_flow.qgas_m3day / Ap / 86400
vs_liq_msec = fluid_flow.qliq_m3day / Ap / 86400

annular.calc_pattern(vs_liq_msec, vs_gas_msec)

print(annular.flow_pattern_name)

separation = new_correlation_Marquez()
separation.v_infinite_z_msec = annular.v_infinite_z_msec
print('v_infinite_z_msec')
print(annular.v_infinite_z_msec)
separation.vs_liq_z_msec = annular.vs_liq_msec
print('vs_liq_msec')
print(annular.vs_liq_msec)

value = separation.calc()

print(value)

