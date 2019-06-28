import uniflocpy.uMultiphaseFlow.flow_pattern_annulus_Caetano as FPA
import uniflocpy.uTools.uconst as uc

class new_correlation_Marquez(object):
    def __init__(self):
        self.vs_liq_z_msec = 10
        self.v_infinite_z_msec = 5

        self.ratio = None
        self.M = None
        self.natural_sepatarion_d = None

    def calc(self):
        a = - 0.0093
        b = 57.758
        c = 34.4
        d = 1.308
        self.ratio = self.vs_liq_z_msec / self.v_infinite_z_msec
        self.M = - ((a * b + c * self.ratio ** d) / (b + self.ratio ** d))
        self.natural_sepatarion_d = 1 - (1 - ((1 + self.M) ** 272 + self.ratio ** 272)**(1/272) +
                                         self.ratio)
        return self.natural_sepatarion_d

try_sep = new_correlation_Marquez()
print(try_sep.calc())