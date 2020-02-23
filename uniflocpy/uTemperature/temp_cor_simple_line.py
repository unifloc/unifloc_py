"""
Кобзарь О.С. Хабибуллин Р.А.
Температурная корреляция в виде прямой линии
"""
import math
from scipy.optimize import fsolve
import uniflocpy.uTools.uconst as uc

const_g_m2sec = uc.g
pi = math.pi


class SimpleLineCor(): #TODO сделать учет инлинометрии, по абсолютной и по измеренной глубине
    def __init__(self):

        self.p_bar = None
        self.t_c = None

        self.t_bottomhole_c = None
        self.t_wellhead_c = None
        self.h_measured_wellhead_m = None
        self.h_measured_bottomhole_m = None

        self.grad_t_cm = None

    def calc_grad_t_cm(self, p_bar, t_c): #TODO проверить направление градиента, здесь только вверх
        self.p_bar = p_bar
        self.t_c = t_c
        self.grad_t_cm = (self.t_wellhead_c - self.t_bottomhole_c) /\
                         (self.h_measured_wellhead_m - self.h_measured_bottomhole_m)

