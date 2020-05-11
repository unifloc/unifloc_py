
import math
import uniflocpy.uMultiphaseFlow.friction_Bratland as fr  # модуль для расчета коэффициента трения
import uniflocpy.uTools.uconst as uc
import numpy as np
import scipy.optimize as sp
const_g_m2sec = uc.g
import uniflocpy.uPVT.BlackOil_model as bom


class FrictionVBA():
    def __init__(self):
        self.d_m = None
        self.relative_roughness = None
        self.epsilon_m = None
        self.number_re = None

    def calc_f(self, number_re, epsilon_m, d_m):
        self.relative_roughness = epsilon_m / d_m
        self.d_m = d_m
        self.epsilon_m = epsilon_m
        self.number_re = number_re
        relative_roughness = self.relative_roughness

        if number_re == 0:
            return 0
        else:
            if number_re > 2000:
                f_n = (2 * np.log10(2 / 3.7 * relative_roughness -
                                   5.02 / number_re * np.log10(2 / 3.7 * relative_roughness + 13 / number_re))) ** -2
                result = 20
                i = 0
                while result > 0.001 or i < 19:
                    new_fn = (1.74 - 2 * np.log10(2 * relative_roughness + 18.7 / (number_re * f_n ** 0.5))) ** -2
                    result = np.abs(new_fn-f_n)
                    i = i + 1
                    f_n = new_fn
                return f_n
            else:
                return 64 / number_re


class Beggs_Brill_cor():
    """
    Класс для хранения данных и расчета градиента давления по методу Беггз и Брилл
    """
    def __init__(self, epsilon_friction_m=18.288 * 10 ** (-6), angle_grad=90, friction_type=0,
                 pains_corr_using=0, gravity_grad_coef=1, friction_grad_coef=1, acceleration_grad_coef=1,
                 acceleration_grad_using=0):
        """
        Инизиализация гидравлической корреляции

        :param epsilon_friction_m: шероховатость трубы, м
        :param angle_grad: угол наклона трубы от горизонтали, град
        """

        self.epsilon_friction_m = epsilon_friction_m
        self.gravity_grad_coef = gravity_grad_coef
        self.friction_grad_coef = friction_grad_coef
        self.acceleration_grad_coef = acceleration_grad_coef
        self.acceleration_grad_using = acceleration_grad_using


        self.d_m = None

        self.p_pa = None
        self.t_c = None

        self.vsl_msec = None  # приведенная скорость жидкости (3.10)
        self.vsg_msec = None  # приведенная скорость газа (3.11)
        self.vm_msec = None  # приведенная (общая) скорость смеси (3.12)

        self.liquid_content = None  # объемное содержание жидкости при отсутствии проскальзывания (3.8)

        self.rho_liq_kgm3 = None
        self.rho_gas_kgm3 = None
        self.mun_pas = None
        self.rhon_kgm3 = None
        self.sigma_liq_Nm = None

        self.val_number_Fr = None

        self.flow_regime = None


        self.number_Re = None
        self.friction_coefficient = None
        self.result_friction = None
        self.Ek = None
        self.rhos_kgm3 = None

        self.result_grad_pam = None

        # импорт модуля для расчета коэффициента трения
        if friction_type == 0:
            self.module_friction = FrictionVBA()
        elif friction_type == 1:
            self.module_friction = fr.Friction()
        else:
            self.module_friction = FrictionVBA()

        self.friction_grad_pam = None
        self.density_grad_pam = None
        self.acceleration_grad_pam = None
        self.friction_grad_part_percent = None
        self.density_grad_part_percent = None
        self.acceleration_grad_part_percent = None

        self.gas_fraction_real_d = None
        self.liquid_holdup_d = None

        self.fluid =bom.Fluid(activate_rus_cor=1)

    def calc_grad(self, p_bar, t_c):
        """
        Функция для расчета градиента давления по методу Беггз и Брилл

        :param p_bar: давление, бар
        :param t_c: температура, С
        :return: градиент давления, Па /м
        """
        self.p_pa = uc.bar2Pa(p_bar)
        self.t_c = t_c
        if self.p_pa <= 0:
            self.result_grad_pam = 0
            return 0
        else:
            betta_gas_d = 1 - self.liquid_content
            с1 = (2.2361 * np.exp(0.049 * mu_liquid_relative_d)) / (1 + 1.1002 * np.exp(0.049 * mu_liquid_relative_d)) - \
                8.17 * 10**(-3) * mu_liquid_relative_d ** 0.6 * (self.d_m / 0.015 - 1)

            с2 = (1 + 0.1082 * np.exp(0.049 * mu_liquid_relative_d)) / (1 + 1.1002 * np.exp(0.049 * mu_liquid_relative_d)) - \
                 (0.1006 - 2.52 * 10**(-3) *(mu_liquid_relative_d -1)) * (self.d_m / 0.015 - 1)

            fr_mix = v_mix ** 2 / (9.81 * self.d_m)

            real_gas_fraction_d = betta_gas_d / (c1 + c2 * fr_mix)

            rho_mix_real_kgm3 = self.rho_liq_kgm3 * (1 - real_gas_fraction_d) + self.rho_gas_kgm3 *  real_gas_fraction_d

            re_n = v_mix * self.d * rho_mix_real_kgm3 / self.mun_pas

            rho_sr_kgm3 = self.fluid.rho_oil_kgm3




            return self.result_grad_pam

