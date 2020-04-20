
"""
Олег Кобзарь
01.06.2019
Гидравлическая корреляция Беггз и Брилл в исполнении через класс
"""

import math
import uniflocpy.uMultiphaseFlow.friction_Bratland as fr  # модуль для расчета коэффициента трения
import uniflocpy.uTools.uconst as uc
import numpy as np
import scipy.optimize as sp
const_g_m2sec = uc.g

# TODO добавить учет расчета сверху вниз


class Beggs_Brill_cor():
    """
    Класс для хранения данных и расчета градиента давления по методу Беггз и Брилл
    """
    def __init__(self, epsilon_friction_m=18.288 * 10 ** (-6), angle_grad=90):
        """
        Инизиализация гидравлической корреляции

        :param epsilon_friction_m: шероховатость трубы, м
        :param angle_grad: угол наклона трубы от горизонтали, град
        """

        self.epsilon_friction_m = epsilon_friction_m

        self.angle_grad = angle_grad  # угол наклона ствола скважины от горизонтали
        self.angle_rad = None

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

        self.liquid_content_with_zero_angle = None
        self.correction_factor_betta = None
        self.angle_correction_factor = None
        self.liquid_content_with_angle = None

        self.number_Re = None
        self.friction_coefficient = None
        self.y = None
        self.s = None
        self.result_friction = None
        self.Ek = None
        self.rhos_kgm3 = None
        self.liquid_content_with_Pains_cor = None
        self.result_grad_pam = None

        # импорт модуля для расчета коэффициента трения
        self.module_friction = fr.Friction()

        self.friction_grad_pam = None
        self.density_grad_pam = None
        self.acceleration_grad_pam = None
        self.friction_grad_part_percent = None
        self.density_grad_part_percent = None
        self.acceleration_grad_part_percent = None

        self.L1 = None
        self.L2 = None
        self.L3 = None
        self.L4 = None

    def __calc_hltetta__(self):
        """
        Для расчета объемного содержания жидкости с учетом проскальзывания, режимов потока

        :param self: набор всех данных
        :return: ничего, последний расчет - объемное соодержание жидкости с поправкой на угол и поправкой Пэйна
        """
        if self.flow_regime == 0:
            # Segregated Flow - Расслоенный
            a = 0.98
            b = 0.4846
            c = 0.0868
        if self.flow_regime == 1:
            # Intermittent Flow - Прерывистый
            a = 0.845
            b = 0.5351
            c = 0.0173
        if self.flow_regime == 2:
            # Distributed Flow - Распределенный
            a = 1.065
            b = 0.5824
            c = 0.0609
        self.liquid_content_with_zero_angle = a * self.liquid_content ** b / self.val_number_Fr ** c

        self.Nlv = (self.vsl_msec * (self.rho_liq_kgm3 / (const_g_m2sec * self.sigma_liq_Nm)) ** (1 / 4))

        if self.flow_regime == 2:
            self.correction_factor_c = 0
        else:
            if self.flow_regime == 0:
                d = 0.011
                e = -3.768
                f = 3.539
                g = - 1.614
            if self.flow_regime == 1:
                d = 2.96
                e = 0.305
                f = -0.4473
                g = 0.0978

            result = ((1 - self.liquid_content) *
                      math.log(d * self.liquid_content ** e * self.Nlv ** f * self.val_number_Fr ** g))
            if result <= 0:
                self.correction_factor_c = 0
            else:
                self.correction_factor_c = result

        self.angle_rad = self.angle_grad * math.pi / 180 # TODO если скважина вертикальная, будет коррекция

        self.angle_correction_factor = (1 + self.correction_factor_c *
                                        ((math.sin(1.8 * self.angle_rad)) - (1 / 3) *
                                         (math.sin(1.8 * self.angle_rad)) ** 3))
        self.liquid_content_with_angle = self.liquid_content_with_zero_angle * self.angle_correction_factor

        if self.angle_grad > 0:  # uphill flow
            #self.liquid_content_with_Pains_cor = 0.924 * self.liquid_content_with_angle
            self.liquid_content_with_Pains_cor = 1 * self.liquid_content_with_angle
        else:  # downhill flow
            self.liquid_content_with_Pains_cor = 0.685 * self.liquid_content_with_angle

        if self.liquid_content_with_Pains_cor > 1:  # reality check
            self.liquid_content_with_Pains_cor = 1
        if self.liquid_content_with_Pains_cor < self.liquid_content:  #TODO check reality
            self.liquid_content_with_Pains_cor = self.liquid_content

    def determine_flow_pattern(self, number_Fr, liquid_content):
        """
        Определение режима течения
        :return:
        """
        self.val_number_Fr = number_Fr
        self.liquid_content = liquid_content

        self.L1 = 316 * self.liquid_content ** 0.302
        self.L2 = 0.0009252 * self.liquid_content ** (-2.4684)
        self.L3 = 0.1 * self.liquid_content ** (-1.4516)
        self.L4 = 0.5 * self.liquid_content ** (-6.738)

        if (self.liquid_content < 0.01 and number_Fr < self.L1) or (self.liquid_content >= 0.01 and number_Fr < self.L2):
            # Segregated Flow - разделенный режим
            self.flow_regime = 0
        else:
            if self.L2 <= number_Fr < self.L3 and self.liquid_content >= 0.01:
                # Transition Flow - переходный режим
                self.flow_regime = 3
            else:
                if (0.01 <= self.liquid_content < 0.4 and self.L3 < number_Fr <= self.L1) or (
                        self.liquid_content >= 0.4 and self.L3 < number_Fr <= self.L4):
                    # Intermittent Flow - прерывистый режим
                    self.flow_regime = 1
                if (self.liquid_content < 0.4 and number_Fr >= self.L1) or (self.liquid_content >= 0.4 and number_Fr > self.L4):
                    # Distributed Flow - распределенный режим
                    self.flow_regime = 2
        return self.flow_regime

    def determine_flow_pattern2(self, n_fr, lambda_l):

        if (n_fr >= 316 * lambda_l ** 0.302 or n_fr >= 0.5 * lambda_l ** -6.738):
            flow_pattern = 2
        else:
            if (n_fr <= 0.000925 * lambda_l ** -2.468):
                flow_pattern = 0
            else:
                if (n_fr <= 0.1 * lambda_l ** -1.452):
                    flow_pattern = 3
                else:
                    flow_pattern = 1
        return flow_pattern


    def __friction_factor__(self, number_re, relative_roughness):
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
            self.val_number_Fr = self.vm_msec ** 2 / const_g_m2sec / self.d_m  # (4.109)

            self.flow_regime = self.determine_flow_pattern(self.val_number_Fr, self.liquid_content)
            #self.flow_regime = self.determine_flow_pattern2(self.val_number_Fr, self.liquid_content)

            if self.flow_regime != 3:
                self.__calc_hltetta__()
            else:
                self.flow_regime = 0
                self.__calc_hltetta__()
                hltetta_segr = self.liquid_content_with_Pains_cor
                self.flow_regime = 1
                self.__calc_hltetta__()
                hltetta_inter = self.liquid_content_with_Pains_cor
                A = (self.L3 - self.val_number_Fr) / (self.L3 - self.L2)
                B = 1 - A
                self.liquid_content_with_Pains_cor = A * hltetta_segr + B * hltetta_inter
                self.flow_regime = 3

            self.number_Re = self.rhon_kgm3 * self.vm_msec * self.d_m / self.mun_pas

            self.friction_coefficient = self.module_friction.calc_f(self.number_Re, self.epsilon_friction_m,
                                                                    self.d_m)

            self.friction_coefficient = self.__friction_factor__(self.number_Re, self.epsilon_friction_m / self.d_m)

            self.y = self.liquid_content / self.liquid_content_with_Pains_cor ** 2
            if 1 < self.y < 1.2:
                self.s = math.log(2.2 * self.y - 1.2)
            elif 1 == self.y:
                self.s = 0
            else:
                lny = math.log(self.y)
                self.s = lny / (-0.0523 + 3.182 * lny - 0.8725 * lny ** 2 + 0.01853 * lny ** 4)

            self.result_friction = self.friction_coefficient * math.exp(self.s)

            self.Ek = self.vm_msec * self.vsg_msec * self.rhon_kgm3 / self.p_pa
            self.Ek = 0

            self.rhos_kgm3 = (self.rho_liq_kgm3 * self.liquid_content_with_Pains_cor +
                              self.rho_gas_kgm3 * (1 - self.liquid_content_with_Pains_cor))

            self.result_grad_pam = ((self.result_friction * self.rhon_kgm3 * self.vm_msec ** 2 / 2 / self.d_m +
                                     self.rhos_kgm3 * const_g_m2sec * math.sin(self.angle_rad)) / (1 - self.Ek))

            self.friction_grad_pam = (self.result_friction * self.rhon_kgm3 * self.vm_msec ** 2 / 2 / self.d_m)

            self.density_grad_pam = self.rhos_kgm3 * const_g_m2sec * math.sin(self.angle_rad)

            self.acceleration_grad_pam = self.result_grad_pam * self.Ek

            self.friction_grad_part_percent = self.friction_grad_pam / self.result_grad_pam * 100

            self.density_grad_part_percent = self.density_grad_pam / self.result_grad_pam * 100

            self.acceleration_grad_part_percent = self.acceleration_grad_pam / self.result_grad_pam * 100

            return self.result_grad_pam



