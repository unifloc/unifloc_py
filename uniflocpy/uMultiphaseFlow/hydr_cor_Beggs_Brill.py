
"""
Олег Кобзарь
01.06.2019
Гидравлическая корреляция Беггз и Брилл в исполнении через класс
"""

import math
import uniflocpy.uMultiphaseFlow.friction_Bratland as fr  # модуль для расчета коэффициента трения
import uniflocpy.uTools.uconst as uc

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
        self.angle_rad = self.angle_grad * math.pi / 180

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
        self.friction_grad_part_percent = None
        self.density_grad_part_percent = None

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

        self.angle_correction_factor = (1 + self.correction_factor_c *
                                        ((math.sin(1.8 * self.angle_rad)) - (1 / 3) *
                                         (math.sin(1.8 * self.angle_rad)) ** 3))

        self.liquid_content_with_angle = self.liquid_content_with_zero_angle * self.angle_correction_factor

        if self.angle_grad > 0:
            self.liquid_content_with_Pains_cor = 0.924 * self.liquid_content_with_angle
        else:
            self.liquid_content_with_Pains_cor = 0.685 * self.liquid_content_with_angle

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

            number_Fr = self.val_number_Fr

            CL = self.liquid_content
            L1 = 316 * CL ** 0.302
            L2 = 0.0009252 * CL ** (-2.4684)
            L3 = 0.1 * CL ** (-1.4516)
            L4 = 0.5 * CL ** (-6.738)
            if (CL < 0.01 and number_Fr < L1) or (CL >= 0.01 and number_Fr < L2):
                # Segregated Flow - разделенный режим
                self.flow_regime = 0
            if (0.01 <= CL < 0.4 and L3 < number_Fr <= L1) or (CL >= 0.4 and L3 < number_Fr <= L4):
                # Intermittent Flow - прерывистый режим
                self.flow_regime = 1
            if (CL < 0.4 and number_Fr >= L1) or (CL >= 0.4 and number_Fr > L4):
                # Distributed Flow - распределенный режим
                self.flow_regime = 2
            if L2 <= number_Fr < L3 and CL >= 0.01:
                # Transition Flow - переходный режим
                self.flow_regime = 3

            if self.flow_regime != 3:
                self.__calc_hltetta__()
            else:
                self.flow_regime = 0
                self.__calc_hltetta__()
                hltetta_segr = self.liquid_content_with_Pains_cor
                self.flow_regime = 1
                self.__calc_hltetta__()
                hltetta_inter = self.liquid_content_with_Pains_cor
                A = (L3 - self.val_number_Fr) / (L3 - L2)
                B = 1 - A
                self.liquid_content_with_Pains_cor = (A * hltetta_segr + B * hltetta_inter)

            self.number_Re = self.rhon_kgm3 * self.vm_msec * self.d_m / self.mun_pas

            self.friction_coefficient = self.module_friction.calc_f(self.number_Re, self.epsilon_friction_m,
                                                                    self.d_m)

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

            self.rhos_kgm3 = (self.rho_liq_kgm3 * self.liquid_content_with_Pains_cor +
                              self.rho_gas_kgm3 * (1 - self.liquid_content_with_Pains_cor))

            self.result_grad_pam = ((self.result_friction * self.rhon_kgm3 * self.vm_msec ** 2 / 2 / self.d_m +
                                     self.rhos_kgm3 * const_g_m2sec * math.sin(self.angle_rad)) / (1 - self.Ek))

            self.friction_grad_pam = (self.result_friction * self.rhon_kgm3 * self.vm_msec ** 2 / 2 / self.d_m)

            self.density_grad_pam = self.rhos_kgm3 * const_g_m2sec * math.sin(self.angle_rad)

            self.friction_grad_part_percent = self.friction_grad_pam / (1 - self.Ek) / self.result_grad_pam * 100

            self.density_grad_part_percent = self.density_grad_pam / (1 - self.Ek) / self.result_grad_pam * 100

            return self.result_grad_pam



