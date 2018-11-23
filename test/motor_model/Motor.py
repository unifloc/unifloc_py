import math
from scipy.interpolate import interp1d
import numpy as np

import g_circuit
from MotorBase import MotorBase, _def_get_value


class Motor:
    def __init__(self, voltage_lin__V, current_lin__A, nom_efficient, nom_cos,
                 nom_slip, frequency):

        self.voltage_lin__V = voltage_lin__V
        self.current_lin__A = current_lin__A
        self.voltage_phase__V = voltage_lin__V/math.sqrt(3)
        self.current_phase__A = current_lin__A
        self.nom_efficient = nom_efficient
        self.nom_cos = nom_cos
        self.nom_slip = nom_slip
        self.frequency__Hz = frequency
        self.electric_power__kW = math.sqrt(3) * current_lin__A * \
                                  voltage_lin__V * nom_cos * 1e-3
        self.shaft_power__kW = self.electric_power__kW * nom_efficient

    def calc_shaft_power(self, voltage_work__V, current_work__A,
                         motor_lambda=2):

        """
        Расчет мощности ПЭД по построенной кривой (ток от загрузки)
        :param voltage_work__V: рабочее напряжение ПЭД(фаз), В
        :param current_work__A: рабочий ток ПЭД(фаз), A
        :param motor_lambda: отношение максимального момента к номинальному
        :return: Мощность ПЭД
        """
        r_2__Om, r_1__Om, x_k__Om = g_circuit.calc_resistances(
            self.shaft_power__kW,
            self.voltage_phase__V,
            self.nom_slip,
            self.nom_efficient,
            motor_lambda
        )

        # Расчитываем ток в таком промежутке загрузки ПЭД
        loading = np.arange(0.05, 0.95, 0.05)
        power_range = loading * self.shaft_power__kW

        current_range = []
        for power in power_range:
            _, _, _, I1__A, _, _ = g_circuit.motor_data_loading(
                power,
                self.shaft_power__kW,
                self.voltage_phase__V,
                self.nom_slip,
                self.nom_efficient,
                self.nom_cos,
                self.frequency__Hz,
                voltage_work__V,
                r_1__Om, r_2__Om, x_k__Om
            )
            current_range.append(I1__A)
        f = interp1d(current_range, power_range)
        motor_power__kW = f(current_work__A)

        return motor_power__kW

    def get_shaft_power_by_curve(self, motor_name, frequency__Hz, d__mm,
                                 current_work__A):
        """
        Расчет мощности ПЭД по отцифрованной кривой (ток от загрузки)
        :param motor_name: Производитель
        :param frequency__Hz: частота вращения
        :param d__mm: диаметр ПЭД
        :param current_work__A: Рабочий ток
        :return: Мощность двигателя
        """

        curve = MotorBase(motor_name, frequency__Hz, d__mm)

        # Расчитываем ток в таком промежутке загрузки ПЭД
        loading_range = np.arange(0.05, 0.95, 0.05)
        current_range = []
        for loading in loading_range:
            current = _def_get_value(curve.current_coefficient, loading) * \
                      self.current_phase__A / 1e3

            current_range.append(current)

        f = interp1d(current_range, loading_range)
        motor_loading = f(current_work__A)

        return motor_loading * self.shaft_power__kW







