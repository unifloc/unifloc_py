import pandas as pd
from pandas import read_csv


class Motor(object):

    def __init__(self, motor_base, company, frequency_nom, d__mm):
        self.motor_base = motor_base
        self.company = company
        self.frequency_nom = frequency_nom
        self.d__mm = d__mm

    @property
    def current_coefficient(self):
        return get_coefficient(self.motor_base, self.company,
                               self.frequency_nom, self.d__mm, parameter='I')

    @property
    def frequency_coefficient(self):
        return get_coefficient(self.motor_base, self.company,
                               self.frequency_nom, self.d__mm, parameter='f')

    @property
    def cos_coefficient(self):
        return get_coefficient(self.motor_base, self.company,
                               self.frequency_nom, self.d__mm, parameter='cos')

    @property
    def efficient_coefficient(self):
        return get_coefficient(self.motor_base, self.company,
                               self.frequency_nom, self.d__mm, parameter='eff')


def get_coefficient(motor_base, company, frequency_nom, d__mm, parameter):
    motor = motor_base.loc[motor_base['Company'] == company]\
        .loc[motor_base['frequency_nom'] == frequency_nom]\
        .loc[motor_base['d_mm'] == d__mm]

    coefficients = []
    for i in range(7):
        coefficient = (motor['{}_{}'.format(parameter, i)].values[0])
        coefficients.append(coefficient)
    return coefficients


# TODO: сделать чтобы база уже в классе лежала
motor_base1 = read_csv('/Users/poleshkomi/Desktop/Projects/Unifloc/test'
                      '/Motor model/db/db_motor.csv', sep=';')
