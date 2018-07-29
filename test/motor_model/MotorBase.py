from pandas import read_csv
import pandas as pd
from os.path import dirname


class Motor(object):

    def __init__(self, motor_base: pd.DataFrame.astype, company: str,
                 frequency_nom: int, d__mm: int) -> None:
        self.motor_base = motor_base
        self.company = company
        self.frequency_nom = frequency_nom
        self.d__mm = d__mm
        self.nominal_power_kW = None

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
    current_motor = motor_base.loc[motor_base['Company'] == company]\
        .loc[motor_base['frequency_nom'] == frequency_nom]\
        .loc[motor_base['d_mm'] == d__mm]

    coefficients = []
    for i in range(7):
        coefficient = (current_motor['{}_{}'.format(parameter, i)].values[0])
        coefficients.append(coefficient)
    return coefficients


# TODO: сделать чтобы база уже в классе лежала
motor_base =read_csv('{}/db/db_motor.csv'.format(dirname(__file__)), sep=';')
