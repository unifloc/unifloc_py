from pandas import read_csv
from os.path import dirname


class MotorBase(object):
    """
    Класс общающийся с базой данных ПЭД. Забирает коэффициенты
    отцифрованных кривых
    """

    def __init__(self, company: str, frequency_nom: int, d__mm: int) -> None:
        self.company = company
        self.frequency_nom = frequency_nom
        self.d__mm = d__mm
        self.nominal_power_kW = None
        self.motor_base = read_csv('{}/db/db_motor.csv'.format(
            dirname(__file__)), sep=';')

    @property
    def current_coefficient(self):
        return _get_coefficient(self.company, self.frequency_nom, self.d__mm,
                                parameter='I', motor_base=self.motor_base)

    @property
    def frequency_coefficient(self):
        return _get_coefficient(self.company, self.frequency_nom, self.d__mm,
                                parameter='f', motor_base=self.motor_base)

    @property
    def cos_coefficient(self):
        return _get_coefficient(self.company, self.frequency_nom, self.d__mm,
                                parameter='cos', motor_base=self.motor_base)

    @property
    def efficient_coefficient(self):
        return _get_coefficient(self.company, self.frequency_nom, self.d__mm,
                                parameter='eff', motor_base=self.motor_base)


def _get_coefficient(company, frequency_nom, d__mm, parameter, motor_base):

    current_motor = motor_base.loc[motor_base['Company'] == company]\
        .loc[motor_base['frequency_nom'] == frequency_nom]\
        .loc[motor_base['d_mm'] == d__mm]

    coefficients = []
    for i in range(7):
        coefficient = (current_motor['{}_{}'.format(parameter, i)].values[0])
        coefficients.append(coefficient)
    return coefficients


def _def_get_value(coefficient, motor_loading):

    value = coefficient[0] + coefficient[1] * motor_loading + \
            coefficient[2] * motor_loading**2 + coefficient[3] * \
            motor_loading**3 + coefficient[4] * motor_loading**4 + \
            coefficient[5] * motor_loading**5 + coefficient[6] * \
            motor_loading**6

    return value