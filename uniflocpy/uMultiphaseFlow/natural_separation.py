"""
Модуль расчета естественной сепарации
будет постепенной пополняться новыми методами
Кобзарь О.С. 16.07.2019
"""

class new_correlation_Marquez(object):
    """
    Новая корреляция Marquez для расчета естественной сепарации
    """
    def __init__(self):
        self.vs_liq_z_msec = 10
        self.v_infinite_z_msec = 8

        self.ratio = None
        self.M = None
        self.natural_sepatarion_d = None

    def calc(self):
        """
        Расчет естественной сепарации

        :return: коэффициент естественной сепарации (значение в пределах от 0 до 1)
        """
        a = - 0.0093
        b = 57.758
        c = 34.4
        d = 1.308
        self.ratio = self.vs_liq_z_msec / self.v_infinite_z_msec
        self.M = ((a * b + c * self.ratio ** d) / (b + self.ratio ** d))
        self.natural_sepatarion_d = ((1 + self.M) ** 272 + self.ratio ** 272) ** (1 / 272) - self.ratio
        return self.natural_sepatarion_d

