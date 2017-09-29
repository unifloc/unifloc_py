"""
описание констант и методов конвертации единиц измерения
"""
import scipy.constants as const

g = const.g   # gravity
pi = const.pi

class AbstractUnit():
    """
    Абстрактный класс для определения свойств размерной единицы измерения
    Не надо создавать объекты этого класса в явнов виде
    для опредения разрмерности надо задать словать _unit с названиями размерностей и переводных коэффциентов
    """
    def __init__(self, val):
        self._value = val
#        self._unit = {}
       
    def __getitem__(self, key):
        # если значение или тип ключа некорректны, list выбросит исключение
        return  self._value / self._unit[key]
        
    def __getattr__(self, name):
        return  self._value /  self._unit[name] 
    
    def __str__(self):
        return str(self._value) + ' atm'

    def __setitem__(self, key, item):
        self._value = item * self._unit[key] 
        
    def __setattr__(self, key, item):
        if key in _unit:
            print("device test")
#        else:
#            super(MyTest, self).__setattr__(name, value)
            # in python3+ you can omit the arguments to super:
            #super().__setattr__(name, value)

class Pressure(AbstractUnit):
    
    def __init__(self, val):
        super().__init__(val)
        self._unit = {'Pa':1/const.bar, 'psi':const.psi/const.bar, 'bar':1, 'atm': const.atm/const.bar, 'MPa':1000000/const.bar}


class Length(AbstractUnit):
    
    def __init__(self, val):
        super().__init__(val)
        self._unit = {'m':1, 'ft':const.foot, 'mm':1/1000, 'inch': const.inch, 'km':1000}

    
#    @property
#    def atm(self):
#        return self
    
#    @atm.setter
#    def atm(self, value):
#        return float.__new__(Pressure, value)
    
#    @property
#    def psi(self):
#        return  bar2psi(self._val)
    
#    @psi.setter
#    def psi(self,value):
#        self._val = psi2bar(value)
    

def psi2pa(value=1):
    return value * const.psi


def bar2psi(value=1):
    return value * const.bar / const.psi

def psi2bar(value=1):
    return value / const.bar * const.psi


def bar2atm(value=1):
    return value * const.bar / const.atm


def atm2bar(value=1):
    return value * const.atm / const.bar


def c2f(value):
    return const.convert_temperature(value, 'C', 'F')


def f2c(value):
    return const.convert_temperature(value, 'F', 'K')


def c2k(value):
    return const.convert_temperature(value, 'C', 'K')


def m3_2_bbl(value=1):
    return value / const.barrel


def bbl2m3(value=1):
    return value / const.barrel


def m3m3_to_m3t(value, Gamma=1):
    pass
