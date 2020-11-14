from abc import ABC
from abc import abstractmethod

Root = 'Root'

class HE2_ABC_Fluid(ABC):
    @abstractmethod
    def calc(self, P_bar, T_C):
        """
        Метод пересчитывает все свойства флюида, в заданые PT-условия
        :param P_bar: Давление
        :param T_C: и температура для которых надо посчитать свойства флюида
        :return None - расчитаные свойства лежат в полях экземпляра
        """
        pass

class HE2_ABC_GraphEdge(ABC):
    """
    Любая дуга графа сети труб, должна уметь считать на себе потерю давления.
    Для этого предлагается избыточный интерфейс, три метода.
    2. perform_calc_forward() заданы p,t в начале координат трубы, возвращается p,t в конце координат
    3. perform_calc_backward() заданы p,t в конце координат трубы, возвращается p,t в начале координат
    1. perform_calc используется unifloc-style (00, 01, 10, 11) для указания направления потока и расчета
    """
    @abstractmethod
    def perform_calc(self, P_bar, T_C, X_kgsec, unifloc_direction):
        """
        :param P_bar: Известные давление
        :param T_C: и температура на одном из концов трубы/объекта. На каком именно - см. unifloc_direction
        :param X_kgsec: Массовый поток через трубу/объект. В этом методе знак потока игнорируется
        :param unifloc_direction: Допустимы значения 00, 01, 10, 11. Первая цифра задает направление расчета, вдоль оси координат или против,
        вторая цифра задает направление потока, таким же образом.
        :return (p, t) - кортеж с расчитаными давлением и температурой
        """
        pass

    @abstractmethod
    def perform_calc_forward(self, P_bar, T_C, X_kgsec):
        """
        :param P_bar: Давление
        :param T_C: и температура в нулевой координате трубы/объекта, в начале первого сегмента
        :param X_kgsec: Массовый поток через трубы. Отрицательная величина значит что поток течет против направления оси координат
        :return (p, t) - кортеж с расчитаными давлением и температурой в конце трубы/объекта, в конце последнего сегмента
        """
        pass

    @abstractmethod
    def perform_calc_backward(self, P_bar, T_C, X_kgsec):
        """
        :param P_bar: Давление
        :param T_C: и температура в конце трубы/объекта, в конце последнего сегмента
        :param X_kgsec: Массовый поток через трубы. Отрицательная величина значит что поток течет против направления оси координат
        :return (p, t) - кортеж с расчитаными давлением и температурой в нулевой координате трубы/объекта, в начале первого сегмента
        """
        pass

class HE2_ABC_GraphVertex(ABC):
    pass

class HE2_ABC_Graph(ABC):
    pass

class HE2_ABC_PipeSegment(ABC):
    pass

class HE2_ABC_Pipeline(ABC):
    pass

