"""
tests for PVT_correlations functions
"""
import uPVT.PVT_correlations as PVTfunc
import matplotlib.pyplot as plt
import numpy as np



def prep_plot(func, tset, goset, ggset, rsbset, plot_title, plot_xlab, plot_ylab, plt_):
    """
    функция для автоматизации построения графиков по давлению насыщения
    взята из блокнота
    строит графики для набора температур, плотности нефти, плотности газа
    :param func: функция, которая будет отображаться
    :param tset: набор значений температур
    :param goset: набор значений плотности нефти
    :param ggset: набор значений плотности газа
    :param rsbset: набор значений газосодержания
    :param plot_title: заголовок
    :param plot_xlab: подпись по оси х
    :param plot_ylab: подпись по оси y
    :param plt_: объект для рисования
    :return: готовит заготовку для рисования
    """
    for t in tset:
        for gg in ggset:
            for go in goset:
                pb_set=[]
                for rsb in rsbset:
                    pb_set.append(func(rsb,t_K = t,gamma_gas = gg,gamma_oil = go))
                plt_.plot(rsbset, pb_set, label='t = %1.0f $ ^{\circ}\mathrm{K}$'%t +
                                                 ' $\gamma_g$ = %1.2f'%gg +
                                                 ' $\gamma_o$ = %1.2f'%go)
    plt_.title(plot_title)
    plt_.ylabel(plot_ylab, color = 'black')
    plt_.xlabel(plot_xlab, color = 'black')
    plt_.legend()


def pb_Standing_test():
    rsb_max = 200
    rsbset =  np.arange(1, rsb_max, 2)
    plt.figure(figsize=(15, 8))
    f = PVTfunc.unf_pb_Standing_MPaa
    # рисуем первый график
    plt.subplot(221)
    goset = [0.75, 0.8, 0.85, 0.9, 0.95]
    ggset = [0.7]
    tset = [300]
    prep_plot(f, tset, goset, ggset, rsbset,
              'Давление насыщения от газосодержания',
              '$R_{sb}, м^3/м^3$',
              '$P_b, MPa$', plt)
    # рисуем второй график
    plt.subplot(222)
    goset = [0.75, 0.8, 0.85, 0.9, 0.95]
    ggset = [0.8]
    tset = [330]
    prep_plot(f,  tset, goset, ggset, rsbset,
              'Давление насыщения от газосодержания',
              '$R_{sb}, м^3/м^3$',
              '$P_b, MPa$', plt)
    # рисуем третий график
    plt.subplot(223)
    goset = [0.75, 0.8, 0.85, 0.9, 0.95]
    ggset = [0.9]
    tset = [350]
    prep_plot(f,  tset, goset, ggset, rsbset,
              'Давление насыщения от газосодержания',
              '$R_{sb}, м^3/м^3$',
              '$P_b, MPa$', plt)
    plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35)
    # рисуем четвертый график
    plt.subplot(224)
    goset = [0.75, 0.8, 0.85, 0.9, 0.95]
    ggset = [0.9]
    tset = [380]
    prep_plot(f,  tset, goset, ggset, rsbset,
              'Давление насыщения от газосодержания',
              '$R_{sb}, м^3/м^3$',
              '$P_b, MPa$', plt)
    plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35)
    # рисуем все
    plt.show()



pb_Standing_test()
