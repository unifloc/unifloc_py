import matplotlib.pyplot as plt
import numpy as np
import uMultiphaseFlow.Flow_pattern_map as Fluid

# Исходные данные
sigma = 8.41 * 10 ** (-3)
rho_liq = 761.7
rho_gas = 194.1
d_m = 0.1524
f_m = 1.49 * 10 ** (-2)
vel_gas = np.arange(0.01, 40, 0.005)  # иногда проскакивает пересечение, в этом случае нужно менять эпсилон и шаг
epsilon = 0.01
beta = 90
mu_liq = 0.68 * 10 ** (-3)
mu_gas = 1.9 * 10 ** (-5)

# Считаем скорости для графиков
vel_gas_bubble2slug = []
vel_gas_dispersedbubble2bubble = []
vel_gas_dispersedbubble2churn = []
vel_gas_churn2annular = []
vel_gas_slug2churn = []
vel_liq_bubble2slug = []
vel_liq_dispersedbubble2bubble = []
vel_liq_dispersedbubble2churn = []
vel_liq_slug2churn = []
"""
for vel in vel_gas:
    vel_liq_dispersedbubble2churn.append(Fluid.dispersedbubble2churn(vel)) 
    vel_liq_bubble2slug.append(Fluid.bubble2slug(vel, rho_liq, rho_gas, sigma))
    vel_liq_dispersedbubble2bubble.append(Fluid.dispersedbubble2bubble(vel, rho_liq, sigma, rho_gas, f_m, d_m))
    vel_liq_slug2churn.append(Fluid.slug2churn(vel, d_m, l_e))
"""
a_l = True
b_l = True
c_l = True
# TODO нужно сделать без сложной конструкции, а с помощью встроенных функций
for vel in vel_gas:
    a = Fluid.bubble2slug(vel, rho_liq, rho_gas, sigma, beta)
    b = Fluid.dispersedbubble2bubble(vel, rho_liq, sigma, rho_gas, f_m, d_m, beta)[0]
    c = Fluid.dispersedbubble2churn(vel)
    # d = slug2churn(vel, d_m, l_e)
    if abs(a - b) > epsilon and a_l == True:
        vel_liq_bubble2slug.append(a)
        vel_liq_dispersedbubble2bubble.append(b)
        vel_gas_bubble2slug.append(vel)
        vel_gas_dispersedbubble2bubble.append(vel)
    elif abs(b - c) > epsilon and b_l == True:
        a_l = False
        vel_liq_dispersedbubble2bubble.append(b)
        vel_gas_dispersedbubble2bubble.append(vel)
    else:
        b_l = False
        vel_liq_dispersedbubble2churn.append(c)
        vel_gas_dispersedbubble2churn.append(vel)

# Переход к кольцевой структуре
vel_liq_churn2annular_max = float(Fluid.dispersedbubble2bubble(Fluid.churn2annular(sigma, rho_liq, rho_gas), rho_liq,
                                                               sigma, rho_gas, f_m, d_m, beta)[0])
vel_liq_churn2annular = np.arange(0.01, vel_liq_churn2annular_max, 0.1)
for vel in vel_liq_churn2annular:
    vel_gas_churn2annular.append(Fluid.churn2annular(sigma, rho_liq, rho_gas))

# Строим графики
plt.figure(figsize=(7, 7))
plt.loglog(vel_gas_bubble2slug, vel_liq_bubble2slug, linewidth=3)
plt.loglog(vel_gas_dispersedbubble2churn, vel_liq_dispersedbubble2churn, linewidth=3)
plt.loglog(vel_gas_dispersedbubble2bubble, vel_liq_dispersedbubble2bubble, linewidth=3)
plt.loglog(vel_gas_churn2annular, vel_liq_churn2annular, linestyle='--', linewidth=3)
plt.loglog(vel_gas_slug2churn, vel_liq_slug2churn, linewidth=3)
plt.xlabel('Superficial Gas Velocity , m/s', size=18)
plt.ylabel('Superficial Liquid Velocity , m/s', size=18)
plt.xlim(np.min(vel_gas), np.max(vel_gas))
plt.ylim(0.01, np.max(vel_gas))
plt.title('Карта структур течения для B = {} °'.format(beta))
plt.text(np.min(vel_gas) + 0.003, 0.04, 'Bubbly', size=14)
plt.text(np.min(vel_gas_dispersedbubble2bubble) + 0.03, np.max(vel_liq_dispersedbubble2bubble) + 5, 'Dispersed Bubble',
         size=14)
plt.text(np.min(vel_gas_dispersedbubble2churn) + 8, 0.04, 'Annular', size=14)
plt.text(np.min(vel_gas_bubble2slug) + 0.06, 0.04, 'Slug', size=14)
plt.show()


beta = 0
mu_liq = 0.68 * 10 ** (-3)
mu_gas = 1.9 * 10 ** (-5)
rho_liq = 993
rho_gas = 1.14
d_m = 0.05
vel_gas = np.arange(0.1, 40, 0.5)
vel_liq_stratified2nonstratified = []
for vel in vel_gas:
    vel_liq_stratified2nonstratified.append(Fluid.stratified2nonstratified(rho_gas, rho_liq, vel, d_m, beta, mu_liq,
                                                                           mu_gas))
plt.loglog(vel_gas, vel_liq_stratified2nonstratified)
plt.show()

"""
x = Fluid.parameter_x(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq)
print(x)
y = Fluid.parameter_y(d_m, rho_liq, rho_gas, mu_gas, vel_gas, beta)
print(y)
h_l = Fluid.combined_momentum_equation(d_m, rho_liq, rho_gas, mu_liq, mu_gas, vel_gas, vel_liq, beta)
print(h_l)
"""
