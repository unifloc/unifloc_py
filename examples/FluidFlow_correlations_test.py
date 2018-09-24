"""
2018 August 09 14:26
"""
import uMultiphaseFlow.FluidFlow_correlations as Flow
import matplotlib.pyplot as plt
import numpy as np

"""
sigma_liq_Nm = 8.41 * 10 ** (-3)
rho_liq_kgm3 = 761.7
rho_gas_kgm3 = 94.1
d_m = 0.1524
f = 1.49 * 10 ** (-2)
vel_liq_super_ms = np.arange(0.01, 10, 0.05)
vel_gas_a = []
vel_gas_b = []
for vel in vel_liq_super_ms:
    vel_gas_b.append(Flow.unf_velocity_bubble2slug(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, vel))
    vel_gas_a.append(Flow.unf_velocity_dispersed2bubble(sigma_liq_Nm, rho_liq_kgm3, rho_gas_kgm3, f, vel, d_m))
plt.loglog(vel_gas_b, vel_liq_super_ms)
plt.loglog(vel_gas_a, vel_liq_super_ms)
plt.grid()
plt.show()
"""

l = 1000
p1 = 10
t1 = 20
t2 = 60
d_m = 0.089
q_liq_m3d = 100
theta = 90
e = 18.288 * 10 ** (-6)
wct = 0.5
sigma_liq_Nm = 8.41 * 10 ** (-3)
l, p = Flow.pressure_drop_BeggsBrill(l, p1, t1, t2, d_m, q_liq_m3d, sigma_liq_Nm, theta, e, wct)
plt.plot(p, l, label='КРД по Беггсу-Бриллу')
plt.xlim(0)
plt.gca().invert_yaxis()
plt.legend()
plt.grid()
plt.show()
