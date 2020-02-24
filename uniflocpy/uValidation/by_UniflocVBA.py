import uniflocpy.uPVT.BlackOil_model as BlackOil_model

import uniflocpy.uValidation.python_api as python_api
import math
import uniflocpy.uTools.uconst as uc
p_bar = 250
p_atma = uc.bar2atm(p_bar)
t_c = 80
gamma_oil = 0.87
gamma_water = 1
gamma_gas = 1.2
rsb_m3m3 = 50
rp_m3m3 = 80
pb_cal_bar = 120
t_res_c = 100
bob_cal_m3m3 = 1.2
mu_oil_bubble_cp = 1

keywords = {"gamma_oil": gamma_oil, "gamma_gas": gamma_gas, "gamma_wat":gamma_water,
                                    "rsb_m3m3": rsb_m3m3, "t_res_c": t_res_c}

keywords_vba = {"t_c": t_c, "gamma_gas": gamma_gas,
                "gamma_oil": gamma_oil, "gamma_wat": gamma_water, "rsb_m3m3": rsb_m3m3, "tres_C": t_res_c} #проверено, улетают свойства газа (из-за z), st oil-gas, mu_wate

python_fluid = BlackOil_model.Fluid(**keywords)

uniflocvba = python_api.API('UniflocVBA_7.xlam')

def re_perc(y_fact,_y_calc):
    return abs((y_fact-_y_calc)/y_fact * 100)


python_fluid.calc(p_bar, t_c)
vba_value = uniflocvba.calc_PVT_bo_m3m3(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.b_oil_m3m3
print(f"b_oil_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_bw_m3m3(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.b_wat_m3m3
print(f"b_wat_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_bg_m3m3(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.b_gas_m3m3
print(f"b_gas_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_pb_atma(**keywords_vba)
python_value = uc.bar2atm(python_fluid.pb_bar)
print(f"pb_bar. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_rs_m3m3(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.rs_m3m3
print(f"rs_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_mu_gas_cP(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.mu_gas_cp
print(f"mu_gas_cp. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_mu_oil_cP(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.mu_oil_cp
print(f"mu_oil_cp. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_mu_wat_cP(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.mu_wat_cp
print(f"mu_wat_cp. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_z(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.z
print(f"z. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_salinity_ppm(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.s_ppm
print(f"s_ppm. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_SToilgas_Nm(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.sigma_oil_gas_Nm
print(f"sigma_oil_gas_Nm. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

vba_value = uniflocvba.calc_PVT_STwatgas_Nm(p_atma=p_atma, **keywords_vba)
python_value = python_fluid.sigma_wat_gas_Nm
print(f"sigma_wat_gas_Nm. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")





