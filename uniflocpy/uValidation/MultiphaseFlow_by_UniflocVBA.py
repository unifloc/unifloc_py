import sys
sys.path.append('../')

import uniflocpy.uTools.data_workflow as data_workflow
import pandas as pd
from uniflocpy.uTools import plotly_workflow
import uniflocpy.uPVT.BlackOil_model as BlackOil_model
import uniflocpy.uValidation.python_api as python_api
import uniflocpy.uTools.uconst as uc
import uniflocpy.uPVT.PVT_fluids as PVT_fluids

def create_result_df_from_vba_output(vba_result, p_bar, t_c):
    result_dict = {}
    for key, value in zip(vba_result[1],vba_result[0]):
        result_dict[key] = value
    result_dict['p_bar'] = p_bar
    result_dict['t_c'] = t_c
    this_df = pd.DataFrame(result_dict, index = [0])
    return this_df

uniflocvba = python_api.API('E:\\Git\\unifloc_vba\\UniflocVBA_7.xlam')

gamma_oil = 0.86
gamma_water = 1
gamma_gas = 1.45 / 1.293
rsb_m3m3 = 56
t_res_c = 40
t_c = 30
p_bar = 100
q_liq_sm3day = 100
watercut_perc = 0


keywords_python = {"gamma_oil": gamma_oil, "gamma_gas": gamma_gas, "gamma_wat":gamma_water,
                                    "rsb_m3m3": rsb_m3m3, "t_res_c": t_res_c}

keywords_vba = {"qliq_sm3day":q_liq_sm3day, "fw_perc":watercut_perc,
                "t_C": t_c, "gamma_gas": gamma_gas,
                "gamma_oil": gamma_oil, "gamma_wat": gamma_water, "rsb_m3m3": rsb_m3m3, "tres_C": t_res_c}

blackoil_option = BlackOil_model.BlackOil_option()
blackoil_option.b_wat_cor_number = 1
blackoil_option.mu_wat_cor_number = 1
blackoil_option.rho_wat_cor_number = 1
blackoil_option.z_cor_number = 1
blackoil_option.pseudocritical_temperature_cor_number = 1
blackoil_option.pseudocritical_pressure_cor_number = 1
blackoil_option.rho_gas_cor_number = 1
blackoil_option.b_gas_cor_number = 1 - 1
blackoil_option.mu_dead_oil_cor_number = 2
blackoil_option.sigma_oil_gas_cor_number = 2
blackoil_option.sigma_wat_gas_cor_number = 1

python_fluid = BlackOil_model.Fluid(**keywords_python, option = blackoil_option)

multiphase_flow = PVT_fluids.FluidFlow(python_fluid)
multiphase_flow.qliq_on_surface_m3day = q_liq_sm3day
multiphase_flow.fw_on_surface_perc  =watercut_perc

python_flow_model_db = data_workflow.Data()
python_flow_model_db.clear_data()
vba_result_df = None
for p_bar in range(1, 400, 10):
    multiphase_flow.calc(p_bar, t_c)
    p_atm = uc.bar2atm(p_bar)
    python_flow_model_db.get_data(multiphase_flow, object_name='python_flow_model')
    vba_result = uniflocvba.MF_all_mf(p_atma = p_atm, **keywords_vba)
    this_vba_result_df = create_result_df_from_vba_output(vba_result, p_bar, t_c)
    try:
        vba_result_df = vba_result_df.append(this_vba_result_df)
    except:
        vba_result_df = this_vba_result_df.copy()


result_unifloc_python = python_flow_model_db.get_data_as_df()
result_unifloc_python.index = result_unifloc_python['python_flow_model.p_bar']
result_unifloc_python.index.name = 'p_bar'

vba_result_df.index = result_unifloc_python.index
vba_result_df.index.name = 'p_bar'
vba_result_df['mn_kgsec'] = vba_result_df['mw_kgsec'] + vba_result_df['mg_kgsec'] + vba_result_df['mo_kgsec']
vba_result_df['liquid_content_d'] = 1 - vba_result_df['gas_fraction_d']
vba_result_df = vba_result_df.add_prefix('vba_flow_model.')
all_result = result_unifloc_python.join(vba_result_df)


banches_with_patterns = {
                        'Объемный расход нефти': [["q", 'oil'], ['cor_number', 'cal']],
                        'Объемный расход газа': [[["q_gas_rc_m3day"], ["qgas_m3day"]], ['cor_number', 'cal']],
                        'Объемный расход газа (все типы)': [["q", 'gas'], ['cor_number', 'cal']],
                        'Объемный расход воды': [["q", 'wat'], ['mass_fraction','cor_number', 'cal']],

                        'Вязкость ГЖС': [[['flow_model.mu_mix'],['flow_model.mun']], ['cor_number', 'cal']],
                        'Вязкость жидкости': [['flow_model.mu_liq'], ['cor_number', 'cal']],
                        'Вязкости': [['flow_model.mu'], ['cor_number', 'cal']],

                        'Плотность ГЖС': [[['rho_mix'],['rhon']], ['cor_number', 'cal']],
                        'Плотность жидкости': [['rho_liq'], ['cor_number', 'cal']],
                        'Плотности': [['rho'], ['cor_number', 'cal']],

                        'Теплоемкость ГЖС': [['heatcap'], ['cor_number', 'cal']],
                        'Поверхностное натяжение ГЖС': [['sigma'], ['cor_number', 'cal']],

                        'Массовый расход нефти': [[['mo','kgsec'],['oil','kgsec']], ['cor_number', 'cal']],
                        'Массовый расход газа': [[['mg','kgsec'],['gas','kgsec']], ['cor_number', 'cal']],
                        'Массовый расход воды': [[['mw','kgsec'],['wat','kgsec']], ['cor_number', 'cal']],
                        'Массовый расход жидкости': [['liq', 'kgsec'], ['cor_number', 'cal']],
                        'Массовый расход смеси': [[['mn', 'kgsec'], ['flowraten', 'kgsec']], ['cor_number', 'cal']],

                        'Массовый расходы': [['kgsec'], ['cor_number', 'cal']],

                        'Объемный расход ГЖС': [['mix', 'm3day'], ['cor_number', 'cal']],
                        'Объемный расход жидкости': [['liq', 'm3day'], ['cor_number', 'cal']],

                        'Объемный расходы': [['q'], ['cor_number', 'cal']],
                        'Доля жидкости в потоке ГЖС': [['liquid_content'], ['cor_number', 'cal']],
                        'Доля газа в потоке ГЖС': [['gas_fraction'], ['cor_number', 'cal']],


                        'Газосодержание': [['rs'], ['cor_number', 'cal']],
                         'Коэффициент сверхсжимаемости': [['z'], ['cor_number', 'cal', 'mpa', 'tpb']],
                         'Давление насыщения': [['pb'], ['cor_number', 'cal', 'mpa', 'tpb']],



                         'Вязкость': [['mu'], ['cor_number', 'cal']],
                         'Плотность': [['rho'], ['cor_number', 'cal']],
                         'Объемный коэффициент': [ ['b','m3m3'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         'Поверхностное натяжение': [['sigma'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         'Коэффициент сжимаемости': [ ['comp'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         "Теплоемкость": [[["heatcap"], ["JkgC"]], ['fl.','cor_number', 'cal']]}

banches = plotly_workflow.create_banches_from_pattern(all_result, banches_with_patterns)

plotly_workflow.create_report_html(all_result, banches, 'MultiphaseFlow_by_UniflocVBA.html',
                                   shared_xaxes='all', cols=3, one_plot_height=300,
                                   verical_spacing=None,
                                   title_text=f"Сравнение MF UniflocVBA, ось x - бары"
                                              f"\n исходные данные: \n gamma_oil: {gamma_oil}, gamma_water: {gamma_water}, "
                                              f"gamma_gas: {gamma_gas}, rsb_m3m3: {rsb_m3m3}, "
                                              f"t_res_c: {t_res_c}, t_c={t_c}, "
                                              f"q_liq_sm3day: {q_liq_sm3day}, "
                                              f"watercut_perc: {watercut_perc}")
