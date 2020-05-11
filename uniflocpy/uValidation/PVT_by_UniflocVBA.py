import sys
sys.path.append('../')


import uniflocpy.uTools.data_workflow as data_workflow
import pandas as pd
from uniflocpy.uTools import plotly_workflow
import uniflocpy.uPVT.BlackOil_model as BlackOil_model
import uniflocpy.uValidation.python_api as python_api
import uniflocpy.uTools.uconst as uc

def create_result_df_from_vba_output(vba_result, p_bar, t_c):
    result_dict = {}
    for key, value in zip(vba_result[1],vba_result[0]):
        result_dict[key] = value
    result_dict['p_bar'] = p_bar
    result_dict['t_c'] = t_c
    this_df = pd.DataFrame(result_dict, index = [0])
    this_df['pb_bar'] = uc.atm2bar(this_df['pb_atma'])
    return this_df


#
#для надстройки из ветки dev22_2
#
uniflocvba = python_api.API('E:\\Git\\unifloc_vba\\UniflocVBA_7.xlam')


gamma_oil = 0.87
gamma_water = 1
gamma_gas = 1.2
rsb_m3m3 = 100
t_res_c = 120
t_c = 50
p_bar = 100
#rp_m3m3 = 80
#pb_cal_bar = 120
#bob_cal_m3m3 = 1.2
#mu_oil_bubble_cp = 1

keywords_python = {"gamma_oil": gamma_oil, "gamma_gas": gamma_gas, "gamma_wat":gamma_water,
                                    "rsb_m3m3": rsb_m3m3, "t_res_c": t_res_c}

keywords_vba = {"t_C": t_c, "gamma_gas": gamma_gas,
                "gamma_oil": gamma_oil, "gamma_wat": gamma_water, "rsb_m3m3": rsb_m3m3, "tres_C": t_res_c} #проверено, улетают свойства газа (из-за z), st oil-gas, mu_wate


blackoil_option = BlackOil_model.BlackOil_option()
blackoil_option.b_wat_cor_number = 1
blackoil_option.mu_wat_cor_number = 1
blackoil_option.rho_wat_cor_number = 1
blackoil_option.z_cor_number = 1
blackoil_option.pseudocritical_temperature_cor_number = 1
blackoil_option.pseudocritical_pressure_cor_number = 1
blackoil_option.rho_gas_cor_number = 1
blackoil_option.b_gas_cor_number = 1
blackoil_option.mu_dead_oil_cor_number = 2
blackoil_option.sigma_oil_gas_cor_number = 2
blackoil_option.sigma_wat_gas_cor_number = 1


python_fluid = BlackOil_model.Fluid(**keywords_python, option = blackoil_option)


BlackOil_model_db = data_workflow.Data()
BlackOil_model_db.clear_data()
vba_result_df = None
for p_bar in range(2, 400, 5):
    python_fluid.calc(p_bar, t_c)
    p_atm = uc.bar2atm(p_bar)
    BlackOil_model_db.get_data(python_fluid, object_name='python_fluid')
    vba_result = uniflocvba.PVT_all_pvt(p_atm, **keywords_vba)
    this_vba_result_df = create_result_df_from_vba_output(vba_result, p_bar, t_c)
    try:
        vba_result_df = vba_result_df.append(this_vba_result_df)
    except:
        vba_result_df = this_vba_result_df.copy()


result_unifloc_python = BlackOil_model_db.get_data_as_df()
result_unifloc_python.index = result_unifloc_python['python_fluid.p_bar']
result_unifloc_python.index.name = 'p_bar'

vba_result_df.index = result_unifloc_python.index
vba_result_df.index.name = 'p_bar'
vba_result_df = vba_result_df.add_prefix('vba_fluid.')
all_result = result_unifloc_python.join(vba_result_df, rsuffix=' (uniflocvba)')


banches_with_patterns = {'Газосодержание': [['rs'], ['cor_number', 'cal']],
                         'Коэффициент сверхсжимаемости': [['z'], ['cor_number', 'cal', 'mpa', 'tpb']],
                         'Давление насыщения': [['pb'], ['cor_number', 'cal', 'mpa', 'tpb']],

                        'Вязкость нефти': [['mu', 'oil'], ['cor_number', 'cal']],
                        'Вязкость газа': [['mu', 'gas'], ['cor_number', 'cal']],
                        'Вязкость воды': [['mu', 'wat'], ['cor_number', 'cal']],

                        'Плотность нефти': [['rho', 'oil'], ['cor_number', 'cal']],
                        'Плотность газа': [['rho', 'gas'], ['cor_number', 'cal']],
                        'Плотность воды': [['rho', 'wat'], ['cor_number', 'cal']],

                         'Объемный коэффициент нефти': [[['b', 'oil', 'm3m3'],['bo','m3m3']], ['bob','cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         'Объемный коэффициент газа': [['b', 'g', 'm3m3'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         'Объемный коэффициент воды': [['b', 'wat', 'm3m3'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],

                         "Теплоемкость нефти": [[["heatcap", 'oil'], ["JkgC",'o_']], ['fl.','cor_number', 'cal']],
                        "Теплоемкость газа": [[["heatcap", 'gas'], ["JkgC",'g_']], ['fl.','cor_number', 'cal']],
                        "Теплоемкость воды": [[["heatcap", 'wat'], ["JkgC",'w_']], ['fl.','cor_number', 'cal']],

                         'Вязкость': [['mu'], ['cor_number', 'cal']],
                         'Плотность': [['rho'], ['cor_number', 'cal']],
                         'Объемный коэффициент': [ ['b','m3m3'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         'Поверхностное натяжение': [['sigma'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         'Коэффициент сжимаемости': [ ['comp'], ['cor_number', 'cal', 'mpa', 'tpb', 'rs']],
                         "Теплоемкость": [[["heatcap"], ["JkgC"]], ['fl.','cor_number', 'cal']]}

banches = plotly_workflow.create_banches_from_pattern(all_result, banches_with_patterns)

plotly_workflow.create_report_html(all_result, banches, 'PVT_by_UniflocVBA.html',
                                   shared_xaxes='all', cols=3, one_plot_height=300,
                                   verical_spacing=0.05,
                                   title_text=f"Сравнение PVT свойств UniflocVBA и UniflocPy, по оси x давление в барах"
                                              f"\n исходные данные: \n gamma_oil={gamma_oil}, gamma_water={gamma_water}, "
                                              f"gamma_gas={gamma_gas}, rsb_m3m3={rsb_m3m3}, "
                                              f"t_res_c ={t_res_c}, t_c={t_c}")
