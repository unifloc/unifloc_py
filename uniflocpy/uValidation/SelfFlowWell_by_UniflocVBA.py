import sys
sys.path.append('../')

import uniflocpy.uWell.deviation_survey as dev_sev
import uniflocpy.uTools.data_workflow as utool
import uniflocpy.uTools.uconst as uconst
import uniflocpy.uWell.uPipe as Pipe
import uniflocpy.uWell.Self_flow_well as self_flow_well
import plotly.graph_objs as go
import numpy as np
from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
from plotly import tools
import pandas as pd
init_notebook_mode(connected=True)
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
data = utool.Data()
from uniflocpy.uTools import plotly_workflow
import re
import uniflocpy.uPVT.BlackOil_model as BlackOil_model
import uniflocpy.uTemperature as uTemperature
import uniflocpy.uReservoir.IPR_simple_line as IPR_simple_line
import uniflocpy.uTools.plotly_workflow as plotly_workflow
import uniflocpy.uValidation.python_api as python_api
import uniflocpy.uValidation.by_UniflocVBA as bvba


calc_options ={"step_lenth_in_calc_along_wellbore_m":25,
                "without_annulus_space":False,
                "solver_using":True}

pb_bar = 9 * 10 ** 5
fluid_data = {"rsb_m3m3": 56,
              "gamma_oil": 0.86,
              "gamma_gas": 1.45 / 1.293}

well_data = {"h_intake_mes_m": 1211,
             "h_intake_vert_m": 1211,
             "h_bottomhole_mes_m": 1757,
             "h_bottomhole_vert_m": 1757,

             "geothermal_grad_cm": 0.02,
             "t_wellhead_c": 20,
             "t_bottomhole_c": 40,
             "t_earth_init_in_reservoir_c": 40,
             'p_bottomhole_bar': 155.5,
             "d_casing_inner_m": 0.133,
             "d_tube_inner_m": 0.0503,
             "qliq_on_surface_m3day": 240,
             "fw_on_surface_perc": 25}
real_measurements = pd.DataFrame(
    {'p_survey_mpa': [0.975, 8.495, 9.44, 10.365, 10.902, 11.272, 12.085, 12.907, 13.785, 14.67, 15.55],
     'h_mes_survey_m': [0, 957, 1057, 1157, 1211, 1257, 1357, 1457, 1557, 1657, 1757]})


#well_data["t_wellhead_c"] = well_data["t_bottomhole_c"]

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


simple_well = self_flow_well.self_flow_well(fluid=1, reservoir = 0, pipe=0, temp_corr=1, **fluid_data,
                                            **well_data, **calc_options)

simple_well.pipe.fluid_flow.fl.option = blackoil_option

simple_well.pipe.hydr_cor.epsilon_friction_m = 0.0001

simple_well.calc_all_from_down_to_up()

uniflocvba = python_api.API('E:\\Git\\unifloc_vba\\UniflocVBA_7.xlam')


str_pvt = uniflocvba.PVT_encode_string(**fluid_data,tres_C =  well_data['t_earth_init_in_reservoir_c'])

calc_along_coord = 0
flow_along_coord =0
hydr_corr = 0
temp_method = 1
c_calibr_grav=1
c_calibr_fric=1
roughness_m=0.0001
q_gas_sm3day=0
out_curves_num_points=int(well_data["h_bottomhole_mes_m"]/calc_options['step_lenth_in_calc_along_wellbore_m'])


result =uniflocvba.MF_p_pipeline_atma(
    well_data["qliq_on_surface_m3day"],
    well_data["fw_on_surface_perc"],
    [
        [0,0],
         [well_data["h_bottomhole_mes_m"], well_data["h_bottomhole_mes_m"]
         ]
    ],
    uconst.bar2atm(well_data["p_bottomhole_bar"]),
    well_data["t_bottomhole_c"],
    calc_along_coord,
    flow_along_coord,
    str_pvt,
    [[0,well_data["d_tube_inner_m"] * 1000],
     [well_data["h_intake_mes_m"],well_data["d_casing_inner_m"] * 1000],
     [well_data["h_bottomhole_mes_m"],well_data["d_casing_inner_m"] * 1000]],
    hydr_corr,
    [[0,well_data["t_wellhead_c"]], [well_data["h_bottomhole_mes_m"],well_data["t_bottomhole_c"]]],
    temp_method,
    c_calibr_grav,
    c_calibr_fric,
    roughness_m,
    q_gas_sm3day,
    out_curves_num_points)


result_vba = bvba.covert_result_from_vba_to_df(result)


result_df = simple_well.data.get_data_as_df()
result_df.to_excel('test.xlsx')
result_df = result_df.set_index('well.h_calculated_mes_m')


real_measurements["p_survey_bar"] = real_measurements["p_survey_mpa"] * 10
real_measurements = real_measurements.set_index(real_measurements['h_mes_survey_m'])
real_measurements.index.name = 'well.h_calculated_mes_m'

result_df = result_df.join(real_measurements, how = 'outer')


result_df = result_df.join(result_vba, how = 'outer')



banches_with_patterns = {'Итоговый график': [[["p_bar", 'fl'], ['t_c', 'fl'], ['well.t_calculated_earth_init'], ['survey'], ['p_calculated_bar_vba'], ['t_calculated_c_vba']],
                                             ['mu', 'h_mes_survey', 'mpa']],



                        'Режим потока': [["flow_regime"], ['cor_number', 'cal']],
                        'Распределение давления': [["p_calculated_bar"], ['mu', 'h_mes_survey', 'mpa','fl','ipr']],
                        'Распределение температуры': [[ ['t_calculated_c_vba'], ['t_c', 'fl']],['mu', 'h_mes_survey', 'mpa']],
                        'Диаметры': [[["d_m"],['diam']], ['earth', 'mes_m']],
                        'Приведенные скорости': [['_msec'],['secm', 'msecpam', 'earth', 't_calculated', 'mass_flowrate', 'gasfrac']],
                        'Приведенная скорость жидкости': [["vsl_msec"],['secm', 'msecpam', 'earth', 't_calculated', 'mass_flowrate', 'gasfrac']],
                        'Еще одни скорости': [[["c_vl"],['c_vg']],['tpb', 'number']],
                        'Градиент давления по гравитации': [[ 'density'],['tpb', 'number', 'well_profile', 'percent']],
                        'Градиент давления по трению': [[ 'friction'],['tpb', 'number', 'well_profile', 'percent']],
                        'Число Re': [["number_Re"],['tpb', 'well_profile']],
                        'Истинное содержание жидкости': [["liquid_content_with_Pains"], ['tpb', 'well_profile']],
                        'Градиенты': [["grad"],['tpb', 'well_profile','percent','angle','therm','t_c']],
                        'angle_correction': [['angle_correction'],['tpb', 'well_profile']],
                        #'Парам': [[],[]],
                        #'Парам': [[],[]],
                        #'Парам': [[],[]],
                        'Объемный расход газа (все типы)': [["q", 'gas'], ['cor_number', 'cal']],
                        'Объемный расход воды': [["q", 'wat'], ['mass_fraction','cor_number', 'cal']],

                        'Вязкость ГЖС': [[['mu_mix'],['mun']], ['cor_number', 'cal']],
                        'Вязкость жидкости': [['mu_liq'], ['cor_number', 'cal']],
                        'Вязкости': [['mu'], ['cor_number', 'cal']],

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

banches = plotly_workflow.create_banches_from_pattern(result_df, banches_with_patterns)

plotly_workflow.create_report_html(result_df, banches, 'SelfFlowWell_by_UniflocVBA.html',
                                    shared_xaxes=False,
                                   shared_yaxes='all', cols=2, one_plot_height=400,
                                   verical_spacing=None,
                                   title_text=f"Сравнение SelfFlowWell by UniflocVBA, ось y - h_mes_m"
                                              f"\n исходные данные: \n gamma_oil: {str(fluid_data['gamma_gas'])[:4]},"
                                              f"gamma_gas: {str(fluid_data['gamma_gas'])[:4]}, "
                                              f"rsb_m3m3: {fluid_data['rsb_m3m3']}, "
                                              f"q_liq_sm3day: {well_data['qliq_on_surface_m3day']}, "
                                              f"watercut_perc: {well_data['fw_on_surface_perc']}",
                                   swap_xy=True,
                                   reversed_y=True
                                   )






