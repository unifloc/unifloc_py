
import uniflocpy.uTools.data_workflow as utool
import uniflocpy.uWell.uPipe as Pipe
import uniflocpy.uWell.Self_flow_well as self_flow_well

import pandas as pd

data = utool.Data()

import uniflocpy.uPVT.BlackOil_model as BlackOil_model
import uniflocpy.uTemperature as uTemperature
import uniflocpy.uReservoir.IPR_simple_line as IPR_simple_line


def calc_well_with_one_parameter(parameter):
    fluid_data = {"rsb_m3m3": 56,
                  "gamma_oil": 0.86,
                  "gamma_gas": 0.9}

    well_data = {"h_intake_mes_m": 1211,
                 "h_intake_vert_m": 1211,
                 "h_bottomhole_mes_m": 1757,
                 "h_bottomhole_vert_m": 1757,

                 "geothermal_grad_cm": 0.02,
                 "t_bottomhole_c": 40,
                 "t_earth_init_in_reservoir_c": 40,
                 'p_bottomhole_bar': 155.5,
                 "d_casing_inner_m": 0.133,
                 "d_tube_inner_m": 0.0503,
                 "qliq_on_surface_m3day": 240,
                 "fw_on_surface_perc": 25,

                 't_wellhead_c': 20,
                 'p_wellhead_bar': 9.75}
    real_measurements = pd.DataFrame(
        {'p_survey_mpa': [0.975, 8.495, 9.44, 10.365, 10.902, 11.272, 12.085, 12.907, 13.785, 14.67, 15.55],
         'h_mes_survey_m': [0, 957, 1057, 1157, 1211, 1257, 1357, 1457, 1557, 1657, 1757]})

    calc_options = {"step_lenth_in_calc_along_wellbore_m": 24.35,
                    "without_annulus_space": False}

    reservoir = IPR_simple_line.IPRSimpleLine()
    ipr_m3daybar = reservoir.calc_pi_m3daybar(well_data['qliq_on_surface_m3day'], well_data['p_bottomhole_bar'], 250)

    fluid_data = {"rsb_m3m3": 56,
                  "gamma_oil": 0.86,
                  "gamma_gas": 0.9}

    pipe = Pipe.Pipe()

    pipe = Pipe.Pipe(temp_cor=uTemperature.temp_cor_simple_line.SimpleLineCor())

    blackoil_option = BlackOil_model.BlackOil_option()

    blackoil_option.b_wat_cor_number = 1
    blackoil_option.mu_wat_cor_number = 1
    blackoil_option.rho_wat_cor_number = 1

    fluid = BlackOil_model.Fluid(**fluid_data, option=blackoil_option)

    simple_well = self_flow_well.self_flow_well(fluid=fluid, reservoir=reservoir, pipe=pipe,
                                                **well_data, **calc_options)
    simple_well.well_work_time_sec = 1

    simple_well.save_all = False

    simple_well.fw_on_surface_perc = 20
    simple_well.pipe.fluid_flow.fl.rsb_m3m3 = parameter
    results_q = []
    results_p = []
    for i in range(1, 300, 10):
        simple_well.p_wellhead_bar = 20
        simple_well.data.clear_data()
        simple_well.qliq_on_surface_m3day = i
        simple_well.calc_all_from_up_to_down()
        results_q.append(i)
        results_p.append(simple_well.p_bottomhole_bar)
    one_df = pd.DataFrame({f"p_bottomhole_bar {parameter}": results_p}, index = results_q)

    return one_df
if __name__ == '__main__':
    result = calc_well_with_one_parameter(20)
    print(result)