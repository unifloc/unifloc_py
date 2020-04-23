
import pandas as pd
import uniflocpy.uWell.Self_flow_well as self_flow_well
import uniflocpy.uPVT.BlackOil_model as bom


blackoil_option = bom.BlackOil_option()

blackoil_option.b_wat_cor_number = 1
blackoil_option.mu_wat_cor_number = 1
blackoil_option.rho_wat_cor_number = 1

def calc_well_with_one_parameter(data):

    object_class, all_parameters_dict, defined_range, parameter_name_to_iterate, parameter_name_to_extract,\
        parameter_name_to_fill, parameter_value_to_fill = data
    result_i = []
    result_parameter = []
    for i in defined_range:
        this_dict = all_parameters_dict
        this_dict[parameter_name_to_iterate] = i
        this_object = object_class(**this_dict)
        this_object.pipe.fluid_flow.fl.option = blackoil_option
        this_object.calc_all_from_up_to_down()
        result_i.append(i)
        result_parameter.append(this_object.__dict__[parameter_name_to_extract])
    one_df = pd.DataFrame({f"{parameter_name_to_extract} при {parameter_name_to_fill} = {parameter_value_to_fill}":
                               result_parameter}, index=result_i)
    one_df.index.name = parameter_name_to_iterate
    return one_df

if __name__ == '__main__':
    data = (self_flow_well.self_flow_well, {'fluid':1,'temp_corr':1}, [10,20,30],
                                          'qliq_on_surface_m3day', 'p_bottomhole_bar')
    result = calc_well_with_one_parameter(data)
    print(result)