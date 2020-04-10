
import uniflocpy.uTools.uconst as uc
import numpy as np
import pandas as pd

columns_name_dict = {
    'num': ["num"],
    'h_calculated_mes_m': ["h,m"],
    'h_calculated_vert_m': ["hvert,m"],
    "p,atma": ["p,atma"],
    "t,C": ["t,C"],
    "Hl": ["Hl"],
    "flow_regime": ["fpat"],
    "t_amb, C": ["t_amb, C"],
    "diam, mm": ["diam, mm"],

    'c_Roughness': ["c_Roughness"],
    'c_Theta': ["c_Theta"],
    't_earth_init_c': ["c_Tinit"],
    'p_calculated_bar': ["c_P"],
    't_calculated_c': ["c_T"],
    't_earth_init_in_reservoir_c': ["c_Tamb"],
    'c_udl_m': ["c_udl_m"],
    'density_grad_pam': ["c_dpdl_g"],
    'friction_grad_pam': ["c_dpdl_f"],
    'acceleration_grad_pam': ["c_dpdl_a"],
    'c_vsl': ["c_vsl"],
    'c_vsg': ["c_vsg"],
    'liquid_content_with_Pains_cor_d': ["c_Hl"],
    'c_gasfrac': ["c_gasfrac"],
    'mu_oil_cp': ["c_muo"],
    'mu_wat_cp': ["c_muw"],
    'mu_gas_cp': ["c_mug"],
    'mun_cP': ["c_mumix"],
    'rho_oil_kgm3': ["c_rhoo"],
    'rho_wat_kgm3': ["c_rhow"],
    'rho_liq_kgm3': ["c_rhol"],
    'rho_gas_kgm3': ["c_rhog"],
    'rhon_kgm3': ["c_rhomix"],
    'qoil_m3day': ["c_qo"],
    'qwat_m3day': ["c_qw"],
    'qgas_m3day': ["c_qg"],
    'mass_flowrate_oil_kgsec': ["c_mo"],
    'mass_flowrate_wat_kgsec': ["c_mw"],
    'mass_flowrate_gas_kgsec': ["c_mg"],
    "c_vl": ["c_vl"],
    "c_vg": ["c_vg"],

    "rs_m3m3": ["c_Rs"],
}


def rename_columns_by_dict(df, columns_name_dict):
    """
    Специальное изменение названий столбцов по словарю
    :param df:
    :param dict:
    :return:
    """

    for i in df.columns:
        for items in columns_name_dict.items():
            if i in items[1]:
                df = df.rename(columns={i: items[0]})
    return df

def mark_df_columns(df, mark):
    """
    Пометка названий столбцов DataFrame c помошью (this_mark)
    :param df: исходный DataFrame
    :param mark: str, который будет приписан к названию столбца (например, СУ, или Ш)
    :return: DataFrame с новыми названиями столбцов
    """
    for i in df.columns:
        df = df.rename(columns={i: i + '_' + mark})
    return df

def covert_result_from_vba_to_df(result):
    df2 = create_result_df_from_vba_output(result)
    df2 = rename_columns_by_dict(df2, columns_name_dict)
    df2['p_calculated_bar'] = df2['p_calculated_bar'].apply(uc.atm2bar)
    df2['density_grad_pam'] = df2['density_grad_pam'].apply(uc.at2Pa)
    df2['friction_grad_pam'] = df2['friction_grad_pam'].apply(uc.at2Pa)
    df2['acceleration_grad_pam'] = df2['acceleration_grad_pam'].apply(uc.at2Pa)
    df2 = mark_df_columns(df2, 'vba')
    return df2


def create_result_df_from_vba_output(vba_result):
    result_np = np.array(vba_result[3:])
    df = pd.DataFrame(result_np,
                   columns=vba_result[2])
    df = df.set_index(df['h,m'])
    return df

if __name__ == '__main__':
    keywords = {"gamma_oil": gamma_oil, "gamma_gas": gamma_gas, "gamma_wat":gamma_water,
                                        "rsb_m3m3": rsb_m3m3, "t_res_c": t_res_c}

    keywords_vba = {"t_C": t_c, "gamma_gas": gamma_gas,
                    "gamma_oil": gamma_oil, "gamma_wat": gamma_water, "rsb_m3m3": rsb_m3m3, "tres_C": t_res_c} #проверено, улетают свойства газа (из-за z), st oil-gas, mu_wate

    python_fluid = BlackOil_model.Fluid(**keywords)

    uniflocvba = python_api.API('E:\\Git\\unifloc_vba\\UniflocVBA_7.xlam')


    def re_perc(y_fact,_y_calc):
        return abs((y_fact-_y_calc)/y_fact * 100)


    python_fluid.calc(p_bar, t_c)
    vba_value = uniflocvba.PVT_bo_m3m3(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.b_oil_m3m3
    print(f"b_oil_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_bw_m3m3(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.b_wat_m3m3
    print(f"b_wat_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_bg_m3m3(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.b_gas_m3m3
    print(f"b_gas_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_pb_atma(**keywords_vba)
    python_value = uc.bar2atm(python_fluid.pb_bar)
    print(f"pb_bar. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_rs_m3m3(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.rs_m3m3
    print(f"rs_m3m3. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_mu_gas_cP(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.mu_gas_cp
    print(f"mu_gas_cp. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_mu_oil_cP(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.mu_oil_cp
    print(f"mu_oil_cp. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_mu_wat_cP(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.mu_wat_cp
    print(f"mu_wat_cp. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_z(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.z
    print(f"z. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_salinity_ppm(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.s_ppm
    print(f"s_ppm. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_ST_oilgas_Nm(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.sigma_oil_gas_Nm
    print(f"sigma_oil_gas_Nm. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")

    vba_value = uniflocvba.PVT_ST_watgas_Nm(p_atma=p_atma, **keywords_vba)
    python_value = python_fluid.sigma_wat_gas_Nm
    print(f"sigma_wat_gas_Nm. vba_value: {vba_value}, python_value: {python_value}, relative_error: {re_perc(vba_value,python_value)}")





