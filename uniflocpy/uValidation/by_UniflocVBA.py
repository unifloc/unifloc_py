
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
    'vsl_msec': ["c_vsl"],
    'vsg_msec': ["c_vsg"],
    'liquid_content_with_Pains_cor_d': ["c_Hl"],
    'gas_fraction': ["c_gasfrac"],
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
    df2['density_grad_pam'] = df2['density_grad_pam'].apply(uc.atm2Pa)
    df2['friction_grad_pam'] = df2['friction_grad_pam'].apply(uc.atm2Pa)
    df2['acceleration_grad_pam'] = df2['acceleration_grad_pam'].apply(uc.atm2Pa)

    df2['qliq_m3day'] = df2['qoil_m3day'] + df2['qwat_m3day']
    df2['q_mix_n_m3day'] = df2['qliq_m3day'] + df2['qgas_m3day']
    df2['rhos_kgm3'] = df2['rho_liq_kgm3'] * df2['liquid_content_with_Pains_cor_d'] + df2['rho_gas_kgm3'] * (1 - df2['liquid_content_with_Pains_cor_d'])
    df2['grad_pam'] = df2['density_grad_pam'] + df2['friction_grad_pam'] + df2['acceleration_grad_pam']
    df2['d_m'] = df2['diam, mm']
    df2['vm_msec'] = df2['vsl_msec'] + df2['vsg_msec']
    df2['mass_flowrate_liq_kgsec'] = df2['mass_flowrate_oil_kgsec'] + df2['mass_flowrate_wat_kgsec']
    df2['mass_flowraten_kgsec'] = df2['mass_flowrate_liq_kgsec'] + df2['mass_flowrate_gas_kgsec']
    df2['liquid_content'] = 1 - df2['gas_fraction']
    df2 = mark_df_columns(df2, 'vba')
    return df2


def create_result_df_from_vba_output(vba_result):
    result_np = np.array(vba_result[3:])
    df = pd.DataFrame(result_np,
                   columns=vba_result[2])
    df = df.set_index(df['h,m'])
    return df




