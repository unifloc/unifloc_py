a=2
b=2

class Save_temp_fluid_c(object):
    """
    Класс для сохранения значений температуры внутри функции
    temp_fluid, в которой решения выполняются итеративно
    с помощью fsolve()
    """
 
    def __init__(self, init):
        """
        Начальное значение температуры
        """
        self.temp_fluid_c = init

    def save(self,value):
        """
        Сохранение вычисленных значений T в класс
        """
        self.temp_fluid_c=value

class All_properties(object):
	def __init__(self):
		#convective heat transfer in an Oil Well
		self.ql_m3sec=79.5/86400 
		self.qg_m3sec=283/86400
		self.kl_wmc=0.138
		self.kg_wmc=1.73*10**(-4)
		self.rhol_kgm3=882.9
		self.rhog_kgm3=80.3
		self.cpl_jkgc=2512
		self.cpg_jkgc=2093
		self.mul_pas=0.015
		self.mug_pas=1.5*10**(-4)
		self.rti_m=0.0259
       
		#natural convection in well annulus
		self.rto_m=0.0561
		self.rci_m=0.0797
		#parametres are medium
		self.mu_an_pas=0.0001
		self.cp_an_jkgc=1004.81
		self.rho_an_km3=36.92
		self.k_an_wmc=0.865
		self.betta_1c=0.004824
        
		#overall heat transfer coefficient
		self.rco_m=0.0889
		self.rwb_m=0.1079
		self.kcem_wmc=0.779
		self.kt_wmc=25
		self.ke_wmc=2.422
        
		self.time_sec=100*7*24*3600
		self.rhoe_kgm3=2504
		self.cpe_jkgc=1256
        
		self.tei_c=93.3
		self.distance_m=1000
		self.gg_cm=0.027
		self.gamma_gas=0.65
		self.gamma_api=29
		self.p_pa=792.9*10**3
        
		self.pwh=115
		@property
		def pwh(self):
			return self.pwh

		
	
	
def_prop=All_properties()

'''def summ(a=real_prop.a,b=real_prop.b):
	"""
	example:
	umm(a=real_prop.a,b=real_prop.b)
	"""
	print(a+b)
	print('check')'''
	
def fc_var2(p_pa,mt_kgs,rp_sm3sm3,gamma_api,gamma_gas,gg_cm):
    """
    Корреляция Sagar et al. (1991) для расчетра Fc
    физически - совокупность влияния эффекта Джоуля-Томпсона и
    изменения кинетической энергии на теплоперенос
    p_pa - давление, Па
    mt_kgs - массовый расход, кг/с
    rp_sm3sm3 - газовый фактор, м3/м3 ???
    gamma_api - плотность нефти, АPI
    gamma_gas - относительная плотность газа (по воздуху???)
    gg_cm - геотермический градиент, градус Цельсия на м 
	out -  градус Цельсия на м 
	Расчет в несистемных размерностях, выход в СИ
	
    """    
	#Перевод размерностей
    p_psi=p_pa/6894.757293178
    gg_fft=gg_cm/1.82268883056
    rp_scfstb=rp_sm3sm3/0.17810760667903497
    mt_lbms=mt_kgs*2.2046226218487757 #TODO проверить
    
    result_fft= ((-2.978)*10**(-3)+1.006*10**(-6)*p_psi+1.906*10**(-4)*mt_lbms-1.047*10**(-6)*rp_scfstb+
            3.229*10**(-5)*gamma_api+4.009*10**(-3)*gamma_gas-0.3551*gg_fft)
    return result_fft*1.82268883056