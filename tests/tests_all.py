"""
@author Кобзарь Олег
Запуск всех тестов из всех файликов
"""
# загрузка всех модулей
import unittest
import uniflocpy.uMotor.equivalent_circuit as equivalent_circuit
import uniflocpy.uTemperature.temp_cable_NS as temp_cable_NS
import uniflocpy.uTemperature.temp_cor_Hasan_Kabir_integrated as temp_cor_Hasan_Kabir_integrated
import uniflocpy.uMultiphaseFlow.friction_Bratland as friction_Bratland
import uniflocpy.uMultiphaseFlow.hydr_cor_Beggs_Brill as hydr_cor_Beggs_Brill
import uniflocpy.uPVT.PVT_fluids as PVT_fluids
import uniflocpy.uPVT.PVT_correlations as PVT_correlations

# объединение в список
modules = [equivalent_circuit, temp_cable_NS, temp_cor_Hasan_Kabir_integrated, friction_Bratland,
           hydr_cor_Beggs_Brill, PVT_fluids, PVT_correlations]

testLoad = unittest.TestLoader()  # загрузчик из модулей
calcTestSuite = unittest.TestSuite()  # объединитель тестов

for i in modules:  # загрузка тестов их модулей
    calcTestSuite.addTests(testLoad.loadTestsFromModule(i))

runner = unittest.TextTestRunner(verbosity=2)
runner.run(calcTestSuite)