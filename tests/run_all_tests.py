"""
Полное тестирование uniflocpy и applications,
которое соберается из тестирования отдельных модулей
"""
import unittest

import tests.uMotor_test as uMotor_test
import tests.uMultiphaseFlow_test as uMultiphaseFlow_test
import tests.uPVT_test as uPVT_test
import tests.uTemperature_test as uTemperature_test
import tests.uTools_test as uTools_test


# объединение в список отдельных тестовых модулей мини-пакетов
modules = [uMotor_test, uMultiphaseFlow_test, uPVT_test, uTemperature_test,
           uTools_test]

testLoad = unittest.TestLoader()  # загрузчик из модулей
calcTestSuite = unittest.TestSuite()  # объединитель тестов

for i in modules:  # загрузка тестов из модулей
    calcTestSuite.addTests(testLoad.loadTestsFromModule(i))

runner = unittest.TextTestRunner(verbosity=2)
runner.run(calcTestSuite)  # запуск абсолютно всех тестов
