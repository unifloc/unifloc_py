import datetime

left_boundaries = {252: [datetime.datetime(2018, 12, 7)],
                   326: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   507: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   540: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   569: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   570: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   601: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   658: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   693: [datetime.datetime(2018, 9, 22), datetime.datetime(2018, 11, 28)],
                   1479: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   1509: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   1567: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   1602: [datetime.datetime(2018, 8, 1), datetime.datetime(2018, 11, 28)],
                   1628: [datetime.datetime(2019, 1, 30)]
                   }

right_boundaries = {252: [datetime.datetime(2019, 2, 28)],
                    326: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    507: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    540: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    569: [datetime.datetime(2019, 11, 28), datetime.datetime(2019, 2, 27)],
                    570: [datetime.datetime(2018, 11, 6), datetime.datetime(2019, 2, 28)],
                    601: [datetime.datetime(2018, 11, 6), datetime.datetime(2019, 2, 28)],
                    658: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    693: [datetime.datetime(2018, 11, 6), datetime.datetime(2019, 2, 28)],
                    1479: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    1509: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    1567: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    1602: [datetime.datetime(2018, 11, 7), datetime.datetime(2019, 2, 28)],
                    1628: [datetime.datetime(2019, 2, 28)]
                    }

well_interval = {252: '01.07.2018-31.03.2019',
                 326: '01.08.2018-28.02.2019',
                 507: '01.07.2018-31.03.2019',
                 540: '01.08.2018-28.02.2019',
                 569: '01.07.2018-31.03.2019',
                 570: '01.08.2018-28.02.2019',
                 601: '01.07.2018-31.03.2019',
                 658: '01.08.2018-28.02.2019',
                 693: '01.08.2018-28.02.2019',
                 1354: '01.07.2018-31.03.2019',
                 1479: '01.08.2018-28.02.2019',
                 1509: '01.08.2018-28.02.2019',
                 1567: '01.07.2018-31.03.2019',
                 1602: '01.07.2018-31.03.2019',
                 1628: '01.07.2018-31.03.2019'
                 }

wells = [252, 326, 570, 569, 601, 658, 693, 1479, 1509, 1567, 1602, 1628]

critical_time_well = {252: 59,
                      326: 100,
                      569: 30,
                      507: 16,
                      540: 80,
                      570: 16,
                      601: 25,
                      658: 16,
                      693: 16,
                      1479: 50,
                      1509: 30,
                      1567: 16,
                      1602: 80,
                      1628: 80
                      }


def get_left_bound(well: int) -> list:
    return left_boundaries[well]


def get_rigth_bound(well: int) -> list:
    return right_boundaries[well]


def get_total_well_interval(well: int) -> str:
    return well_interval[well]


def get_well_names():
    return wells


def get_crit_time(well: int) -> int:
    return critical_time_well[well]
