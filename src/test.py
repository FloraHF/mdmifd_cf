from datetime import datetime

date, time = str(datetime.now()).split(' ')
t0 = '-'.join([date] + time.split('.')[0].split(':'))
print(t0)