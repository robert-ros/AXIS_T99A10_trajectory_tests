#!/usr/bin/env python

from scipy.interpolate import interp1d
import numpy as np
import time

x = np.linspace(0, 30, num=31, endpoint=True)
#y = 90*np.cos(-x**2/9.0)+90
y = 45*np.cos(2**np.pi*x*10)+45
f = interp1d(x, y)
f2 = interp1d(x, y, kind='cubic')
xnew = np.linspace(0, 30, num=131, endpoint=True)

index = int()
spline_time_now = float()
spline_time_last = float()
spline_elapsed_time = float()

index = 0
spline_time_now = 0
spline_time_last = 0
spline_elapsed_time = 0


import matplotlib.pyplot as plt
#plt.plot(x, y, 'o', xnew, f(xnew), '-', xnew, f2(xnew), 'o')
plt.plot(x, y, '-', xnew, f2(xnew), 'o')
plt.legend(['data','cubic'], loc='best')
plt.show(block = False)

count = int()
count = 0

for t in xnew:

    count+=1
    print(count)    
    print(f2(t))

    spline_time_now = t 
    spline_elapsed_time = spline_time_now - spline_time_last
    spline_time_last = spline_time_now
    
    print(spline_elapsed_time)
    time.sleep(spline_elapsed_time)





