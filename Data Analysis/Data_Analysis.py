""""
Created on Sat May 18 18:04:15 2019

@author: EOL
"""

import numpy as np
import matplotlib.pyplot as plt
data = np.genfromtxt('Successful_flightdata_2019-05-18.csv', dtype = 'float',
                     names = True,delimiter=',')
#Specify range to encompass launch

m = np.logical_and(data['Time']>660,data['Time']<720)
#plt.plot(data['Time'],m,'.')
plt.figure()
plt.plot(data['Time'],data['Xa'],'.')
plt.plot(data['Time'],data['Ya'],'.')
plt.plot(data['Time'],data['Za'],'.')
plt.xlim(660,720)

print('Burn Between 667.9478788376 s and 668.6690139771 s; 0.7211351395 s recorded'
      ' burn, data was logged  approximately every 0.04514024148 s, 22.1531823283 hz'
      ', Data will be analyzed between 660 s and 720 s')
