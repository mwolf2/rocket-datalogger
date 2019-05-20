""""
Created on Sat May 18 18:04:15 2019

@author: EOL
"""

import numpy as np
import matplotlib.pyplot as plt
data = np.genfromtxt('Successful_flightdata_2019-05-18.csv', dtype = 'float',
                     names = True,delimiter=',')
#Specify range to encompass launch,Flight Dependant
m = np.logical_and(data['Time']>660,data['Time']<720)
data['Time'] = data['Time'] - 667.9478788376

#Acceleration plot
plt.figure()
plt.plot(data['Time'][m],data['Xa'][m],'.')
plt.plot(data['Time'][m],data['Ya'][m],'.')
plt.plot(data['Time'][m],data['Za'][m],'.')

#Time sampling plot
plt.figure()
plt.plot(data['Time'][m][1:]-data['Time'][m][:-1])

alpha = .7
beta = .09
def alpha_beta_filter(time,rawdata,alpha,beta):
    xfilt = np.copy(rawdata)
    vfilt = np.copy(0*rawdata)
    for i in range(1, len(time)):
        dt = time[i]-time[i-1]
        xfilt[i] = xfilt[i-1] + dt*vfilt[i-1]
        vfilt[i] = vfilt[i-1]
        
        error = rawdata[i] - xfilt[i]
        
        xfilt[i] = xfilt[i] + alpha * error
        vfilt[i] = vfilt[i] + (beta * error) / dt
    return xfilt,vfilt

def remove_outliers(rawdata,threshold = 10):
    mask = np.copy(0*rawdata)+ True
    for i in range (1, len(rawdata)-1):
        match = -0.5*rawdata[i-1] + rawdata[i] - .5*rawdata[i+1]
        if abs(match) > 40:
            mask[i]= False
            
    return mask 
mask = remove_outliers(rawdata = data['AGL'])
mBarom = np.logical_and(mask,m)
#Agl plot 
Aglfilt, vertvel = alpha_beta_filter(data['Time'][mBarom],data['AGL'][mBarom],alpha,beta)
plt.figure()
plt.plot(data['Time'][mBarom],data['AGL'][mBarom],'.')
plt.plot(data['Time'][mBarom],Aglfilt,'.')
#Vertical velocity plot
plt.figure()
plt.plot(data['Time'][mBarom],vertvel,'.')

#Flight dependant data
print('Burn Between 0 and 0.7211351395 s; data was logged approximately every 0.04514024148 s, 22.1531823283 hz'
      '; Data will be analyzed between T-8 s and T+52 s, launch is T-0 seconds;' 
      'event happened between T+9.0 and T+9.8;'
      'Apogee at T+6.25s, 816 ft, 255 m;')
