import matplotlib.pyplot as plt
import pandas as pd


df = pd.read_csv('data.csv', sep=',')
Altitude = df['AltitudeFromTerrain']
Time = df['Time']
Velocity = df['Velocity']
Acceleration = df['Acceleration']
Mass = df['Mass']

plt.plot(Time, Velocity)
plt.ylabel(r'$V$')
plt.xlabel(r'$t$')
plt.title(r'Изменение скорости ракеты от времени(KSP)')
plt.grid(True)
plt.show()

plt.plot(Time, Acceleration)
plt.ylabel(r'$a$')
plt.xlabel(r'$t$')
plt.title(r'Изменение ускорения ракеты(KSP) по времени')
plt.grid(True)
plt.show()

plt.plot(Altitude)
plt.ylabel(r'$H$')
plt.xlabel(r'$t$')
plt.title(r'Изменение высоты ракеты от времени(KSP)')
plt.grid(True)
plt.show()

plt.plot(Time, Mass * 1000)
plt.ylabel(r'$M$')
plt.xlabel(r'$t$')
plt.title(r'Изменение массы ракеты от времени(KSP)')
plt.grid(True)
plt.show()
