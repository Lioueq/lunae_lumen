from math import sqrt, sin, cos, pi

import matplotlib.pyplot as plt
import pandas as pd

# Atmospheric pressure at different altitudes
pressure = {0: 1.225, 2000: 1.0065, 5000: 0.7365, 8000: 0.5258,
            10000: 0.4135, 12000: 0.3119, 16000: 0.1665, 20000: 0.0889, 24000: 0.0469,
            32000: 0.0136, 40000: 0.004, 50000: 0.00103}
PI = 3.141592653589793
RAD_TO_DEG = 180 / PI


class Vector:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        if not isinstance(other, Vector):
            raise TypeError
        return Vector(self.x + other.x, self.y + other.y)


# Функция, преобразующая углы из градусов в радианы.
def radians_to_degrees(x):
    return x / RAD_TO_DEG


def interpolate(a, b, t):
    return a * (1 - t) + b * t


# Функция, вычисляющая плотность атмосферы на заданной высоте.
def calculate_density(current_height: int):
    items = list(pressure.items())
    for i in range(1, len(pressure)):
        if items[i - 1][0] <= current_height < items[i][0]:
            return interpolate(items[i - 1][1], items[i][1],
                               (current_height - items[i - 1][0]) / (items[i][0] - items[i - 1][0]))
    return 0


time_rise = []
height = 0
resistance_coefficient = 0.765
first_stage_thrust = 2300000
data = {'Altitude': [], 'Velocity': [], 'Acceleration_x': [], 'Acceleration_y': []}
acceleration = Vector()
velocity = Vector()
angle_of_attack = 90
gravitational_constant = 3.98 * 10 ** 12
initial_mass = 74600
time = 0
x = [0]
y = [0]
df = pd.read_csv('data.csv', sep=',')
Inclination = df['Inclination']

while height <= 150000:
    total_speed = sqrt(velocity.x ** 2 + velocity.y ** 2)
    thrust_force = first_stage_thrust
    gravity_force = gravitational_constant * initial_mass / ((6400000 + height) ** 2)
    air_density = calculate_density(height)
    resistance_force = (resistance_coefficient * (pi * 12.25) * (total_speed ** 2)
                        * air_density / 2)
    acceleration.y = (thrust_force - resistance_force - gravity_force) * sin(
        radians_to_degrees(angle_of_attack)) / initial_mass
    height += (velocity.y + acceleration.y) / 2
    data['Altitude'].append(height)
    acceleration.x = (thrust_force - resistance_force - gravity_force) * cos(
        radians_to_degrees(angle_of_attack)) / initial_mass
    data['Velocity'].append(sqrt(velocity.x ** 2 + velocity.y ** 2))
    data['Acceleration_x'].append(acceleration.x)
    data['Acceleration_y'].append(acceleration.y)

    velocity.x = acceleration.x + velocity.x
    velocity.y = acceleration.y + velocity.y

    if time < 99:
        initial_mass -= 334
    if time == 98:
        initial_mass -= 12400
    if 98 < time < 169:
        initial_mass -= 224
    if time == 168:
        initial_mass -= 3000
    if 168 < time < 332:
        initial_mass -= 27
    if time == 331:
        initial_mass -= 200

    time += 1
    x.append(x[-1] + velocity.x + acceleration.x / 2)

    if time >= 97 and time <= 140:
        first_stage_thrust = 300000
        print(time)
    if time >= 141 and time <= 167:
        first_stage_thrust = 70000
    if time > 167:
        first_stage_thrust = 5000
    angle_of_attack = angle_of_attack - Inclination[time]
    time_rise.append(time)
with open("data.csv", "r") as file:
    ksp = pd.read_csv(file)
plt.plot(ksp["Velocity"][:200], color='blue')
plt.plot(time_rise, data['Velocity'], color='red')
plt.legend(["KSP", "Мат.модель"])
plt.title('Зависимость скорости от времени')
plt.xlabel("Время в секундах")
plt.ylabel("Скорость, м/c")
plt.show()
plt.title("Зависимость плотности воздуха от высоты")
plt.xlabel("Высота, м")
plt.ylabel("Плотность")
plt.plot(pressure.keys(), pressure.values())
plt.show()
plt.title("Зависимость скорости и высоты")
plt.xlabel("Высота, м")
plt.ylabel("скорость, м/с")
plt.plot(data["Altitude"], data["Velocity"], color="red")
plt.title("Зависимость скорости и высоты")
plt.xlabel("Высота, м")
plt.ylabel("скорость, м/с")
plt.scatter(ksp["AltitudeFromTerrain"][:350], ksp["Velocity"][:350], 1, color="blue")
plt.legend(["Мат.модель", "KSP"])
plt.show()
plt.plot(list(map(lambda x: x / 1000, ksp["AltitudeFromTerrain"][:200])), color='blue')
plt.plot(range(len(data["Altitude"])), list(map(lambda x: x / 1000, data["Altitude"])), color='red')
plt.legend(["KSP", "Мат.модель"])
plt.title('Зависимость высоты от времени')
plt.xlabel('Время в секундах')
plt.ylabel('Высота, км')
plt.show()
data = pd.DataFrame(data)
data.to_csv("data.csv")
print(data)
