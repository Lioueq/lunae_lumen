import math
import time
import krpc
from numpy import pi, dot, cross, array, arccos, sign
from numpy.linalg import norm


def angle_between(obj1, obj2, conn):
    frame = obj1.orbit.body.non_rotating_reference_frame
    pos1 = array(obj1.position(frame))
    pos2 = array(obj2.position(frame))

    # векторное произведение от pos1 до pos2.
    pos1 /= norm(pos1)
    pos2 /= norm(pos2)

    # вычисляем угол между векторами
    angle = arccos(dot(pos1, pos2))

    # переводим в радианы
    cp = cross(pos1, pos2)
    cp_local = conn.space_center.transform_direction(cp, frame, obj1.orbital_reference_frame)
    if cp_local[2] > 0:
        angle = -angle
    return angle


def time_until_phase(obj1, obj2, phase, conn):
    ''' find time until angle between obj1 and obj2 is phase '''
    angle = angle_between(obj1, obj2, conn)

    # вычисляем орбитальную радиальную скорость
    radians_per_second_1 = 2 * pi / obj1.orbit.period
    radians_per_second_2 = 2 * pi / obj2.orbit.period

    # вычисляем изменение угла в секунду
    relative_angle_change_per_second = radians_per_second_2 - radians_per_second_1
    delta = phase - angle
    while sign(delta) != sign(relative_angle_change_per_second):
        delta += sign(relative_angle_change_per_second) * 2 * pi
    return delta / relative_angle_change_per_second


def launch_phase():
    # параметры запуска
    turn_start_altitude = 250
    turn_end_altitude = 45000
    target_altitude = 150000
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0
    time.sleep(1)

    # активируем первую ступень
    vessel.control.activate_next_stage()
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)

    # отцепляем держатели
    vessel.control.activate_next_stage()

    # главный цикл
    srbs_separated = False
    turn_angle = 0
    while True:
        # gravity turn
        if turn_start_altitude < altitude() < turn_end_altitude:
            frac = ((altitude() - turn_start_altitude) /
                    (turn_end_altitude - turn_start_altitude))
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

        # отцепляем двигатели если кончилось топливо
        if not srbs_separated:
            if liq_fuel() < 0.1:
                vessel.control.activate_next_stage()
                srbs_separated = True

        if apoapsis() > target_altitude*0.9:
            break

    # выключаем двигатели если достигли заданного перицентра
    vessel.control.throttle = 0.25
    while apoapsis() < target_altitude:
        pass
    vessel.control.throttle = 0.0
    time.sleep(1)

    vessel.control.activate_next_stage()

    # ждем выхода из атмосферы
    while altitude() < 70500:
        pass


def kerbin_circularization_burn():
    # стабилизирования орбиты (используя уравнение vis-viva)
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    # высчитываем время работы двигателя
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

   # стабилизируем спутник
    vessel.auto_pilot.disengage()
    vessel.control.sas = True
    time.sleep(1)
    vessel.control.sas_mode = conn.space_center.SASMode.prograde

    # ожидание включения двигателей (варп тайм)
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # включаем двигатели
    time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burn_time/2) > 0:
        pass
    vessel.control.throttle = 1.0
    time.sleep(burn_time)
    vessel.control.throttle = 0
    vessel.control.remove_nodes()
    # стабилизируем спутник
    vessel.auto_pilot.disengage()
    vessel.control.sas = True
    time.sleep(1)
    vessel.control.sas_mode = conn.space_center.SASMode.prograde


def mun_phase():
    r_kerbin = vessel.orbit.semi_major_axis
    mu = vessel.orbit.body.gravitational_parameter
    mun = conn.space_center.bodies['Mun']
    r_mun = mun.orbit.semi_major_axis
    transfer_phase = pi * (1 - (((r_kerbin + r_mun) / (2 * r_mun)) ** 1.5))

    # высчитываем delta v для перелета
    dt = time_until_phase(vessel, mun, transfer_phase, conn)
    v1 = math.sqrt(mu / r_kerbin)
    r_ellipse = (r_kerbin + r_mun) / 2
    v2 = math.sqrt(mu * ((2/r_kerbin) - (1/r_ellipse)))
    delta_v = v2 - v1

    # варп тайм
    node = vessel.control.add_node(ut() + dt, prograde=delta_v)
    conn.space_center.warp_to(ut() + dt - 5)
    time.sleep(5)

    # выключаем двигатели
    vessel.control.throttle = 0.0
    time.sleep(0.01)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v / Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate
    vessel.control.throttle = 1.0
    time.sleep(burn_time * 0.95)
    vessel.control.throttle = 0.25
    while apoapsis() < r_mun:
        time.sleep(0.05)
    vessel.control.throttle = 0.0
    node.remove()

    time_to_change_of_soi = vessel.orbit.time_to_soi_change
    assert time_to_change_of_soi > 0
    conn.space_center.warp_to(ut() + time_to_change_of_soi + 10)

def mun_circularization_phase():
    # получаем параметры муны для стабилизации орбиты
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    node = vessel.control.add_node(
        ut() + vessel.orbit.time_to_periapsis, prograde=delta_v)

    # высчитываем время работы двигателя
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(- delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # ожидание включения двигателей (варп тайм)
    time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
    burn_ut = ut() + time_to_periapsis() - (burn_time / 2)
    lead_time = 15
    conn.space_center.warp_to(burn_ut - lead_time)

    # стабилизируем спутник
    vessel.control.sas = True
    time.sleep(0.1)
    vessel.control.sas_mode = conn.space_center.SASMode.retrograde
    while time_to_periapsis() - (burn_time/2.) > 0:
        pass
    vessel.control.throttle = 1.0
    time.sleep(burn_time)
    vessel.control.throttle = 0.0
    time.sleep(2)
    node.remove()
    vessel.control.activate_next_stage()

conn = krpc.connect(name='Lunae lumen')
vessel = conn.space_center.active_vessel

# датчики
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
stage_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
liq_fuel = conn.add_stream(stage_resources.amount, 'LiquidFuel')

# стадии перелета
launch_phase()
kerbin_circularization_burn()
mun_phase()
mun_circularization_phase()
