import math
import time
import krpc
from numpy import pi, dot, cross, array, arccos, sign
from numpy.linalg import norm


turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000


def angle_between(obj1, obj2, conn):
    frame = obj1.orbit.body.non_rotating_reference_frame
    pos1 = array(obj1.position(frame))
    pos2 = array(obj2.position(frame))

    # find the cross product from pos1 to pos2
    pos1 /= norm(pos1)
    pos2 /= norm(pos2)

    # angle from dot product
    angle = arccos(dot(pos1, pos2))

    # flip if cross product opposite orbital normal of first object
    cp = cross(pos1, pos2)
    cp_local = conn.space_center.transform_direction(cp, frame, obj1.orbital_reference_frame)
    if cp_local[2] > 0:  # left handed coordinate system
        angle = - angle

    return angle

def time_until_phase(obj1, obj2, phase, conn):
    ''' find time until angle between obj1 and obj2 is phase '''
    angle = angle_between(obj1, obj2, conn)
    print("angle between is", angle)

    # find orbital radial velocity
    radians_per_second_1 = 2 * pi / obj1.orbit.period
    radians_per_second_2 = 2 * pi / obj2.orbit.period

    # assume obj2 is running away, obj1 is chasing
    relative_angle_change_per_second = radians_per_second_2 - radians_per_second_1
    print("p, a, r", phase, angle, relative_angle_change_per_second)
    delta = phase - angle
    while sign(delta) != sign(relative_angle_change_per_second):
        delta += sign(relative_angle_change_per_second) * 2 * pi
    return delta / relative_angle_change_per_second


def launch_phase():
    # Pre-launch setup
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0
    time.sleep(1)
    print('Launch!')


    # Activate the first stage
    vessel.control.activate_next_stage()
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)

    # break off chains)))
    vessel.control.activate_next_stage()

    # Main ascent loop
    srbs_separated = False
    turn_angle = 0
    while True:
        # Gravity turn
        if turn_start_altitude < altitude() < turn_end_altitude:
            frac = ((altitude() - turn_start_altitude) /
                    (turn_end_altitude - turn_start_altitude))
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

        # Separate SRBs when finished
        if not srbs_separated:
            if liq_fuel3() < 0.1:
                vessel.control.activate_next_stage()
                srbs_separated = True
                print('SRBs separated')

        # Decrease throttle when approaching target apoapsis
        if apoapsis() > target_altitude*0.9:
            print('Approaching target apoapsis')
            break

    # Disable engines when target apoapsis is reached
    vessel.control.throttle = 0.25
    while apoapsis() < target_altitude:
        pass
    print('Target apoapsis reached')
    vessel.control.throttle = 0.0
    time.sleep(1)

    print("break off main engines")
    vessel.control.activate_next_stage()

    # Wait until out of atmosphere
    print('Coasting out of atmosphere')
    while altitude() < 70500:
        pass

def kerbin_circularization_phase():
    # Plan circularization burn (using vis-viva equation)
    print('Planning circularization burn')
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate


   # Orientating the ship
    vessel.auto_pilot.disengage()
    print('Auto Pilot disengaged')
    vessel.control.sas = True
    print('SAS engaged')
    time.sleep(1)
    vessel.control.sas_mode = conn.space_center.SASMode.prograde
    print('SAS set to prograde')

    # Waiting until burn
    print('Waiting until circularization burn')
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Executing burn
    print('Ready to execute burn')
    time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burn_time/2) > 0:
        pass
    print('Executing burn')
    vessel.control.throttle = 1.0
    time.sleep(burn_time)
    print('The vessel is successfully parked at ', target_altitude // 1000,' km orbit')
    vessel.control.throttle = 0
    vessel.control.remove_nodes()
    # Orientating the ship
    vessel.auto_pilot.disengage()
    print('Auto Pilot disengaged')
    vessel.control.sas = True
    print('SAS engaged')
    time.sleep(1)
    vessel.control.sas_mode = conn.space_center.SASMode.prograde
    print('SAS set to prograde')

def moon_phase():
    global state
    r_kerbin = vessel.orbit.semi_major_axis
    mu = vessel.orbit.body.gravitational_parameter
    mun = conn.space_center.bodies['Mun']
    r_mun = mun.orbit.semi_major_axis
    transfer_phase = pi * (1 - (((r_kerbin + r_mun) / (2 * r_mun)) ** 1.5))
    print("phase", transfer_phase)

    # compute node and delta-v for transfer
    dt = time_until_phase(vessel, mun, transfer_phase, conn)
    v1 = math.sqrt(mu / r_kerbin)
    r_ellipse = (r_kerbin + r_mun) / 2
    v2 = math.sqrt(mu * ((2/r_kerbin) - (1/r_ellipse)))
    delta_v = v2 - v1
    print("dV", delta_v)

    node = vessel.control.add_node(ut() + dt, prograde=delta_v)
    conn.space_center.warp_to(ut() + dt - 5)
    time.sleep(5)  # reorient

    print('Executing transfer burn')
    vessel.control.throttle = 0.0
    time.sleep(0.01)  # let throttle go to zero
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v / Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate
    print("burn time", burn_time)
    vessel.control.throttle = 1.0
    time.sleep(burn_time * 0.95)
    vessel.control.throttle = 0.25
    while apoapsis() < r_mun:
        time.sleep(0.05)
    vessel.control.throttle = 0.0
    node.remove()

    # warp to change of influence
    time_to_change_of_soi = vessel.orbit.time_to_soi_change
    assert time_to_change_of_soi > 0
    conn.space_center.warp_to(ut() + time_to_change_of_soi + 10)

def moon_circularization_phase():
    global state
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    assert delta_v < 0
    node = vessel.control.add_node(
        ut() + vessel.orbit.time_to_periapsis, prograde=delta_v)  # negative delta v

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(- delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Wait until burn
    print('Waiting until second circularization burn')
    time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
    burn_ut = ut() + time_to_periapsis() - (burn_time / 2)
    lead_time = 5  # warp turns off SAS, give time to reorient
    conn.space_center.warp_to(burn_ut - lead_time)

    # use SAS for reotrgrade burn
    vessel.control.sas = True
    time.sleep(0.1)  # allow SAS to turn on
    vessel.control.sas_mode = conn.space_center.SASMode.retrograde

    # Orientate ship
    print('Orientating ship for second circularization burn')
    while time_to_periapsis() - (burn_time/2.) > 0:
        pass
    print('Executing burn - {} seconds'.format(burn_time))
    vessel.control.throttle = 1.0
    time.sleep(burn_time)
    vessel.control.throttle = 0.0
    node.remove()
    vessel.control.activate_next_stage()
    vessel.control.activate_next_stage()
    state = False


# Connect to KSP
conn = krpc.connect(name='Lunae lumen')
vessel = conn.space_center.active_vessel
state = True

# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
stage_3_resources = vessel.resources_in_decouple_stage(stage=3, cumulative=False)

liq_fuel3 = conn.add_stream(stage_3_resources.amount, 'LiquidFuel')
liq_fuel2 = conn.add_stream(stage_2_resources.amount, 'LiquidFuel')

# phases
launch_phase()
kerbin_circularization_phase()
moon_phase()
moon_circularization_phase()
