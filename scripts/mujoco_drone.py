import numpy as np
from scipy.spatial.transform import Rotation as R

def control_action(system, u):
    u1 = u[0]
    u2 = u[1]
    u3 = u[2]
    u4 = u[3]
    
    system.ctrl[0] = u1
    system.ctrl[1] = u2
    system.ctrl[2] = u3
    system.ctrl[3] = u4
    return None

def controller_z(mass, gravity, qdp, qp):
    # Control Gains
    Kp = 50*np.eye(3, 3)
    # Split values
    xp = qp[0]
    yp = qp[1]
    zp = qp[2]

    xdp = qdp[0]
    ydp = qdp[1]
    zdp = qdp[2]

    # Control error
    error = qdp - qp

    error_vector = error.reshape((3,1))

    # Control Law
    aux_control = Kp@error_vector

    # Gravity + compensation velocity
    control_value = mass*gravity + aux_control[2,0]
    
    return control_value

def pid(sp, real, memories, kp, ki, kd, t_sample):
    error = np.tanh(sp - real)
    error_1 = memories[1, 0]
    error_2 = memories[2, 0]
    u_1 = memories[0, 0]
    p = kp * (error - error_1)
    i = ki * error * t_sample
    d = kd * (error - 2 * error_1 + error_2) / t_sample
    u = u_1 + p + i + d

    # Update memories
    memories[0, 0] = u
    memories[2, 0] = error_1
    memories[1, 0] = error
    return u, memories

def controller_attitude_roll(qdp, qp, rate, ts, r_c, v_c):

    # Split values

    p = rate[0]
    q = rate[1]
    r = rate[2]

    xpd = qdp[0]
    ypd = qdp[1]
    zpd = qdp[2]

    xp = qp[0]
    yp = qp[1]
    zp = qp[2]

    roll_d, v_c = pid(ypd, yp, v_c, 0.5, 0, 0.001, ts)
    # Control Law
    control_value, r_c = pid(-roll_d, p, r_c, 0.05, 0, 0.05, ts)

    
    return control_value, r_c, v_c

def controller_attitude_pitch(qdp, qp, rate, ts, p_c, v_c):

    # Split values

    p = rate[0]
    q = rate[1]
    r = rate[2]

    xpd = qdp[0]
    ypd = qdp[1]
    zpd = qdp[2]

    xp = qp[0]
    yp = qp[1]
    zp = qp[2]

    pitch_d, v_c = pid(xpd, xp, v_c, 0.5, 0, 0.001, ts)

    # Control Law
    control_value, p_c = pid(pitch_d, q, p_c, 0.05, 0, 0.05, ts)

    
    return control_value, p_c, v_c

def controller_attitude_r(qdp, qp, rate_d, rate):

    # Split values
    rd= rate_d[0]

    p = rate[0]
    q = rate[1]
    r = rate[2]

    error = rd - r
    # Control Law
    control_value = 0.1*error
    return control_value


def get_system_states_pos_sensor(system):
    q0 = system.sensor("position_drone").data.copy()
    x = np.array([q0], dtype=np.double)
    return x

def get_system_states_vel_sensor(system):
    q0 = system.sensor("linear_velocity_drone").data.copy()
    x = np.array([q0], dtype=np.double)
    return x

def get_system_states_vel_a_sensor(system):
    q0 = system.sensor("angular_velocity_drone").data.copy()
    x = np.array([q0], dtype=np.double)
    return x

def get_system_states_ori_sensor(system):
    q0 = system.sensor("quat_drone").data.copy()
    x = np.array([q0[1], q0[2], q0[3], q0[0]], dtype=np.double)
    r = R.from_quat(x)
    return r.as_euler('xyz', degrees=False)

def get_system_states_quat_sensor(system):
    q0 = system.sensor("quat_drone").data.copy()
    x = np.array([q0], dtype=np.double)
    return x
