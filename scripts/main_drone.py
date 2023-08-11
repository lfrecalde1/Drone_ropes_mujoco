import numpy as np
import rospy
import time
import mujoco
import mujoco.viewer
from fancy_plots import fancy_plots_2, fancy_plots_1
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mujoco_drone import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Global variables Linear velocities
vxd = 0.0
vyd = 0.0
vzd = 0.0
# Global Variables angular velocities
wxd = 0.0
wyd = 0.0
wzd = 0.0


def velocity_call_back(velocity_message):
    global vxd, vyd, vzd, wxd, wyd, wzd
    # Read the linear Velocities
    vxd = velocity_message.linear.x
    vyd = velocity_message.linear.y
    vzd = velocity_message.linear.z

    # Read desired angular velocities from node
    wxd = velocity_message.angular.x
    wyd = velocity_message.angular.y
    wzd = velocity_message.angular.z
    return None

def main(odom_pub, ref_pub):
    # Load Model form XML file
    m = mujoco.MjModel.from_xml_path('drone.xml')
    # Print color of the box and position of the red box

    # Get information form the xml
    data = mujoco.MjData(m)

    # odometry Message
    odom_drone = Odometry()
    ref_drone = Twist()

    # Simulation time parameters
    ts = 0.001
    tf = 200
    t = np.arange(0, tf+ts, ts, dtype=np.double)

    # Parameters of the entire system
    mass = 0.4 + 4*(0.025) + 4*(0.015) + 0.05 # Mass condering the rope and the load
    g = 9.81

    # States System pose
    q = np.zeros((3, t.shape[0]+1), dtype=np.double)
    # States System Ori
    n = np.zeros((3, t.shape[0]+1), dtype=np.double)
    quat = np.zeros((4, t.shape[0]+1), dtype=np.double)

    # States System pose
    qp = np.zeros((3, t.shape[0]+1), dtype=np.double)
    rate_b = np.zeros((3, t.shape[0]+1), dtype=np.double)

    # Control signals
    u = np.zeros((4, t.shape[0]), dtype=np.double)

    # Desired reference signals of the system
    qdp = np.zeros((3, t.shape[0]), dtype=np.double)
    qdp[0,:] = 0.0
    qdp[1,:] = 0.0
    qdp[2,:] = 0.0

    rate_d = np.zeros((1, t.shape[0]), dtype=np.double)

    # Define Paramerters for the software
    m.opt.timestep = ts

    # Reset Properties system
    mujoco.mj_resetDataKeyframe(m, data, 0)  # Reset the state to keyframe 0

    # Set initial Conditions
    data.qpos[0] = 0
    data.qpos[1] = 0
    data.qpos[2] = 0.05

    # Memory PID
    roll_c = np.array([[0.0], [0.0], [0.0]])
    pitch_c = np.array([[0.0], [0.0], [0.0]])

    vy_c = np.array([[0.0], [0.0], [0.0]])
    vx_c = np.array([[0.0], [0.0], [0.0]])

    with mujoco.viewer.launch_passive(m, data) as viewer:
        if viewer.is_running():
            # Initial data System
            q[:, 0] = get_system_states_pos_sensor(data)
            qp[:, 0] = get_system_states_vel_sensor(data)
            n[: ,0] = get_system_states_ori_sensor(data)
            quat[: ,0] = get_system_states_quat_sensor(data)
            rate_b[: ,0] = get_system_states_vel_a_sensor(data)
            odom_drone = get_odometry(q[:, 0], quat[:,0], qp[:, 0], rate_b[:, 0], odom_drone)
            send_odometry(odom_drone, odom_pub)
            # Init System take off
            for k in range(0, 12000):
                tic = time.time()

                u[0, k] = controller_z(mass, g, [0, 0, 0.3], qp[:,k ])
                u[1, k], roll_c, vy_c = controller_attitude_roll([0, 0, 0.3], qp[:, k], [n[0, k], n[1, k], rate_b[2, k]], ts, roll_c, vy_c)
                u[2, k], pitch_c, vx_c = controller_attitude_pitch([0, 0, 0.3], qp[:, k], [n[0, k], n[1, k], rate_b[2, k]], ts, pitch_c, vx_c)
                u[3, k] = controller_attitude_r([0, 0, 0.3], qp[:, k], rate_d[:, k], [n[0, k], n[1, k], rate_b[2, k]])
                control_action(data, u[:,k])
                # System evolution
                mujoco.mj_step(m, data)

                # System values
                q[:, k+1] = get_system_states_pos_sensor(data)
                qp[:, k+1] = get_system_states_vel_sensor(data)
                n[: ,k+1] = get_system_states_ori_sensor(data)
                quat[: ,k+1] = get_system_states_quat_sensor(data)
                rate_b[: ,k+1] = get_system_states_vel_a_sensor(data)

                # Send Odometry Drone
                odom_drone = get_odometry(q[:, k+1], quat[:,k+1], qp[:, k+1], rate_b[:, k+1], odom_drone)
                send_odometry(odom_drone, odom_pub)

                # Send Control inputs
                ref_drone = get_reference(u[:, k], ref_drone)
                send_reference(ref_drone, ref_pub)

                # System evolution visualization
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
                viewer.sync()
                # Section to guarantee same sample times
                while (time.time() - tic <= m.opt.timestep):
                    None
                toc = time.time() - tic 


            # Simulation of the system
            for k in range(0, t.shape[0]):
                tic = time.time()
                # Update Reference Signals
                qdp[0, k] = vxd
                qdp[1, k] = vyd
                qdp[2, k] = vzd
                rate_d[0, k] = wzd

                # Control Section
                u[0, k] = controller_z(mass, g, qdp[:, k], qp[:, k])
                u[1, k], roll_c, vy_c = controller_attitude_roll(qdp[:, k], qp[:, k], [n[0, k], n[1, k], rate_b[2, k]], ts, roll_c, vy_c)
                u[2, k], pitch_c, vx_c = controller_attitude_pitch(qdp[:, k], qp[:, k], [n[0, k], n[1, k], rate_b[2, k]], ts, pitch_c, vx_c)
                u[3, k] = controller_attitude_r(qdp[:, k], qp[:, k], rate_d[:, k], [n[0, k], n[1, k], rate_b[2, k]])

                # Send Control Actions  to the system
                control_action(data, u[:,k])

                # System Evolution
                mujoco.mj_step(m, data)

                # Get system states
                q[:, k+1] = get_system_states_pos_sensor(data)
                qp[:, k+1] = get_system_states_vel_sensor(data)
                n[: ,k+1] = get_system_states_ori_sensor(data)
                quat[: ,k+1] = get_system_states_quat_sensor(data)
                rate_b[: ,k+1] = get_system_states_vel_a_sensor(data)

                # Send Odometry Drone
                odom_drone = get_odometry(q[:, k+1], quat[:,k+1], qp[:, k+1], rate_b[:, k+1], odom_drone)
                send_odometry(odom_drone, odom_pub)

                # Send Control inputs
                ref_drone = get_reference(u[:, k], ref_drone)
                send_reference(ref_drone, ref_pub)

                # System evolution visualization
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
                viewer.sync()

                # Section to guarantee same sample times
                while (time.time() - tic <= m.opt.timestep):
                    None
                toc = time.time() - tic 
                print(toc)

        fig1, ax1, ax2 = fancy_plots_2()
        ## Axis definition necesary to fancy plots
        ax1.set_xlim((t[0], t[-1]))
        ax2.set_xlim((t[0], t[-1]))
        ax1.set_xticklabels([])
        state_xd, = ax1.plot(t,qdp[0,0:t.shape[0]],
                    color='#9C1816', lw=2, ls="-")

        state_yd, = ax1.plot(t,qdp[1,0:t.shape[0]],
                    color='#179237', lw=2, ls="-")

        state_zd, = ax1.plot(t,qdp[2,0:t.shape[0]],
                    color='#175E92', lw=2, ls="-")

        state_x, = ax1.plot(t,qp[0,0:t.shape[0]],
                    color='#BB5651', lw=2, ls="-.")
        state_y, = ax1.plot(t,qp[1,0:t.shape[0]],
                    color='#76BB51', lw=2, ls="-.")
        state_z, = ax1.plot(t,qp[2,0:t.shape[0]],
                    color='#518EBB', lw=2, ls="-.")
        ax1.set_ylabel(r"$[m/s]$", rotation='vertical')
        ax1.legend([state_x,state_y,state_z,state_xd,state_yd,state_zd],
            [r'$\dot{x}$', r'$\dot{y}$', r'$\dot{z}$', r'$\dot{x}_d$', r'$\dot{y}_d$', r'$\dot{z}_d$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
        ax1.grid(color='#949494', linestyle='-.', linewidth=0.5)

        ax2.set_xlim((t[0], t[-1]))
        ax2.set_xticklabels([])
        state_rd, = ax2.plot(t,rate_d[0,0:t.shape[0]],
                    color='#9C1816', lw=2, ls="-")

        state_r, = ax2.plot(t, rate_b[2,0:t.shape[0]],
                    color='#76BB51', lw=2, ls="-.")
        
        ax2.set_ylabel(r"$[r/s]$", rotation='vertical')
        ax2.legend([state_r, state_rd],
            [r'${r}$', r'${r}_d$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
        ax2.grid(color='#949494', linestyle='-.', linewidth=0.5)

        fig1.savefig("system_states_controller.eps")
        fig1.savefig("system_states_controller.png")
        fig1
        plt.show()

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("drone_rope_simulation",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic = "/odom"
        odometry_publisher = rospy.Publisher(odomety_topic, Odometry, queue_size = 10)

        # Subscribe Info
        velocity_topic = "/cmd_vel"
        velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, velocity_call_back)

        # Publisher Info
        reference_topic = "/input_ref"
        reference_subscriber = rospy.Publisher(reference_topic, Twist, queue_size = 10)

        # Main System
        main(odometry_publisher, reference_subscriber)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass


    # Velocidades en el mundo Lineales
    # Velocidad angulares en el cuerpo
    # Vref Cuerpo
    # Quaternions 
    # Posiciones
    # Empuje y torque


