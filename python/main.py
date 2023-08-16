import sys
import time
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R
from scipy.io import savemat

## Global variables system
xd = 3.0
yd = -4.6
zd = 5.16
vxd = 0.0
vyd = 0.0
vzd = 0.0

qx = 0.0005
qy = 0.0
qz = 0.0
qw = 1.0
wxd = 0.0
wyd = 0.0
wzd = 0.0

# Get system states
def get_system_states_sensor():
    quat = np.array([qx, qy, qz, qw], dtype=np.double)
    rot = R.from_quat(quat)
    euler = rot.as_euler('xyz', degrees=False)

    x = np.array([xd, yd, zd, qw, qx, qy, qz, euler[0], euler[1], euler[2]], dtype=np.double)
    return x

# Get system velocities
def get_system_velocity_sensor():
    x = np.array([vxd, vyd, vzd, wxd, wyd, wzd], dtype=np.double)
    return x

## Reference system
def get_reference(ref, ref_msg):
        ref_msg.twist.linear.x = 0
        ref_msg.twist.linear.y = 0
        ref_msg.twist.linear.z = ref[0]

        ref_msg.twist.angular.x = ref[1]
        ref_msg.twist.angular.y = ref[2]
        ref_msg.twist.angular.z = ref[3]
        return ref_msg

def send_reference(ref_msg, ref_pu):
    ref_pu.publish(ref_msg)
    return None

def odometry_call_back(odom_msg):
    global xd, yd, zd, qx, qy, qz, qw, time_message, vxd, vyd, vzd, wxd, wyd, wzd

    # Read desired linear velocities from node
    time_message = odom_msg.header.stamp
    xd = odom_msg.pose.pose.position.x 
    yd = odom_msg.pose.pose.position.y
    zd = odom_msg.pose.pose.position.z
    vxd = odom_msg.twist.twist.linear.x
    vyd = odom_msg.twist.twist.linear.y
    vzd = odom_msg.twist.twist.linear.z


    qx = odom_msg.pose.pose.orientation.x
    qy = odom_msg.pose.pose.orientation.y
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w

    wxd = odom_msg.twist.twist.angular.x
    wyd = odom_msg.twist.twist.angular.y
    wzd = odom_msg.twist.twist.angular.z
    return None

def main(control_pub):

    # Twist 
    ref_drone = TwistStamped()

    # Simulation time parameters
    ts = 0.03
    tf = 20
    t = np.arange(0, tf+ts, ts, dtype=np.double)

    # Get initial Conditions of the system
    # States System pose
    h = np.zeros((10, t.shape[0]+1), dtype=np.double)

    # States System pose
    hp = np.zeros((6, t.shape[0]+1), dtype=np.double)

    # Control signals
    u_ref = np.zeros((4, t.shape[0]), dtype=np.double)

    # Define Control Action
    #u_ref[0, :]=  0.7*np.cos(0.5*t)+0.2*np.sin(0.4*t)
    #u_ref[1, :] = 0.1*np.cos(1*t)+0.1*np.cos(2*t)
    #u_ref[2, :] = 0.1*np.cos(2*t)+0.1*np.cos(5*t) 
    #u_ref[3, :] = 1*np.sin(2*t)

    u_ref[0, :]=  0
    u_ref[1, :] = -0.8 * np.sign(np.sin(t))
    u_ref[2, :] = 0.5 * np.sign(np.sin(t))
    u_ref[3, :] = 0.5 * np.sign(np.cos(t))

    experiment_number = 3



    for k in range(0, 50):
        tic = time.time()
        ## Get Contol Action or nothing
        ref_drone = get_reference([0, 0, 0, 0], ref_drone)
        send_reference(ref_drone, control_pub)

        # Loop_rate.sleep()
        while (time.time() - tic <= ts):
                None
        toc = time.time() - tic 
        print("Init System")

    # Set initial Conditions
    h[:, 0] = get_system_states_sensor()
    hp[:, 0] = get_system_velocity_sensor()
    t_k = 0

    for k in range(0, t.shape[0]):
        tic = time.time()

        # Send Control action to the system
        ref_drone = get_reference(u_ref[:, k], ref_drone)
        send_reference(ref_drone, control_pub)


        # Loop_rate.sleep()
        while (time.time() - tic <= ts):
                None
        toc = time.time() - tic 
        print(toc)

        # Save Data
        h[:, k+1] = get_system_states_sensor()
        hp[:, k+1] = get_system_velocity_sensor()
        t_k = t_k + toc
    # Set zero Values
    ref_drone = get_reference([0, 0, 0, 0], ref_drone)
    send_reference(ref_drone, control_pub)

    mdic_h = {"h": h, "label": "experiment_h"}
    mdic_hp = {"hp": hp, "label": "experiment_hp"}
    mdic_u = {"u_ref": u_ref, "label": "experiment_u"}
    mdic_t = {"t": t, "label": "experiment_t"}

    savemat("h_"+ str(experiment_number) + ".mat", mdic_h)
    savemat("hp_"+ str(experiment_number) + ".mat", mdic_hp)
    savemat("u_ref_" + str(experiment_number) + ".mat", mdic_u)
    savemat("t_"+ str(experiment_number) + ".mat", mdic_t)

    return None


if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("Python_Node",disable_signals=True, anonymous=True)

        # Odometry topic
        odometry_webots = "/dji_sdk/odometry"
        odometry_subscriber = rospy.Subscriber(odometry_webots, Odometry, odometry_call_back)

        # Cmd Vel topic
        velocity_topic = "/m100/velocityControl"
        velocity_publisher = rospy.Publisher(velocity_topic, TwistStamped, queue_size = 10)

        main(velocity_publisher)



    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass