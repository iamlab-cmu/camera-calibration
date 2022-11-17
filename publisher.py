 ###################################################################
 #
 # This python file is to read in [cam_transform.csv, ee_transform.csv]
 # and ROS publish them as /Transforms for VISP calibrator package 
 #
 # Output : /Transforms
 # 
 # Input  : [cam_transform.csv, ee_transform.csv]
 #          
 
 # E-mail : MoonRobotics@cmu.edu    (Lee Moonyoung)
 # Code written by c.h.johnkim@gmail.com (John Chunghee Kim)
 #
 # Versions :
 # v1.0
 ###################################################################

import rospy
from geometry_msgs.msg import Transform

def read_log():
    object2cam_file = open("./cam_transform.txt", "r") 
    object2cam = []
    for line in object2cam_file: 
        pose = [float(x) for x in line.split(',')]
        object2cam.append(pose)
    
    effector2world_file = open("./ee_transform.txt", "r") 
    effector2world = []
    for line in effector2world_file: 
        pose = [float(x) for x in line.split(',')]
        effector2world.append(pose)
    
    return effector2world, object2cam      

def transformation_publisher():
    effector2world, object2cam = read_log()
    print("Number of transformation pairs: ", len(effector2world))

    pub_co = rospy.Publisher('/camera_object', Transform, queue_size=10)
    pub_we = rospy.Publisher('/world_effector', Transform, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    co = Transform()
    we = Transform()
    rospy.sleep(1)

    if (len(effector2world) == len(object2cam)):
        for i in range(len(effector2world)):
            co = Transform()
            we = Transform()
        
            co.translation.x = object2cam[i][0]
            co.translation.y = object2cam[i][1]
            co.translation.z = object2cam[i][2]
            co.rotation.x = object2cam[i][3]
            co.rotation.y = object2cam[i][4]
            co.rotation.z = object2cam[i][5]
            co.rotation.w = object2cam[i][6]
            we.translation.x = effector2world[i][0]
            we.translation.y = effector2world[i][1]
            we.translation.z = effector2world[i][2]
            we.rotation.x = effector2world[i][3]
            we.rotation.y = effector2world[i][4]
            we.rotation.z = effector2world[i][5]
            we.rotation.w = effector2world[i][6]
        
            pub_co.publish(co)
            pub_we.publish(we)
            rate.sleep()
    else:
        print("ERROR: Transformation pairs do not match.")

if __name__ == '__main__':
    rospy.init_node('hand2eye_calibration', anonymous=True)
    transformation_publisher()
