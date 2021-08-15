#! /usr/bin/env python
"""Node ur5_2"""
import sys
import copy
import threading
import math
import cv2
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import numpy as np
import tf2_ros
import tf2_msgs.msg
import yaml
import time



from std_msgs.msg import String
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper
#Importing neccessary libraries and modules.


class Ur5Moveit(object):
    """This is class Ur5Moveit"""

    def __init__(self):
        """This is Constructor"""
        rospy.init_node('main.py', anonymous=True)
        #Attributes to store frame names for TF.
        self.target_frame2 = "logical_camera_2_"

        #Attribute to store transform.
        self._robot_ns = '/'  + 'ur5_2'
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
        self._exectute_trajectory_client.wait_for_server()
        self.Ids=['00','01','02','10','11','12','20', '21', '22']
        self.FinalColors=[]


        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._Logical_Cam_Values = LogicalCameraImage()
        self._Logical_Camera_Subcscriber = rospy.Subscriber("/eyrc/vb/logical_camera_2",
                                                            LogicalCameraImage,
                                                            self.Logical_Cam_CB)

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def Logical_Cam_CB(self, data):
        """Callback method for Logical Camera"""
        self._Logical_Cam_Values = data

    
    def get_qr_data(self, arg_image):
        #decode qrcode
        qr_result = decode(arg_image)
        #list to store colors
        colors = []
        #loop through recieved code
        for qr in qr_result:
            #extract x,y,w,h
            x,y,w,h = qr.rect
            #find indexes of packages to sort later
            index = int(str(y/100)+str(x/100))
            colors.append([index,qr.data])
        #sort the list of tupples based on indices
        colors.sort()
        for k in colors:
            #colo is what we need
            junk,col = k
            self.FinalColors.append(col)
    
    def getColors(self):
        """Method to get colors in suitable way"""
        print "colors",self.FinalColors
        colors = self.FinalColors[0:9]
        self.FinalColors = []
        for i in colors:
            self.FinalColors.append(i[0])



    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        
        (rows,cols,channels) = cv_image.shape

        print "CHECK THE ROWS AND ALL",(rows, cols, channels)
        image = cv_image
        image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        image = cv2.addWeighted(image, 2.4, np.zeros(image.shape, image.dtype), 0, 0)

        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(image, (720/2, 1280/2)) 

        #cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

        rospy.loginfo(self.get_qr_data(image))

        self.image_sub.unregister()
        cv2.destroyAllWindows()

        cv2.waitKey(3)


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """Method for Cartisian translation"""
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        #rospy.loginfo("Path computed successfully. Moving the arm.")


        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def goto_package_cartesian(self):
        """ Method to lead EE on top of package effectively
            We recieve flag, if true we condier the recieved frame to calculate
            the transform between the package and the EE to lead the EE on top
            of the package to be able to grip it effectively.
        """

        
        #we construct the frame from recieved frame name
        #package_frame = self.target_frame2 + package_frame + "_frame"
        #print package_frame

        #Store box pose with respect to Logical Camera
        box_dupose = self._Logical_Cam_Values.models[1].pose

        #Here we calculate pose of Box with respect to WORLD
        Box_wrt_World_X = self._Logical_Cam_Values.models[1].pose.position.z - 0.8
        Box_wrt_World_Y = self._Logical_Cam_Values.models[1].pose.position.y
        Box_wrt_World_Z = 2.0 - self._Logical_Cam_Values.models[1].pose.position.x

        """print("-----------CHECK BOX POSE WRT WORLD----------------")

        print(Box_wrt_World_X,Box_wrt_World_Y,Box_wrt_World_Z)"""

        #store EE's pose with respect to WORLD
        pose_values = self._group.get_current_pose().pose


        # Here we calculate the Transform between EE and package
        Trans_x = -(pose_values.position.x - Box_wrt_World_X)
        Trans_y = -(pose_values.position.y - Box_wrt_World_Y)
        Trans_z = -(pose_values.position.z - Box_wrt_World_Z) + 0.197

        #We call the method to calculate the transform.
        #self.func_tf_print(self.target_frame1, package_frame)
        self.ee_cartesian_translation(Trans_x, Trans_y, Trans_z)

        #Grab the package.
        gripperClient(True)

        WORLD_Z = 0.020
        #In order to have enough gap between belt and package  an offset of 0.020.
        self.ee_cartesian_translation(0, 0, WORLD_Z)

        #We remove the in hand packageId from list of ID's
        self.Ids.pop(0)

    def Conveyor_Operator(self):
        """Method for operating the conveyor effectively"""

        #global count variable to keep track of packages.
        #flag to toggle once package is in logical camera's vision.
        flag = False

        packageId = self.Ids[0]

        #Construct package's type name to check if present in Logical Cam's vision.
        packageName = "packagen"+packageId
        print"In controller with ",packageName

        #POW to store conveyor power.
        POW = 100

        #loop infintely
        while True:
            #Check every model in vision and keep conveyor running until package enters.
            for i in range(len(self._Logical_Cam_Values.models)):
                if packageName == self._Logical_Cam_Values.models[i].type:
                    flag = True
                    break
            if flag:
                break
            ConveyorControl(POW)

        #Once package is in boundaries
        #Keep conveyor ON till it is near EE.
        while self._Logical_Cam_Values.models[len(self._Logical_Cam_Values.models)-1].pose.position.y > 0.1:
            ConveyorControl(POW)
        #stop conveyor once package is in desired position.
        ConveyorControl(0)



    def set_joint_angles(self, flag):
        """Method to set joint angles"""
        """flag holds either H,R,G,B which stands for home,red,blue,green respectively"""
        #Store current pose.
        list_joint_values = self._group.get_current_joint_values()

        #Home pose angles.
        Home_Angles = [math.radians(7.84685),
                       math.radians(-139.945119871),
                       math.radians(-58.2879),
                       math.radians(-71.73162),
                       math.radians(89.979374),
                       math.radians(7.79083)]

        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        #Condition check for flag to actuate arm to specified  R,G,B bin or home pose.
        if flag == 'H':
            #if requested for home pose
            pass
        else:
            inflag = self.FinalColors[0]
            #else depending on list of colors actuate the arm accordingly
            
            if inflag == 'r':
                Home_Angles = list_joint_values
                Home_Angles[0] = math.radians(-77)
            elif inflag == 'y':
                Home_Angles = list_joint_values
                Home_Angles[0] = math.radians(180)
            else:
                Home_Angles = list_joint_values
                Home_Angles[0] = math.radians(94)

        #Hard set angles.

        number_attempts = 0
        flag_success = False
        while ( (number_attempts <= 50) and  (flag_success is False) ):
            number_attempts += 1
            self._group.set_joint_value_target(Home_Angles)
            self._group.plan()
            flag_success = self._group.go(wait=True)

        flag_plan = flag_success

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        if not flag == 'H':
            self.FinalColors.pop(0)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def ConveyorControl(power):
    """Client to access Conveyor service"""

    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    try:
        resp1 = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        resp1(power)
    except rospy.ServiceException as e:
        print "Service call failed : %s"%e

def gripperClient(flag):
    """Client to activate or deactivate the gripper"""

    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')

    try:
        resp1 = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
        return resp1(flag)
    except rospy.ServiceException as e:
        print "Service call failed : %s"%e


def main():
    """Main"""

    for i in range(30):
        print"Waiting for packages "
        rospy.sleep(1)

    #Create a new object.
    ur5 = Ur5Moveit()
    print"-------------------------------------------"

    #initialy moving the arm to home pose.
    ur5.set_joint_angles('H')

    ur5.getColors()

    print"Continuing"

    #Run conveyor
    ur5.Conveyor_Operator()
    packageCount = 0

    #while number of packages are less than 9
    while packageCount < 9:
        
        ur5.goto_package_cartesian()
        #creating a new thread.
        thread1 = threading.Thread(name="worker1", target=ur5.Conveyor_Operator)
        thread1.start()
        #by passing NULL arm moves towards bin corresponding to pkg color
        ur5.set_joint_angles('Null')
        gripperClient(False)
        ur5.set_joint_angles('H')
        thread1.join()
        #increment count
        packageCount = packageCount + 1

    #destructor.
    del ur5


if __name__ == '__main__':
    main()