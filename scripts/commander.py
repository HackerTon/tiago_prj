#!/usr/bin/env python
import rospy
import tf2_ros
import time
from actionlib import SimpleActionClient
from tiago_prj.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from aruco_msgs.msg import MarkerArray, Marker
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal, MoveBaseAction, Moveerr
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


poses = [Pose(Point(-2.72496604919, -0.78588116169, 0.0),
              Quaternion(0.0, 0.0, 0.828647508407, 0.559770762733))]


class Driver():
    def __init__(self):
        self.tfbuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfbuffer)

        # initialize pickup_pos from server code
        self.pickup_action = SimpleActionClient(
            '/pickup_pose', PickUpPoseAction)
        time.sleep(1.0)  # to fix wait_for_server bug

        # wait for server to be connect within 20 seconds
        # else logerr and exit program
        if not self.pickup_action.wait_for_server(rospy.Duration(20)):
            rospy.logerr('/pickup_pose cannot be connected')
            rospy.logerr('program die')
            exit()

        self.torso_pub = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1
        )
        self.head_pub = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1
        )
        self.detected_pub = rospy.Publisher(
            '/detected_aruco_pose', PoseStamped, queue_size=1, latch=True)

        # initialize tuck arm action client
        self.play_motion_action = SimpleActionClient(
            '/playmotion', PlayMotionAction)

        # wait for server to be connected within 20 seconds
        # else logerr and exit program
        if not self.play_motion_action.wait_for_server(rospy.Duration(20)):
            rospy.logerr('/playmotion cannot be connected')
            rospy.logerr('program die')
            exit()

        self.move_base_action = SimpleActionClient(
            '/move_base', MoveBaseAction)

        # wait for server to be connected within 20 seconds
        # else logerr and exit program
        if not self.move_base_action.wait_for_server(rospy.Duration(20)):
            rospy.logerr('/move_base cannot be connected')
            rospy.logerr('program die')
            exit()

        rospy.sleep(1.0)  # wait for server bug

        # TODO ADD CALLFUNCTION
        self.pickup_service = rospy.Service('/pickup', Empty)

        rospy.loginfo('Successfully started commander!')

    def move(self, pose):
        movebasegoal = self.create_movebasegoal(pose)
        self.move_base_action.send_goal_and_wait(movebasegoal)
        rospy.loginfo("MOVED TO {} {} {}".format(
            pose.position.x, pose.position.y, pose.position.z))

    def create_movebasegoal(self, pose):
        movebasegoal = MoveBaseGoal()

        movebasegoal.target_pose.header.frame_id = 'map'
        movebasegoal.target_pose.header.stamp = rospy.Time.now()
        movebasegoal.target_pose.pose = pose

        return movebasegoal

    def strip(self, s):
        return s[:1] if s.startwith('/') else

    def pick(self):
        self.tuck_arm()
        self.lower_head()

        rospy.sleep(2)
        aruco_makers = rospy.wait_for_message('/aruco_markers', MarkerArray)

        # get only one aruco from markersarray
        if aruco_makers.markers > 2:
            aruco_pose = aruco_makers.markers[0]
        aruco_pose.header.frame_id = self.strip(aruco_pose.header.frame_id)

        # change aruco into posestamped
        ps = PoseStamped()
        ps.pose.position = aruco_pose.pose.position
        ps.header.stamp = self.tfbuffer.get_latest_common_time(
            'base_footprint', aruco_makers.header.frame_id
        )
        ps.header.frame_id = aruco_pose.header.frame_id

        # make transformation to get pose
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfbuffer.lookup_transform(
                    'base_footprint', ps.header.frame_id, rospy.Time(0))
                aruco_ps = do_transform_pose(ps, transform)
            except Exception as e:
                rospy.logwarn('Transform error try again')
                rospy.sleep(0.01)
                ps.header.stamp = self.tfbuffer.get_latest_common_time(
                    'base_footprint', aruco_pose.header.frame_id
                )

        pick_g = PickUpPoseGoal()

        if operation == 'pick':
            pick_g.object_pose.pose.position = aruco_ps.pose.position
            # set picking point to center of cube
            pick_g.object.pose.pose.position.z -= 0.1*(1.0/2.0)

            pick_g.object_pose.header.frame_id = 'base_footprint'
            pick_g.object_pose.pose.orientation.w = aruco_ps.pose.orientation

            self.detected_pub.publish(pick_g.object_pose)
            rospy.loginfo("Picking" + str(pick_g))
            self.pickup_action.send_goal_and_wait(pick_g)

            result = self.pickup_action.get_result()
            if str(moveit_error_dict[result.error_code]) != 'SUCCESS':
                rospy.logerr('Failed to pick')
                return

            self.lift_torso()

            # Raise arm
            rospy.loginfo("Moving arm to a safe pose")
            pmg = PlayMotionGoal()
            pmg.motion_name = 'pick_final_pose'
            pmg.skip_planning = False
            self.play_m_as.send_goal_and_wait(pmg)
            rospy.loginfo("Raise object done.")

            # Place the object back to its position
            rospy.loginfo("Gonna place near where it was")
            pick_g.object_pose.pose.position.z += 0.05
            self.place_as.send_goal_and_wait(pick_g)
            rospy.loginfo("Done!")

    def lower_head(self):
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.position = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_pub.publish(jt)

    def lift_torso(self):
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.position = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_pub.publish(jt)

    # move arm away code
    def tuck_arm(self):
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_motion_action.send_goal_and_wait(pmg)


if __name__ == '__main__':
    rospy.init_node('robot_commander')
    driver = Driver()
    rospy.spin()
