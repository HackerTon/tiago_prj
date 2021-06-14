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
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal, MoveBaseAction
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


poses = [
    Pose(Point(7.33190965652, 4.13128566742, 0.0), Quaternion(
        0.0, 0.0, 0.999659681939, 0.026086784101)),
    Pose(Point(7.79271697998, -5.17713737488, 0.0),
         Quaternion(0.0, 0.0, -0.0565811701486, 0.998398002394)),
    Pose(Point(-0.135614752769, 0.923734128475, 0.0),
         Quaternion(0.0, 0.0, 0.699049673936, 0.715073110507)),
]


class Driver():
    def __init__(self):
        self.tfbuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfbuffer)
        time.sleep(1.0)  # to fix wait_for_server bug

        # initialize pickup_pos from server code
        self.pickup_action = SimpleActionClient(
            '/pickup_pose', PickUpPoseAction)
        # wait for server to be connect within 20 seconds
        # else logerr and exit program
        if not self.pickup_action.wait_for_server(rospy.Duration(20)):
            rospy.logerr('/pickup_pose cannot be connected')
            rospy.logerr('program die')
            exit()
        rospy.loginfo("/pickup_pose checked!")

        self.torso_pub = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_pub = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1)
        self.detected_pub = rospy.Publisher(
            '/detected_aruco_pose', PoseStamped, queue_size=1, latch=True)

        # initialize tuck arm action client
        self.play_motion_action = SimpleActionClient(
            '/play_motion', PlayMotionAction)
        # wait for server to be connected within 20 seconds
        # else logerr and exit program
        if not self.play_motion_action.wait_for_server(rospy.Duration(20)):
            rospy.logerr('/play_motion cannot be connected')
            rospy.logerr('program die')
            exit()
        rospy.loginfo("/play_motion checked!")

        self.move_base_action = SimpleActionClient(
            '/move_base', MoveBaseAction)
        # wait for server to be connected within 20 seconds
        # else logerr and exit program
        if not self.move_base_action.wait_for_server(rospy.Duration(20)):
            rospy.logerr('/move_base cannot be connected')
            rospy.logerr('program die')
            exit()
        rospy.loginfo("/move_base checked!")

        rospy.sleep(1.0)  # wait for server bug
        self.pickup_service = rospy.Service(
            '/pickup', Empty, lambda x: self.pick('pick'))

        rospy.loginfo('Successfully started commander!')

        # for pose in poses:
        #     self.move(pose)

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
        return s[1:] if s.startswith('/') else s

    def pick(self, operation):
        self.unfold_arm()
        self.lower_head()

        rospy.sleep(2)
        rospy.loginfo('waiting marker')

        try:
            aruco_markers = rospy.wait_for_message(
                '/aruco_many/aruco_markers', MarkerArray, rospy.Duration(10.0))
        except rospy.ROSException as e:
            rospy.logerr('wait for aruco error' + e.message)
            return

        aruco_pose = aruco_markers.markers[0]
        aruco_pose.header.frame_id = self.strip(aruco_pose.header.frame_id)

        # change aruco into posestamped
        ps = PoseStamped()
        ps.pose.position = aruco_pose.pose.pose.position
        ps.header.stamp = self.tfbuffer.get_latest_common_time(
            'base_footprint', aruco_pose.header.frame_id)
        ps.header.frame_id = aruco_pose.header.frame_id

        # make transformation to get pose
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfbuffer.lookup_transform(
                    'base_footprint', ps.header.frame_id, rospy.Time(0))
                aruco_ps = do_transform_pose(ps, transform)
                transform_ok = True
            except Exception as _:
                rospy.logwarn('Transform error try again')
                rospy.sleep(0.01)
                ps.header.stamp = self.tfbuffer.get_latest_common_time(
                    'base_footprint', aruco_pose.header.frame_id
                )
        rospy.loginfo('transform successful')

        pick_g = PickUpPoseGoal()
        if operation == 'pick':
            rospy.loginfo('open')
            pick_g.object_pose.pose.position = aruco_ps.pose.position
            # set picking point to center of cube
            pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)

            pick_g.object_pose.header.frame_id = 'base_footprint'
            pick_g.object_pose.pose.orientation.w = aruco_ps.pose.orientation.w

            self.detected_pub.publish(pick_g.object_pose)
            rospy.loginfo("Picking" + str(pick_g))
            self.pickup_action.send_goal_and_wait(pick_g)

            result = self.pickup_action.get_result()
            if str(moveit_error_dict[result.error_code]) != 'SUCCESS':
                rospy.logerr('Failed to pick')
                return

            self.lift_torso()

            # # Raise arm
            # rospy.loginfo("Moving arm to a safe pose")
            # pmg = PlayMotionGoal()
            # pmg.motion_name = 'pick_final_pose'
            # pmg.skip_planning = False
            # self.play_m_as.send_goal_and_wait(pmg)
            # rospy.loginfo("Raise object done.")

            # # Place the object back to its position
            # rospy.loginfo("Gonna place near where it was")
            # pick_g.object_pose.pose.position.z += 0.05
            # self.place_as.send_goal_and_wait(pick_g)
            # rospy.loginfo("Done!")

    def lower_head(self):
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_pub.publish(jt)

    def lift_torso(self):
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_pub.publish(jt)

    # move arm away code
    def unfold_arm(self):
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_motion_action.send_goal_and_wait(pmg)


if __name__ == '__main__':
    rospy.init_node('robot_commander')
    driver = Driver()
    rospy.spin()
