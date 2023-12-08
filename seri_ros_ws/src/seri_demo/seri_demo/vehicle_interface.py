import os
import sys
import time
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy)
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from tier4_external_api_msgs.srv import Engage
from tier4_external_api_msgs.srv import SetEmergency
from autoware_auto_vehicle_msgs.msg import HazardLightsCommand
from std_msgs.msg import Int64
from derived_object_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry
from data_collection_msgs.msg import DataCollection


class DemoAction(Node):
    """
    Class used to send information to the ego vehicle or sensor in order to disrupt the behavior of the ego vehicle
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        # create a timer
        # timer callback is called every 0.1 seconds
        self._timer = self.create_timer(0.1, self._collect_data_cb)

        # Current position of the ego vehicle
        self._ego_current_position_x = None
        self._ego_current_position_y = None

        # Current position of the front vehicle
        self._front_current_position_x = None
        self._front_current_position_y = None

        # Current linear velocity of the ego vehicle
        self._ego_current_linear_x = None

        self._emergency_ahead_published = False

        # publisher to the /emergency_ahead topic
        self._emergency_ahead_pub = self.create_publisher(
            Int64, 'emergency_ahead', 100)
        
        self._collect_data_pub = self.create_publisher(
            DataCollection, 'collect_data', 100)

        # Callback groups to allow multiple callbacks to be called at the same time
        # timer_cb_group = MutuallyExclusiveCallbackGroup()
        gnss_cb_group = MutuallyExclusiveCallbackGroup()
        objects_cb_group = MutuallyExclusiveCallbackGroup()
        odom_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscriber to the /carla/ego_vehicle/odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self._odom_cb,
            10,
            callback_group=odom_cb_group
        )

        # Subscriber to the /sensing/gnss/pose topic of the ego vehicle
        self.gnss_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.gnss_pose_cb,
            10,
            callback_group=gnss_cb_group
        )

        # create a subscriber to the topic /carla/objects
        self._objects_sub = self.create_subscription(
            ObjectArray,
            '/carla/objects',
            self._carla_objects_cb,
            10,
            callback_group=objects_cb_group
        )

    def _collect_data_cb(self):

        if self._emergency_ahead_published:
            if self._ego_current_linear_x <= 0.1:
                pass

        if self._ego_current_position_x is None or self._ego_current_position_y is None:
            return

        if self._front_current_position_x is None or self._front_current_position_y is None:
            return

        if self._ego_current_linear_x is None:
            return

        # compute distance between ego and front vehicles
        distance = ((self._ego_current_position_x - self._front_current_position_x) **
                    2 + (self._ego_current_position_y - self._front_current_position_y)**2)**0.5
        
        self.get_logger().info(f'Ego Position x: {self._ego_current_position_x}')
        self.get_logger().info(f'Ego Position y: {self._ego_current_position_y}')
        self.get_logger().info(f'Front position x: {self._front_current_position_x}')
        self.get_logger().info(f'Front position y: {self._front_current_position_y}')
        
        self.get_logger().info(f'Distance: {distance}')
        
        msg = DataCollection()
        msg.velocity = self._ego_current_linear_x
        msg.distance = distance
        self._collect_data_pub.publish(msg)

    def _odom_cb(self, msg: Odometry):
        self._ego_current_linear_x = msg.twist.twist.linear.x
        self._ego_current_position_x = msg.pose.pose.position.x
        self._ego_current_position_y = msg.pose.pose.position.y
        # self.get_logger().info(f'Linear velocity: {self._ego_current_linear_x}')

    def _carla_objects_cb(self, msg: ObjectArray):
        '''
        Callback function for the /carla/objects topic
        This function is used to get the current position of the stopped vehicle

        Args:
            msg (ObjectArray): ObjectArray message received from the /carla/objects topic
        '''

        # self.get_logger().info(f'Number of objects: {len(msg.objects)}')
        # for obj in msg.objects:
            # self.get_logger().info('='*80)
            # self.get_logger().info(f'Object ID: {obj.id}')
            
            # linear_acceleration_y = obj.accel.linear.y
            # linear_acceleration_x = obj.accel.linear.x
            # linear_acceleration_y = obj.accel.linear.y
            # angular_acceleration_x = obj.accel.angular.x
            # angular_acceleration_y = obj.accel.angular.y
            # self.get_logger().info(f'Position x: {position_x}')
            # self.get_logger().info(f'Position y: {position_y}')
            # self.get_logger().info(
            #     f'Linear acceleration x: {linear_acceleration_x}')
            # self.get_logger().info(
            #     f'Linear acceleration y: {linear_acceleration_y}')
            # self.get_logger().info(
            #     f'Angular acceleration x: {angular_acceleration_x}')
            # self.get_logger().info(
            #     f'Angular acceleration y: {angular_acceleration_y}')

            # 145 is the ID of the ego vehicle
            # if obj.id == 145:
            #     self._ego_current_linear_x = obj.twist.linear.x
            #     self.get_logger().info(f'Linear velocity: {self._ego_current_linear_x}')

            # 154 is the ID of the front vehicle
            # if obj.id == 154:
            #     position_x = obj.pose.position.x
            #     position_y = obj.pose.position.y
            #     self._front_current_position_x = position_x
            #     self._front_current_position_y = position_y
            #     if 4.79 < position_x < 4.81:
            #         if 34.0 < position_y < 34.1:
            #             if self._emergency_ahead_published is False:
            #                 self.get_logger().info('🚨🚨🚨🚨🚨🚨 Front vehicle is stopping')
            #                 self._emergency_ahead_pub.publish(Int64(data=-1))
            #                 self._emergency_ahead_published = True

    def gnss_pose_cb(self, msg: PoseStamped):
        '''
        Callback function for the /sensing/gnss/pose topic
        This function is used to get the current position of the vehicle

        Args:
            msg (PoseStamped): Pose message received from the /sensing/gnss/pose topic
        '''
        # self._ego_current_position_x = msg.pose.position.x
        # self._ego_current_position_y = msg.pose.position.y

    # def _send_emergency_ahead_vehicle(self):
    #     """
    #     Send a signal to the ego vehicle to indicate that there is an emergency ahead
    #     """
    #     if self._current_position_x is None or self._current_position_y is None:
    #         return
    #     distance = ((self._current_position_x - self._stopped_vehicle_x) **
    #                 2 + (self._current_position_y - self._stopped_vehicle_y)**2)**0.5
    #     if distance < 200.0:
    #         # send a signal to the topic /emergency_ahead
    #         self._emergency_ahead_pub.publish(Int64(data=1))
    #     else:
    #         self._emergency_ahead_pub.publish(Int64(data=0))


class VehicleCommander(Node):
    '''
    Class used to send commands to the vehicle.

    Args:
        Node (rclpy.node.Node): Node class from rclpy
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # counter to be used in the method run()
        self._counter = 0

        self._start_sec = 0
        self._current_sec = 0
        self._elapsed_sec = 0
        self._emergency_ahead = False

        # self._stopped_vehicle_x = 7.61
        # self._stopped_vehicle_y = 203.34

        self.hazard_lights_pub = self.create_publisher(
            HazardLightsCommand,
            '/control/command/hazard_lights_cmd',
            10)

        # sync the time with the simulation time
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        # self._vehicle_id = self.declare_parameter(
        #     'vehicle_id', '').get_parameter_value().string_value
        self._vehicle_id = self.get_name()
        self.get_logger().info(f'Vehicle ID: {self._vehicle_id}')

        self._current_position_x = None
        self._current_position_y = None

        # self._engage_client = self.create_client(Engage, '/api/autoware/set/engage')

        # Get parameters for the initial position and orientation
        self._init_pos_x = self.declare_parameter(
            'init_pos_x').get_parameter_value().double_value
        self._init_pos_y = self.declare_parameter(
            'init_pos_y').get_parameter_value().double_value
        self._init_pos_z = self.declare_parameter(
            'init_pos_z').get_parameter_value().double_value
        self._init_rot_x = self.declare_parameter(
            'init_rot_x').get_parameter_value().double_value
        self._init_rot_y = self.declare_parameter(
            'init_rot_y').get_parameter_value().double_value
        self._init_rot_z = self.declare_parameter(
            'init_rot_z').get_parameter_value().double_value
        self._init_rot_w = self.declare_parameter(
            'init_rot_w').get_parameter_value().double_value
        self._emergency_break_distance = self.declare_parameter(
            'emergency_break_distance', 0.0).get_parameter_value().double_value

        # Get parameters for the goal position and orientation
        self._goal_pos_x = self.declare_parameter(
            'goal_pos_x').get_parameter_value().double_value
        self._goal_pos_y = self.declare_parameter(
            'goal_pos_y').get_parameter_value().double_value
        self._goal_pos_z = self.declare_parameter(
            'goal_pos_z').get_parameter_value().double_value
        self._goal_rot_x = self.declare_parameter(
            'goal_rot_x').get_parameter_value().double_value
        self._goal_rot_y = self.declare_parameter(
            'goal_rot_y').get_parameter_value().double_value
        self._goal_rot_z = self.declare_parameter(
            'goal_rot_z').get_parameter_value().double_value
        self._goal_rot_w = self.declare_parameter(
            'goal_rot_w').get_parameter_value().double_value

        # Callback groups
        callback_group1 = MutuallyExclusiveCallbackGroup()
        callback_group2 = MutuallyExclusiveCallbackGroup()
        callback_group3 = MutuallyExclusiveCallbackGroup()
        # challenge_cb_group = MutuallyExclusiveCallbackGroup()

        # self._challenge_timer = self.create_timer(
        #     0.5, self._handle_challenge_timer, callback_group=challenge_cb_group)

        # Subscribe to /sensing/gnss/pose
        self.gnss_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.gnss_pose_cb,
            10,
            callback_group=callback_group1
        )

        # Subscribe to /emergency_ahead
        # The publisher to this topic is located in the class DemoAction
        # self.emergency_subscriber = self.create_subscription(
        #     Int64,
        #     'emergency_ahead',
        #     self.emergency_cb,
        #     10,
        #     callback_group=callback_group1
        # )

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        # Subscribe to /clock
        self._clock_subscriber = self.create_subscription(
            Clock,
            '/clock',
            self._clock_cb,
            qos_profile,
            callback_group=callback_group3
        )

        # Create a publisher for the PoseWithCovarianceStamped messages
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        self.publisher_initial_pose = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic='/initialpose',
            qos_profile=qos_profile)

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            depth=5)

        # Create a publisher for the PoseStamped messages
        self.publisher_goal_pose = self.create_publisher(
            msg_type=PoseStamped,
            topic='/planning/mission_planning/goal',
            qos_profile=qos_profile)

        # # publish elapsed time
        # self._elapsed_time_pub = self.create_publisher(
        #     Int64,
        #     '/commander/elapsed_time',
        #     10)

        self._timer = self.create_timer(
            3, self.run, callback_group=callback_group2)
        self._vehicle_initialized = False
        self._vehicle_goal_set = False
        self._vehicle_engaged = False
        self._scenario_running = False

    def _clock_cb(self, msg: Clock):
        '''
        Callback function for the /clock topic
        '''
        # compute elapsed time

        self._current_sec = msg.clock.sec
        self._elapsed_sec = self._current_sec - self._start_sec
        # publish elapsed time
        elapsed_time_msg = Int64()
        elapsed_time_msg.data = self._elapsed_sec
        # self._elapsed_time_pub.publish(elapsed_time_msg)

        # self.get_logger().info(f'Start seconds: {self._start_sec}')
        # self.get_logger().info(f'Current seconds: {self._current_sec}')
        # self.get_logger().info(f'Elapsed seconds: {self._elapsed_sec}')
        # if not self._emergency_ahead and self._elapsed_sec > 39:
        #     self._emergency_ahead = True

    # def _handle_challenge_timer(self):

    #     # compute the distance between the current position and the goal position
    #     distance = ((self._current_position_x - self._stopped_vehicle_x) **
    #                 2 + (self._current_position_y - self._stopped_vehicle_y)**2)**0.5

    #     self.get_logger().info(f'Distance: {distance}')
    #     # publish distance between ego and front vehicles
    #     distance_msg = Float64()
    #     distance_msg.data = distance
    #     self._distance_pub.publish(distance_msg)

    #     if self._elapsed_sec == 17:
    #         self.emergency_break()

    # def emergency_cb(self, msg: Int64):
    #     """
    #     Callback function for the /emergency_ahead topic
    #     """
    #     if msg.data == -1:
    #         time.sleep(2.04)
    #         self.get_logger().warn('''🚨 🚨 Vehicle stopped ahead''')

    #         self.emergency_break()

    def gnss_pose_cb(self, msg: PoseStamped):
        '''
        Callback function for the /sensing/gnss/pose topic
        This function is used to get the current position of the vehicle

        Args:
            msg (PoseStamped): Pose message received from the /sensing/gnss/pose topic

        NOTE: This function is not currently used
        '''

        # # Emergency break only for the first vehicle
        # if self._vehicle_id != 'rv1':
        #     return

        self._current_position_x = msg.pose.position.x
        self._current_position_y = msg.pose.position.y

    def emergency_break(self):
        '''
        Emergency break
        '''
        msg = HazardLightsCommand()
        msg.command = HazardLightsCommand.ENABLE
        self.hazard_lights_pub.publish(msg)

        client = self.create_client(
            SetEmergency, '/api/autoware/set/emergency')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('..........Emergency service not available, waiting again...')

        request = SetEmergency.Request()
        request.emergency = True

        future = client.call_async(request)
        future.add_done_callback(self.emergency_break_cb)

    def emergency_break_cb(self, future):
        '''
        Callback function for the emergency break service
        '''
        self.get_logger().info(
            f'...Emergency stop status: {future.result().status.code}', once=True)
        self.get_logger().info(
            f'...Emergency stop message: {future.result().status.message}', once=True)

    def engage(self, engage):
        '''
        Engage to drive towards the goal

        Args:
            engage (bool): Whether to engage or not
        '''
        client = self.create_client(Engage, '/api/autoware/set/engage')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('..........Engage service not available, waiting again...')

        request = Engage.Request()
        request.engage = engage

        future = client.call_async(request)
        future.add_done_callback(self.engage_cb)

    def engage_cb(self, future):
        '''
        Callback function for the engage service
        '''
        self.get_logger().info(
            f'..........Engage code: {future.result().response.code}')
        self.get_logger().info(
            f'..........Engage message: {future.result().response.message}')

    def run(self):
        '''
        Main loop of the node
        '''
        # self.get_logger().info(f'Running Vehicle ID: {self._vehicle_id}')
        # if not self._vehicle_initialized:
        if self._counter < 3:
            self.get_logger().info(f'...Initializing {self._vehicle_id}')
            self.initialize()

        # set the goal
        if self._counter == 3:
            # if not self._vehicle_goal_set:
            self.get_logger().info(f'...Setting goal for {self._vehicle_id}')
            self.set_goal()
            self._vehicle_goal_set = True
        # engage
        if self._counter == 4:
            # if not self._vehicle_engaged and self._vehicle_goal_set:
            self.get_logger().info(f'...Engaging {self._vehicle_id}')
            self.engage(True)
            self._vehicle_engaged = True

        # start scenario runner
        # if self._counter == 3:
        #     if not self._scenario_running and self._vehicle_engaged:
        #         self._start_sec = self._current_sec
        #         self.get_logger().info(f'--------- Starting time: {self._start_sec}')
        #         cd_cmd = 'cd /home/zeid/seri_nist_ws/scenario_runner'
        #         scenario_runner_cmd = 'python3 scenario_runner.py --openscenario Reveal.xosc'
        #         os.system(f'{cd_cmd} && {scenario_runner_cmd}')
        #         self._scenario_running = True

        # if not self._vehicle_engaged and self._vehicle_initialized:
        #     self.engage(True)
        #     self._vehicle_engaged = True

        # if not self._vehicle_goal_set and self._vehicle_initialized:
        #     self.set_goal()
        #     self._vehicle_goal_set = True

        # if not self._vehicle_engaged and self._vehicle_goal_set:
        #     self.engage(True)
        #     self._vehicle_engaged = True

        self._counter += 1

    def set_goal(self):
        '''
        Publish a PoseStamped message to the /planning/mission_planning/goal topic
        '''
        # Create a PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self._goal_pos_x
        pose.pose.position.y = self._goal_pos_y
        pose.pose.position.z = self._goal_pos_z
        pose.pose.orientation.x = self._goal_rot_x
        pose.pose.orientation.y = self._goal_rot_y
        pose.pose.orientation.z = self._goal_rot_z
        pose.pose.orientation.w = self._goal_rot_w

        self.get_logger().info(f'...Setting goal for {self._vehicle_id}')

        # if self._vehicle_id == 'rv2':
        self.get_logger().info(f'......Goal x {self._goal_pos_x}')
        self.get_logger().info(f'......Goal y {self._goal_pos_y}')
        self.get_logger().info(f'......Goal z {self._goal_pos_z}')
        self.get_logger().info(f'......Goal Rot x {self._goal_rot_x}')
        self.get_logger().info(f'......Goal Rot y {self._goal_rot_y}')
        self.get_logger().info(f'......Goal Rot z {self._goal_rot_z}')
        self.get_logger().info(f'......Goal Rot w {self._goal_rot_w}')

        # Publish the PoseStamped message
        self.publisher_goal_pose.publish(pose)
        self.get_logger().info(
            f'..........Published mission plan for {self._vehicle_id}')

    def initialize(self):
        '''
        Initialize the ego vehicle in the map
        '''

        # Create a PoseWithCovarianceStamped message
        pose = PoseWithCovarianceStamped()

        # Set the pose values
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self._init_pos_x
        pose.pose.pose.position.y = self._init_pos_y
        pose.pose.pose.position.z = self._init_pos_z
        pose.pose.pose.orientation.x = self._init_rot_x
        pose.pose.pose.orientation.y = self._init_rot_y
        pose.pose.pose.orientation.z = self._init_rot_z
        pose.pose.pose.orientation.w = self._init_rot_w

        # Set the covariance values
        pose.pose.covariance = [0.0] * 36  # Fill covariance matrix with zeros
        pose.pose.covariance[0] = 0.25  # x
        pose.pose.covariance[7] = 0.25  # y
        pose.pose.covariance[35] = 0.06853891945200942  # yaw
        # Publish the PoseWithCovarianceStamped message
        self.publisher_initial_pose.publish(pose)
        # We should probably wait for the vehicle to initialize
        # TODO: wait for the vehicle to initialize by subscribing to /initialpose
        self.get_logger().info(f'..........Initialized {self._vehicle_id}')
