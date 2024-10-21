import time
import math
from collections import namedtuple
import struct
import re


import rclpy
from rclpy.node import Node
import rclpy.qos as qos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Quaternion, TwistWithCovariance, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


import serial
import utm
from transforms3d.quaternions import quat2mat
from transforms3d.euler import euler2quat
import numpy as np

'''
Message binary layouts differed between documentation and source code.
Values from source code used: https://github.com/Aceinna/aceinna_openrtk_ros_driver/blob/master/openrtk_ros/src/protocol/protocol.h
'''


class MsgSpec:

    def __init__(self, name : str, frame_type : bytes, datalength : int, spec):
        assert(type(frame_type)==bytes)
        assert(len(frame_type)==2)
        self.ftype = frame_type

        field_names = []
        self.struct = "="
        size = 0
        for (n,t) in spec:
            field_names.append(n)
            self.struct += t
            if   t in "xcbB?":
                size += 1
            elif t in "hHe":
                size += 2
            elif t in "iIlLf":
                size += 4
            elif t in "qQd":
                size += 8
            else:
                raise(ValueError(f"Field {n} in MSG {name} specifies invalid type: {t}!"))

        assert size == datalength, "Datalength given does not match message specification!"

        self.size = datalength
        self.tuple = namedtuple(name, field_names)

    def __call__(self, data : bytes):
        # parse here
        return self.tuple._make(struct.unpack(self.struct, data))
    
    def type(self):
        return self.ftype
    
    def len(self):
        return self.size


class RTKIMU(MsgSpec):
    def __init__(self, publisher, clock):
        self.pub = publisher
        self.clock = clock
        self.msg = None
        super().__init__('RTKIMU', b'\x73\x31', 30, 
        (   ('gps_week'      , 'H'),
            ('gps_ms'        , 'I'),
            ('x_acceleration', 'f'),
            ('y_acceleration', 'f'),
            ('z_acceleration', 'f'),
            ('x_gyro_rate'   , 'f'),
            ('y_gyro_rate'   , 'f'),
            ('z_gyro_rate'   , 'f'),
        )
        )

    def __call__(self, raw_data : bytes):
        data = super().__call__(raw_data)
        msg = Imu()
        msg.header.frame_id = "openrtk_imu"
        # Use system clock for now
        # Assume the GPS time is sync to clock another way using chrony or ntpd
        msg.header.stamp = self.clock.now().to_msg()

        msg.orientation_covariance = [-1.0] + [0.0]*8

        msg.angular_velocity.x = data.x_gyro_rate
        msg.angular_velocity.y = data.y_gyro_rate
        msg.angular_velocity.z = data.z_gyro_rate
        msg.angular_velocity_covariance = [0.0]*9

        msg.linear_acceleration.x = data.x_acceleration
        msg.linear_acceleration.y = data.y_acceleration
        msg.linear_acceleration.z = data.z_acceleration
        msg.linear_acceleration_covariance = [0.0]*9
        # copy into buffer location
        self.msg = msg

    def publish(self):
        if self.msg is not None:
            self.pub.publish(self.msg)
            self.msg = None

class RTKGNSS(MsgSpec):
    def __init__(self, publisher, clock):
        self.pub = publisher
        self.clock = clock
        self.msg = None
        super().__init__('RTKGNSS', b'\x67\x31', 77, 
        (   ('gps_week'                 , 'H'),
            ('gps_ms'                   , 'I'),
            ('position_type'            , 'B'),
            ('latitude'                 , 'd'),
            ('longitude'                , 'd'),
            ('height'                   , 'd'),
            ('latitude_std_deviation'   , 'f'),
            ('longitude_std_deviation'  , 'f'),
            ('height_std_deviation'     , 'f'),
            ('num_of_satellites'        , 'B'),
            ('num_satellite_in_solution', 'B'),
            ('hdop'                     , 'f'),
            ('diffage'                  , 'f'),
            ('north_vel'                , 'f'),
            ('east_vel'                 , 'f'),
            ('up_vel'                   , 'f'),
            ('north_vel_std_deviation'  , 'f'),
            ('east_vel_std_deviation'   , 'f'),
            ('up_vel_std_deviation'     , 'f'),
        )
        )

    def __call__(self, raw_data : bytes):
        data = super().__call__(raw_data)
        msg = NavSatFix()
        msg.header.frame_id = "openrtk_antenna"
        # Use system clock for now
        # Assume the GPS time is sync to clock another way using chrony or ntpd
        msg.header.stamp = self.clock.now().to_msg()

        # NavSatStatus
        # Choices: STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX, STATUS_GBAS_FIX
        if   data.position_type==1:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif data.position_type==4:
            msg.status.status = NavSatStatus.STATUS_FIX
        elif data.position_type==5:
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        # Choices: SERVICE_GPS, SERVICE_GLONASS, SERVICE_COMPASS, SERVICE_GALILEO
        # msg doesn't provide this so flub it...
        msg.status.service = NavSatStatus.SERVICE_GPS


        msg.latitude  = data.latitude
        msg.longitude = data.longitude
        msg.altitude  = data.height
        # ENU order, so long, lat, height
        msg.position_covariance = [ data.longitude_std_deviation**2,                             0.0,                          0.0,
                                                               0.0, data.latitude_std_deviation**2,                          0.0,
                                                               0.0,                             0.0, data.height_std_deviation**2]
        
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # copy into buffer location
        self.msg = msg

    def publish(self):
        if self.msg is not None:
            self.pub.publish(self.msg)
            self.msg = None

class RTKINS(MsgSpec):
    def __init__(self, publisher, clock):
        self.pub = publisher
        self.clock = clock
        self.msg = None
        self.zone = None
        super().__init__('RTKINS', b'\x69\x31', 116, 
        (   ('gps_week'         , 'H'),
            ('gps_ms'           , 'I'),
            ('ins_status'       , 'B'),
            ('ins_type'         , 'B'),
            ('lattitude'        , 'd'),
            ('longitude'        , 'd'),
            ('altitude'         , 'd'),
            ('north_velocity'   , 'd'),
            ('east_velocity'    , 'd'),
            ('up_velocity'      , 'd'),
            ('roll'             , 'd'),
            ('pitch'            , 'd'),
            ('heading'          , 'd'),
            ('lat_std'          , 'f'),
            ('lon_std'          , 'f'),
            ('alt_std'          , 'f'),
            ('north_vel_std'    , 'f'),
            ('east_vel_std'     , 'f'),
            ('up_vel_std'       , 'f'),
            ('roll_std'         , 'f'),
            ('pitch_std'        , 'f'),
            ('heading_std'      , 'f')
        )
        )

    def __call__(self, raw_data : bytes):
        data = super().__call__(raw_data)
        msg = Odometry()
        msg.header.frame_id = "utm"
        # Use system clock for now
        # Assume the GPS time is sync to clock another way using chrony or ntpd
        msg.header.stamp = self.clock.now().to_msg()

        msg.child_frame_id = "ins_link"

        # Just in case, project to same UTM grid
        if self.zone is None:
            coords = utm.from_latlon(data.lattitude, data.longitude)
            self.zone = coords[2:]
        else:
            coords = utm.from_latlon(data.lattitude, data.longitude, self.zone[0], self.zone[1])

        msg.pose.pose.position.x = coords[0]
        msg.pose.pose.position.y = coords[1]
        msg.pose.pose.position.z = data.altitude
        q = euler2quat(data.roll, data.pitch, data.heading, "sxyz")
        msg.pose.pose.orientation.w = q[0]
        msg.pose.pose.orientation.x = q[1]
        msg.pose.pose.orientation.y = q[2]
        msg.pose.pose.orientation.z = q[3]

        msg.pose.covariance = [data.lat_std] + [0.0]*6 + [data.lon_std] + [0.0]*6 + [data.alt_std] + [0.0]*6 + \
                              [data.pitch_std]+[0.0]*6 + [data.roll_std]+ [0.0]*6 + [data.heading_std]

        # velocity is tricky as it's supplied ENU and needs to be wrt vehicle coords
        # solution: pack into twist then transform it

        msg.twist.twist.linear.x = data.east_velocity
        msg.twist.twist.linear.y = data.north_velocity
        msg.twist.twist.linear.z = data.up_velocity
        msg.twist.covariance = [data.east_vel_std] + [0.0]*6 + [data.north_vel_std] + [0.0]*6 + [data.up_vel_std] + [0.0]*21
        msg.twist = self.rotate_twist(msg.twist, msg.pose.pose.orientation)
        self.msg = msg

    def publish(self):
        if self.msg is not None:
            self.pub.publish(self.msg)
            self.msg = None

    def rotate_twist(self, twist, quat):
        rot = quat2mat([
            quat.w,
            quat.x,
            quat.y,
            quat.z,
        ])

        vels = np.dot(rot, np.array([twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z]).reshape((3,1)))
        v_cov = list(np.dot(rot,
            np.array(twist.covariance[:9]).reshape((3,3))
        ).reshape(9,))
        ret = TwistWithCovariance()
        ret.twist.linear.x = vels[0, 0]
        ret.twist.linear.y = vels[1, 0]
        ret.twist.linear.z = vels[2, 0]

        #print(v_cov[0:3] + [0.0]*3+ v_cov[3:6] + [0.0]*3 + v_cov[6:9] + [0.0]*21)
        ret.covariance = v_cov[0:3] + [0.0]*3+ v_cov[3:6] + [0.0]*3 + v_cov[6:9] + [0.0]*21
        return ret



class OpenRTK(rclpy.node.Node):
    def __init__(self):
        super().__init__('openrtk_node')

        self.zone = None

        self.declare_parameter('uart_baudrate',460800)
        self.declare_parameter('uart_port_id', '/dev/ttyUSB0')
        self.declare_parameter('uart_timeout',0.1)
        self.declare_parameter('use_utm', False)

        self.timer = self.create_timer(0.1, self.publish)
        
        self.msgs = [
            RTKIMU(self.create_publisher(Imu, 'imu', 10), self.get_clock()),
            RTKGNSS(self.create_publisher(NavSatFix, 'gps_fix', 10), self.get_clock()),
            RTKINS(self.create_publisher(Odometry, 'odom', 10), self.get_clock())
        ]


    def run(self):
        """
        Continually polls and loads data from RTK.

        ROS doesn't support INS sensors well. To address this, we'll hack things a bit.
        The INS LLA will be converted to UTM and the first message used to initialize
        the world frame. This is a bit of an abuse, but it'll get the ball rolling.
        """

        self.use_utm = self.get_parameter('use_utm').get_parameter_value().bool_value

        self.get_logger().info(f"Opening serial port: {self.get_parameter('uart_port_id').get_parameter_value().string_value}")

        # open serial port()
        with serial.Serial(
            self.get_parameter('uart_port_id').get_parameter_value().string_value,
            self.get_parameter('uart_baudrate').get_parameter_value().integer_value,
#            timeout=self.get_parameter('uart_timeout').get_parameter_value().double_value
        ) as port:

            self.first = None
            self.msg = None
            while rclpy.ok():
                # read port up to start of next message
                # this works because Python will short-circuit the expression
                #b = port.read(1)
                #self.get_logger().info(''.join([hex(x) for x in b]))
                #self.get_logger().info(str(b))
                #if b == b'\x55' and port.read(1) == b'\x55':
                if port.read_until(b'\x55\x55'):
                    # just read the \x55\x55 header
                    # next is 2-byte frame type
                    frame_type = port.read(2)
                    # Line below commented out to avoid command line clogging
                    # self.get_logger().info(f"Frame type received: {str(frame_type)} ({''.join([hex(x) for x in frame_type])})")
                    # check that frame type corresponds to supported message
                    if any(map(lambda x: x.ftype==frame_type, self.msgs)):
                        # now data length
                        data_len = int.from_bytes(port.read(1), "little")
                        data = port.read(data_len)
                        checksum = port.read(2)
                        # TODO validate checksum
                        if data_len == len(data) or self.validate_checksum(data, checksum):
                            for parser in filter(lambda x: x.ftype==frame_type, self.msgs):
                                self.get_logger().info(str(type(parser)))
                                # parse data and pack into message
                                parser(data)

                    else:
                        # report the bad frame type
                        pass


                rclpy.spin_once(self)

    def publish(self):
        """
        Publishes latest message.
        """
        for parser in self.msgs:
            parser.publish()


def main():
    rclpy.init()
    node = OpenRTK()
    node.run()

if __name__ == '__main__':
    main()
