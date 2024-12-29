from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from microstrain_inertial_msgs.msg import GNSSAidingStatus, GNSSFixInfo, GNSSDualAntennaStatus, FilterStatus, RTKStatusV1, RTKStatus, FilterAidingMeasurementSummary

from .constants import _DEFAULT_VAL, _DEFAULT_STR
from .constants import _UNIT_DEGREES, _UNIT_GS, _UNIT_GUASSIAN, _UNIT_METERS, _UNIT_RADIANS, _UNIT_METERS_PER_SEC, _UNIT_RADIANS_PER_SEC
from .constants import _ICON_GREY_UNCHECKED_MEDIUM, _ICON_YELLOW_UNCHECKED_MEDIUM, _ICON_YELLOW_CHECKED_MEDIUM, _ICON_GREEN_UNCHECKED_MEDIUM,_ICON_GREEN_CHECKED_MEDIUM, _ICON_TEAL_UNCHECKED_MEDIUM, _ICON_TEAL_CHECKED_MEDIUM, _ICON_BLUE_UNCHECKED_MEDIUM, _ICON_BLUE_CHECKED_MEDIUM, _ICON_RED_UNCHECKED_MEDIUM, _ICON_RED_CHECKED_MEDIUM
from .common import SubscriberMonitor


class GNSSAidingStatusMonitor(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name):
    super(GNSSAidingStatusMonitor, self).__init__(node, node_name, topic_name, GNSSAidingStatus)

  @property
  def tight_coupling(self):
    return self._get_val(self._current_message.tight_coupling)

  @property
  def differential_corrections(self):
    return self._get_val(self._current_message.differential_corrections)

  @property
  def integer_fix(self):
    return self._get_val(self._current_message.integer_fix)

  @property
  def position_fix(self):
    return self._get_val(self._current_message.has_position_fix)
  
  @property
  def tight_coupling_string(self):
    return self._get_small_boolean_icon_string(self.tight_coupling)

  @property
  def differential_corrections_string(self):
    return self._get_small_boolean_icon_string(self.differential_corrections)

  @property
  def integer_fix_string(self):
    return self._get_small_boolean_icon_string(self.integer_fix)

  @property
  def position_fix_string(self):
    return self._get_small_boolean_icon_string(self.position_fix)


class GNSSFixInfoMonitor(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name):
    super(GNSSFixInfoMonitor, self).__init__(node, node_name, topic_name, GNSSFixInfo)

  @property
  def fix_type(self):
    return self._get_val(self._current_message.fix_type)

  @property
  def num_sv(self):
    return self._get_val(self._current_message.num_sv)

  @property
  def fix_type_string(self):
    fix_type = self.fix_type
    if fix_type is not _DEFAULT_VAL:
      if fix_type == GNSSFixInfo.FIX_3D:
        return "3D Fix (%d)" % fix_type
      elif fix_type == GNSSFixInfo.FIX_2D:
        return "2D Fix (%d)" % fix_type
      elif fix_type == GNSSFixInfo.FIX_TIME_ONLY:
        return "Time Only (%d)" % fix_type
      elif fix_type == GNSSFixInfo.FIX_NONE:
        return "None (%d)" % fix_type
      elif fix_type == GNSSFixInfo.FIX_INVALID:
        return "Invalid Fix (%d)" % fix_type
      elif fix_type == GNSSFixInfo.FIX_RTK_FLOAT:
        return "RTK Float (%d)" % fix_type
      elif fix_type == GNSSFixInfo.FIX_RTK_FIXED:
        return "RTK Fixed (%d)" % fix_type
      else:
        return "Invalid (%d)" % fix_type
    else:
      return _DEFAULT_STR

  @property
  def num_sv_string(self):
    return self._get_string(self.num_sv)

class FilterStatusMonitor(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name, device_report_monitor):
    super(FilterStatusMonitor, self).__init__(node, node_name, topic_name, FilterStatus)

    # Save a copy of the device report monitor
    self._device_report_monitor = device_report_monitor
  
  @property
  def filter_state(self):
    return self._get_val(self._current_message.filter_state)

  @property
  def status_flags(self):
    return self._get_val(self._current_message.status_flags)

  @property
  def filter_state_string(self):
    filter_state = self.filter_state
    if filter_state is not _DEFAULT_VAL:
      # Different status codes between GQ7 and GX5
      if self._device_report_monitor.is_gq7:
        if filter_state == 1:
          return "GQ7 Init (%d)" % filter_state
        elif filter_state == 2:
          return "GQ7 Vertical Gyro (%d)" % filter_state
        elif filter_state == 3:
          return "GQ7 AHRS (%d)" % filter_state
        elif filter_state == 4:
          return "GQ7 Full Nav (%d)" % filter_state
        else:
          return "GQ7 Invalid (%d)" % filter_state
      elif self._device_report_monitor.is_gx5:
        if filter_state == 0:
          return "GX5 Startup (%d)" % filter_state
        elif filter_state == 1:
          return "GX5 Initialization (%d)" % filter_state
        elif filter_state == 2:
          return "GX5 Running, Solution Valid (%d)" % filter_state
        elif filter_state == 3:
          return "GX5 Running, Solution Error (%d)" % filter_state
      else:
        return "Unknown Device (%d)" % filter_state
    else:
      return _DEFAULT_STR

  @property
  def filter_state_led_string(self):
    filter_state_string = self.filter_state_string
    if filter_state_string is not _DEFAULT_STR:
      # Trim some characters off of the string
      return filter_state_string[4:-4]
    else:
      return filter_state_string

  @property
  def status_flags_string(self):
    status_flags = self.status_flags
    if status_flags is not _DEFAULT_VAL:
      status_str = ""
      if self._device_report_monitor.is_gq7:
        # Filter condition
        if status_flags & 0b11 == 1:
          status_str += "Stable,"
        elif status_flags & 0b11 == 2:
          status_str += "Converging,"
        elif status_flags & 0b11 == 3:
          status_str += "Unstable/Recovering,"

        # Estimate Warnings
        if status_flags & 0b100 != 0:
          status_str += "Roll/Pitch Warning,"
        if status_flags & 0b1000 != 0:
          status_str += "Heading Warning,"
        if status_flags & 0b10000 != 0:
          status_str += "Position Warning,"
        if status_flags & 0b100000 != 0:
          status_str += "Velocity Warning,"
        if status_flags & 0b1000000 != 0:
          status_str += "IMU Bias Warning,"
        if status_flags & 0b10000000 != 0:
          status_str += "GNSS Clock Warning,"
        if status_flags & 0b100000000 != 0:
          status_str += "Antenna Lever Arm Warning,"
        if status_flags & 0b1000000000 != 0:
          status_str += "Mounting Transform Warning,"
        
        # Solution Error
        if status_flags & 0b1111000000000000 != 0:
          status_str += "Solution Error,"
      elif self._device_report_monitor.is_gx5:
        # GX5 needs the filter state to make sense of the status flags
        filter_state = self.filter_state
        if filter_state is not _DEFAULT_VAL:
          # Initialization flags
          if filter_state == 1:
            if status_flags & 0b1000000000000 != 0:
              status_str += "Attutude not initialized,"
            if status_flags & 0b10000000000000 != 0:
              status_str += "Position & Velocity not initialized,"

          # Running with error flags
          elif filter_state in (2, 3):
            if status_flags & 0b1 != 0:
              status_str += "IMU unavailable,"
            if status_flags & 0b10 != 0:
              status_str += "GNSS,"
            if status_flags & 0b1000 != 0:
              status_str += "Matrix Singularity in calculation,"
            if status_flags & 0b10000 != 0:
              status_str += "Position covariance high warning,"
            if status_flags & 0b100000 != 0:
              status_str += "Velocity covariance high warning,"
            if status_flags & 0b1000000 != 0:
              status_str += "Attitude covariance high warning,"
            if status_flags & 0b10000000 != 0:
              status_str += "NAN in solution,"
            if status_flags & 0b100000000 != 0:
              status_str += "Gyro bias estimate high warning,"
            if status_flags & 0b1000000000 != 0:
              status_str += "Accel bias estimate high warning,"
            if status_flags & 0b10000000000 != 0:
              status_str += "Gyro scale factor estimate high warning,"
            if status_flags & 0b100000000000 != 0:
              status_str += "Accel scale factor estimate high warning,"
            if status_flags & 0b1000000000000 != 0:
              status_str += "Mag bias estimate high warning,"
            if status_flags & 0b10000000000000 != 0:
              status_str += "GNSS antenna offset correction estimate high warning,"
            if status_flags & 0b100000000000000 != 0:
              status_str += "Hard Iron offset estimate high warning,"
            if status_flags & 0b1000000000000000 != 0:
              status_str += "Soft iron correction estimate high warning,"
            if not status_str:
              status_str = "None,"
          
          # No flags if any other case
          else:
            status_str = "None,"
        else:
          status_str = "Unknown Filter State,"
      else:
        status_str = "Uknown Device,"

      # Trim the last comma and return
      if status_str:
        return "%s (%d)" % (status_str[:-1], status_flags)
      else:
        return "Unknown Status Flags (%d)" % status_flags
    else:
      return _DEFAULT_STR
    

class OdomMonitor(SubscriberMonitor):

  _MIN_COVARIANCE_SIZE = 36

  def __init__(self, node, node_name, topic_name, llh=True):
    super(OdomMonitor, self).__init__(node, node_name, topic_name, Odometry, callback=self._callback)

    # Use different units if we are using LLH versus ECEF
    if llh:
      self._xy_units = _UNIT_DEGREES
    else:
      self._xy_units = _UNIT_METERS

    # Initialize some member variables
    self._current_roll = _DEFAULT_VAL
    self._current_pitch = _DEFAULT_VAL
    self._current_yaw = _DEFAULT_VAL

  def _callback(self, status_message):
    quaternion = [self._current_message.pose.pose.orientation.x, self._current_message.pose.pose.orientation.y, self._current_message.pose.pose.orientation.z, self._current_message.pose.pose.orientation.w]
    self._current_roll, self._current_pitch, self._current_yaw = self._euler_from_quaternion(quaternion)
    super(OdomMonitor, self)._default_callback(status_message)

  @property
  def position_x(self):
    return self._get_val(self._current_message.pose.pose.position.x)

  @property
  def position_y(self):
    return self._get_val(self._current_message.pose.pose.position.y)

  @property
  def position_z(self):
    return self._get_val(self._current_message.pose.pose.position.z)

  @property
  def position_uncertainty_x(self):
    if len(self._current_message.pose.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.pose.covariance[0])
    else:
      return _DEFAULT_VAL

  @property
  def position_uncertainty_y(self):
    if len(self._current_message.pose.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.pose.covariance[7])
    else:
      return _DEFAULT_VAL

  @property
  def position_uncertainty_z(self):
    if len(self._current_message.pose.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.pose.covariance[14])
    else:
      return _DEFAULT_VAL

  @property
  def orientation_x(self):
    return self._get_val(self._current_roll)

  @property
  def orientation_y(self):
    return self._get_val(self._current_pitch)

  @property
  def orientation_z(self):
    return self._get_val(self._current_yaw)
  
  @property
  def orientation_uncertainty_x(self):
    if len(self._current_message.pose.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.pose.covariance[21])
    else:
      return _DEFAULT_VAL

  @property
  def orientation_uncertainty_y(self):
    if len(self._current_message.pose.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.pose.covariance[28])
    else:
      return _DEFAULT_VAL

  @property
  def orientation_uncertainty_z(self):
    if len(self._current_message.pose.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.pose.covariance[35])
    else:
      return _DEFAULT_VAL

  @property
  def velocity_x(self):
    return self._get_val(self._current_message.twist.twist.linear.x)

  @property
  def velocity_y(self):
    return self._get_val(self._current_message.twist.twist.linear.y)

  @property
  def velocity_z(self):
    return self._get_val(self._current_message.twist.twist.linear.z)
  
  @property
  def velocity_uncertainty_x(self):
    if len(self._current_message.twist.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.twist.covariance[0])
    else:
      return _DEFAULT_VAL

  @property
  def velocity_uncertainty_y(self):
    if len(self._current_message.twist.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.twist.covariance[7])
    else:
      return _DEFAULT_VAL

  @property
  def velocity_uncertainty_z(self):
    if len(self._current_message.twist.covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.twist.covariance[14])
    else:
      return _DEFAULT_VAL

  @property
  def position_x_string(self):
    return self._get_string_units(self.position_x, self._xy_units)

  @property
  def position_y_string(self):
    return self._get_string_units(self.position_y, self._xy_units)

  @property
  def position_z_string(self):
    return self._get_string_units(self.position_x, _UNIT_METERS)

  @property
  def position_uncertainty_x_string(self):
    return self._get_string_units(self.position_uncertainty_x, _UNIT_METERS)

  @property
  def position_uncertainty_y_string(self):
    return self._get_string_units(self.position_uncertainty_y, _UNIT_METERS)

  @property
  def position_uncertainty_z_string(self):
    return self._get_string_units(self.position_uncertainty_z, _UNIT_METERS)

  @property
  def orientation_x_string(self):
    return self._get_string_units(self.orientation_x, _UNIT_RADIANS)

  @property
  def orientation_y_string(self):
    return self._get_string_units(self.orientation_y, _UNIT_RADIANS)

  @property
  def orientation_z_string(self):
    return self._get_string_units(self.orientation_z, _UNIT_RADIANS)

  @property
  def orientation_uncertainty_x_string(self):
    return self._get_string_units(self.orientation_uncertainty_x, _UNIT_RADIANS)

  @property
  def orientation_uncertainty_y_string(self):
    return self._get_string_units(self.orientation_uncertainty_y, _UNIT_RADIANS)

  @property
  def orientation_uncertainty_z_string(self):
    return self._get_string_units(self.orientation_uncertainty_z, _UNIT_RADIANS)

  @property
  def velocity_x_string(self):
    return self._get_string_units(self.velocity_x, _UNIT_METERS_PER_SEC)

  @property
  def velocity_y_string(self):
    return self._get_string_units(self.velocity_y, _UNIT_METERS_PER_SEC)

  @property
  def velocity_z_string(self):
    return self._get_string_units(self.velocity_z, _UNIT_METERS_PER_SEC)

  @property
  def velocity_uncertainty_x_string(self):
    return self._get_string_units(self.velocity_uncertainty_x, _UNIT_METERS_PER_SEC)

  @property
  def velocity_uncertainty_y_string(self):
    return self._get_string_units(self.velocity_uncertainty_y, _UNIT_METERS_PER_SEC)

  @property
  def velocity_uncertainty_z_string(self):
    return self._get_string_units(self.velocity_uncertainty_z, _UNIT_METERS_PER_SEC)


class GNSSDualAntennaStatusMonitor(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name):
    super(GNSSDualAntennaStatusMonitor, self).__init__(node, node_name, topic_name, GNSSDualAntennaStatus)
  
  @property
  def fix_type(self):
    return self._get_val(self._current_message.fix_type)

  @property
  def heading(self):
    return self._get_val(self._current_message.heading)

  @property
  def heading_uncertainty(self):
    return self._get_val(self._current_message.heading_uncertainty)
  
  @property
  def rec_1_data_valid(self):
    return bool(self._get_val(self._current_message.rcv_1_valid))

  @property
  def rec_2_data_valid(self):
    return bool(self._get_val(self._current_message.rcv_2_valid))
  
  @property
  def antenna_offsets_valid(self):
    return bool(self._get_val(self._current_message.antenna_offsets_valid))

  @property
  def fix_type_string(self):
    fix_type = self.fix_type
    if fix_type is not _DEFAULT_VAL:
      if fix_type == 0:
        return "None (%d)" % fix_type
      elif fix_type == 1:
        return "Float (%d)" % fix_type
      elif fix_type == 2:
        return "Fixed (%d)" % fix_type
      else:
        return _DEFAULT_STR
    else:
      return _DEFAULT_STR
  
  @property
  def heading_string(self):
    return self._get_string_units(self.heading, _UNIT_RADIANS)

  @property
  def heading_uncertainty_string(self):
    return self._get_string_units(self.heading_uncertainty, _UNIT_RADIANS)

  @property
  def rec_1_data_valid_string(self):
    return self._get_small_boolean_icon_string(self.rec_1_data_valid)

  @property
  def rec_2_data_valid_string(self):
    return self._get_small_boolean_icon_string(self.rec_2_data_valid)

  @property
  def antenna_offsets_valid_string(self):
    return self._get_small_boolean_icon_string(self.antenna_offsets_valid)


class ImuMonitor(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name):
    super(ImuMonitor, self).__init__(node, node_name, topic_name, Imu)

  @property
  def accel_x(self):
    return self._get_val(self._current_message.linear_acceleration.x)
  
  @property
  def accel_y(self):
    return self._get_val(self._current_message.linear_acceleration.y)
  
  @property
  def accel_z(self):
    return self._get_val(self._current_message.linear_acceleration.z)
  
  @property
  def vel_x(self):
    return self._get_val(self._current_message.angular_velocity.x)
  
  @property
  def vel_y(self):
    return self._get_val(self._current_message.angular_velocity.y)
  
  @property
  def vel_z(self):
    return self._get_val(self._current_message.angular_velocity.z)

  @property
  def accel_x_string(self):
    return self._get_string_units(self.accel_x, _UNIT_GS)

  @property
  def accel_y_string(self):
    return self._get_string_units(self.accel_y, _UNIT_GS)

  @property
  def accel_z_string(self):
    return self._get_string_units(self.accel_z, _UNIT_GS)

  @property
  def vel_x_string(self):
    return self._get_string_units(self.vel_x, _UNIT_RADIANS_PER_SEC)

  @property
  def vel_y_string(self):
    return self._get_string_units(self.vel_y, _UNIT_RADIANS_PER_SEC)

  @property
  def vel_z_string(self):
    return self._get_string_units(self.vel_z, _UNIT_RADIANS_PER_SEC)


class MagMonitor(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name):
    super(MagMonitor, self).__init__(node, node_name, topic_name, MagneticField)
  
  @property
  def x(self):
    return self._get_val(self._current_message.magnetic_field.x)

  @property
  def y(self):
    return self._get_val(self._current_message.magnetic_field.y)

  @property
  def z(self):
    return self._get_val(self._current_message.magnetic_field.z)

  @property
  def x_string(self):
    return self._get_string_units(self.x, _UNIT_GUASSIAN)

  @property
  def y_string(self):
    return self._get_string_units(self.y, _UNIT_GUASSIAN)

  @property
  def z_string(self):
    return self._get_string_units(self.z, _UNIT_GUASSIAN)


class NavSatFixMonitor(SubscriberMonitor):

  _MIN_COVARIANCE_SIZE = 9

  def __init__(self, node, node_name, topic_name):
    super(NavSatFixMonitor, self).__init__(node, node_name, topic_name, NavSatFix)

  @property
  def position_uncertainty(self):
    if len(self._current_message.position_covariance) >= self._MIN_COVARIANCE_SIZE:
      return self._get_val(self._current_message.position_covariance[0])
    else:
      return _DEFAULT_VAL
  
  @property
  def position_uncertainty_string(self):
    return self._get_string_units(self.position_uncertainty, _UNIT_METERS)


class RTKMonitorBase(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name, message_type):
    super(RTKMonitorBase, self).__init__(node, node_name, topic_name, message_type)

  @property
  def gps_received(self):
    epoch_status = self._current_message.epoch_status
    if epoch_status is not _DEFAULT_VAL:
      return epoch_status & 0b100 != 0
    else:
      return _DEFAULT_VAL

  @property
  def glonass_received(self):
    epoch_status = self._current_message.epoch_status
    if epoch_status is not _DEFAULT_VAL:
      return epoch_status & 0b1000 != 0
    else:
      return _DEFAULT_VAL

  @property
  def galileo_received(self):
    epoch_status = self._current_message.epoch_status
    if epoch_status is not _DEFAULT_VAL:
      return epoch_status & 0b10000 != 0
    else:
      return _DEFAULT_VAL

  @property
  def beidou_received(self):
    epoch_status = self._current_message.epoch_status
    if epoch_status is not _DEFAULT_VAL:
      return epoch_status & 0b100000 != 0
    else:
      return _DEFAULT_VAL
  
  @property
  def version(self):
    return self._get_val(self._current_message.dongle_version)

  @property
  def raw_status_flags(self):
    return self._get_val(self._current_message.raw_status_flags)

  @property
  def signal_quality(self):
    return self._get_val(self._current_message.dongle_signal_quality)

  @property
  def gps_received_string(self):
    return self._get_small_boolean_icon_string(self.gps_received)

  @property
  def glonass_received_string(self):
    return self._get_small_boolean_icon_string(self.glonass_received)

  @property
  def galileo_received_string(self):
    return self._get_small_boolean_icon_string(self.galileo_received)

  @property
  def beidou_received_string(self):
    return self._get_small_boolean_icon_string(self.beidou_received)

  @property
  def version_string(self):
    return self._get_string(self.version)

  @property
  def raw_status_flags_string(self):
    return self._get_string_hex(self.raw_status_flags)

  @property
  def signal_quality_string(self):
    return self._get_string(self.signal_quality)
  
  @property
  def rtk_led_string(self):
    pass


class RTKMonitor(RTKMonitorBase):

  def __init__(self, node, node_name, topic_name):
    super(RTKMonitor, self).__init__(node, node_name, topic_name, RTKStatus)

  @property
  def modem_state(self):
    return self._get_val(self._current_message.dongle_modem_state)

  @property
  def connection_type(self):
    return self._get_val(self._current_message.dongle_connection_type)

  @property
  def rssi(self):
    rssi = self._current_message.dongle_rssi
    if rssi is not _DEFAULT_VAL:
      return -rssi
    else:
      return _DEFAULT_VAL

  @property
  def tower_change_indicator(self):
    tower_change_indicator = self._current_message.dongle_tower_change_indicator
    if tower_change_indicator is not _DEFAULT_VAL:
      return tower_change_indicator
    else:
      return _DEFAULT_VAL

  @property
  def nmea_timeout(self):
    nmea_timeout = self._current_message.dongle_nmea_timeout
    if nmea_timeout is not _DEFAULT_VAL:
      return nmea_timeout
    else:
      return _DEFAULT_VAL

  @property
  def server_timeout(self):
    server_timeout = self._current_message.dongle_server_timeout
    if server_timeout is not _DEFAULT_VAL:
      return server_timeout
    else:
      return _DEFAULT_VAL

  @property
  def rtcm_timeout(self):
    rtcm_timeout = self._current_message.dongle_rtcm_timeout
    if rtcm_timeout is not _DEFAULT_VAL:
      return rtcm_timeout
    else:
      return _DEFAULT_VAL

  @property
  def out_of_range(self):
    out_of_range = self._current_message.dongle_out_of_range
    if out_of_range is not _DEFAULT_VAL:
      return out_of_range
    else:
      return _DEFAULT_VAL

  @property
  def corrections_unavailable(self):
    corrections_unavailable = self._current_message.dongle_corrections_unavailable
    if corrections_unavailable is not _DEFAULT_VAL:
      return corrections_unavailable
    else:
      return _DEFAULT_VAL

  @property
  def modem_state_string(self):
    modem_state = self.modem_state
    if modem_state is not _DEFAULT_VAL:
      if modem_state == self._current_message.MODEM_STATE_OFF:
        return "Off (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_NO_NETWORK:
        return "No Network (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_NETWORK_CONNECTED:
        return "Connected (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_CONFIGURING_DATA_CONTEXT:
        return "Configuring Data Context (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_ACTIVATING_DATA_CONTEXT:
        return "Activating Data Context (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_CONFIGURING_SOCKET:
        return "Configuring Socket (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_WAITING_ON_SERVER_HANDSHAKE:
        return "Waiting on Server Handshake (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_CONNECTED_AND_IDLE:
        return "Connected & Idle (%d)" % modem_state
      elif modem_state == self._current_message.MODEM_STATE_CONNECTED_AND_STREAMING:
        return "Connected & Streaming (%d)" % modem_state
      else:
        return "Invalid (%d)" % modem_state
    else:
      return _DEFAULT_STR

  @property
  def connection_type_string(self):
    connection_type = self.connection_type
    if connection_type is not _DEFAULT_VAL:
      if connection_type == self._current_message.CONNECTION_TYPE_NO_CONNECTION:
        return "No Connection (%d)" % connection_type
      elif connection_type == self._current_message.CONNECTION_TYPE_CONNECTION_2G:
        return "2G (%d)" % connection_type
      elif connection_type == self._current_message.CONNECTION_TYPE_CONNECTION_3G:
        return "3G (%d)" % connection_type
      elif connection_type == self._current_message.CONNECTION_TYPE_CONNECTION_4G:
        return "4G (%d)" % connection_type
      elif connection_type == self._current_message.CONNECTION_TYPE_CONNECTION_5G:
        return "5G (%d)" % connection_type
      else:
        return "Invalid (%d)" % connection_type
    else:
      return _DEFAULT_STR

  @property
  def rssi_string(self):
    return self._get_string(self.rssi)

  @property
  def tower_change_indicator_string(self):
    return self._get_string(self.tower_change_indicator)

  @property
  def nmea_timeout_string(self):
    return self._get_boolean_string(self.nmea_timeout)

  @property
  def server_timeout_string(self):
    return self._get_boolean_string(self.server_timeout)

  @property
  def rtcm_timeout_string(self):
    return self._get_boolean_string(self.rtcm_timeout)

  @property
  def out_of_range_string(self):
    return self._get_boolean_string(self.out_of_range)

  @property
  def corrections_unavailable_string(self):
    return self._get_boolean_string(self.corrections_unavailable)

  @property
  def rtk_led_string(self):
    modem_state = self.modem_state
    if modem_state is not _DEFAULT_VAL:      
      if modem_state == self._current_message.MODEM_STATE_NO_NETWORK:
        return _ICON_YELLOW_UNCHECKED_MEDIUM
      elif modem_state == self._current_message.MODEM_STATE_NETWORK_CONNECTED:
        return _ICON_GREEN_CHECKED_MEDIUM
      elif modem_state == self._current_message.MODEM_STATE_CONFIGURING_DATA_CONTEXT:
        return _ICON_GREEN_CHECKED_MEDIUM
      elif modem_state == self._current_message.MODEM_STATE_ACTIVATING_DATA_CONTEXT:
        return _ICON_GREEN_CHECKED_MEDIUM
      elif modem_state == self._current_message.MODEM_STATE_CONFIGURING_SOCKET:
        return _ICON_GREEN_CHECKED_MEDIUM
      elif modem_state == self._current_message.MODEM_STATE_WAITING_ON_SERVER_HANDSHAKE:
        return _ICON_GREEN_CHECKED_MEDIUM
      elif modem_state == self._current_message.MODEM_STATE_CONNECTED_AND_IDLE:
        return _ICON_BLUE_CHECKED_MEDIUM
      elif modem_state == self._current_message.MODEM_STATE_CONNECTED_AND_STREAMING:
        return _ICON_BLUE_CHECKED_MEDIUM
    else:
      return _ICON_GREY_UNCHECKED_MEDIUM


class RTKMonitorV1(RTKMonitorBase):

  def __init__(self, node, node_name, topic_name):
    super(RTKMonitorV1, self).__init__(node, node_name, topic_name, RTKStatusV1)

  @property
  def controller_state(self):
    return self._get_val(self._current_message.dongle_controller_state)

  @property
  def controller_status(self):
    return self._get_val(self._current_message.dongle_controller_status)

  @property
  def platform_state(self):
    return self._get_val(self._current_message.dongle_platform_state)
  
  @property
  def platform_status(self):
    return self._get_val(self._current_message.dongle_platform_status)

  @property
  def reset_reason(self):
    return self._get_val(self._current_message.dongle_reset_reason)

  @property
  def controller_state_string(self):
    controller_state = self.controller_state
    if controller_state is not _DEFAULT_VAL:
      if controller_state == 0:
        return "Idle (%d)" % controller_state
      elif controller_state == 4:
        return "Active (%d)" % controller_state
      else:
        return "Invalid (%d)" % controller_state
    else:
      return _DEFAULT_STR

  @property
  def controller_status_string(self):
    controller_status = self.controller_status
    if controller_status is not _DEFAULT_VAL:
      if controller_status == 0:
        return "Controller OK (%d)" % controller_status
      elif controller_status == 1:
        return "Awaiting NMEA (%d)" % controller_status
      elif controller_status == 2:
        return "RTK Timed Out (%d)" % controller_status
      elif controller_status == 3:
        return "RTK Unavailable (%d)" % controller_status
      elif controller_status == 7:
        return "Invalid Configuration (%d)" % controller_status
      else:
        return "Invalid (%d)" % controller_status
    else:
      return _DEFAULT_STR

  @property
  def platform_state_string(self):
    platform_state = self.platform_state
    if platform_state is not _DEFAULT_VAL:
      if platform_state == 0:
        return "Modem Powered Down (%d)" % platform_state
      elif platform_state == 1:
        return "Modem Powering Up (%d)" % platform_state
      elif platform_state == 2:
        return "Configuring Modem (%d)" % platform_state
      elif platform_state == 3:
        return "Modem Powering Down (%d)" % platform_state
      elif platform_state == 4:
        return "Modem Ready (%d)" % platform_state
      elif platform_state == 5:
        return "Connecting to Network (%d)" % platform_state
      elif platform_state == 6:
        return "Disconnecting from Network (%d)" % platform_state
      elif platform_state == 7:
        return "Connected to Network (%d)" % platform_state
      elif platform_state == 8:
        return "Connecting to RTK Service (%d)" % platform_state
      elif platform_state == 9 or platform_state == 10:
        return "Unable to Connect to RTK Service (%d)" % platform_state
      elif platform_state == 11:
        return "Disconnecting From RTK Service (%d)" % platform_state
      elif platform_state == 12:
        return "Connected to RTK Service (%d)" % platform_state
      elif platform_state == 13 or platform_state == 14:
        return "Device Error (%d)"
      else:
        return "Invalid (%d)" % platform_state
    else:
      return _DEFAULT_STR

  @property
  def platform_status_string(self):
    platform_status = self.platform_status
    if platform_status is not _DEFAULT_VAL:
      if platform_status == 0:
        return "Platform OK (%d)" % platform_status
      elif platform_status == 4:
        return "RTK Service Connection Broken (%d)" % platform_status
      elif platform_status == 6:
        return "Cell Connection Dropped (%d)" % platform_status
      elif platform_status == 7:
        return "Modem Error (%d)" % platform_status
      else:
        return "Invalid (%d)" % platform_status
    else:
      return _DEFAULT_STR
  
  @property
  def reset_reason_string(self):
    reset_reason = self.reset_reason
    if reset_reason is not _DEFAULT_VAL:
      if reset_reason == 0:
        return "Power On (%d)" % reset_reason
      elif reset_reason == 1:
        return "Unknown (%d)" % reset_reason
      elif reset_reason == 2:
        return "Software Reset (%d)" % reset_reason
      elif reset_reason == 3:
        return "Hardware Error Reset (%d)" % reset_reason
      else:
        return "Invalid (%d)"
    else:
      return _DEFAULT_STR

  @property
  def rtk_led_string(self):
    platform_state = self.platform_state
    if platform_state is not _DEFAULT_VAL:
      if platform_state == 0:
        return _ICON_GREY_UNCHECKED_MEDIUM
      elif platform_state in (1, 2, 4, 5):
        return _ICON_GREEN_UNCHECKED_MEDIUM
      elif platform_state in (3, 6):
        return _ICON_GREEN_CHECKED_MEDIUM
      elif platform_state in (7, 8, 11, 12):
        low_signal_quality = self.signal_quality is _DEFAULT_VAL or self.signal_quality < 5
        out_of_range = self.controller_status is _DEFAULT_VAL or self.controller_status == 3
        if low_signal_quality:
          return _ICON_TEAL_CHECKED_MEDIUM if out_of_range else _ICON_TEAL_UNCHECKED_MEDIUM
        else:
          return _ICON_BLUE_CHECKED_MEDIUM if out_of_range else _ICON_BLUE_UNCHECKED_MEDIUM
      elif platform_state in (9, 10):
        return _ICON_RED_UNCHECKED_MEDIUM
      elif platform_state in (13, 14):
        return _ICON_RED_CHECKED_MEDIUM
      else:
        return _ICON_GREY_UNCHECKED_MEDIUM
    else:
      return _ICON_GREY_UNCHECKED_MEDIUM


class FilterAidingMeasurementSummaryMonitor(SubscriberMonitor):

  def __init__(self, node, node_name, topic_name):
    super(FilterAidingMeasurementSummaryMonitor, self).__init__(node, node_name, topic_name, FilterAidingMeasurementSummary)

  @property
  def gnss1_enabled(self):
    return self._get_val(self._current_message.gnss1.enabled)

  @property
  def gnss1_used(self):
    return self._get_val(self._current_message.gnss1.used)

  @property
  def gnss2_enabled(self):
    return self._get_val(self._current_message.gnss2.enabled)

  @property
  def gnss2_used(self):
    return self._get_val(self._current_message.gnss2.used)

  @property
  def dual_antenna_enabled(self):
    return self._get_val(self._current_message.dual_antenna.enabled)

  @property
  def dual_antenna_used(self):
    return self._get_val(self._current_message.dual_antenna.used)

  @property
  def heading_enabled(self):
    return self._get_val(self._current_message.heading.enabled)

  @property
  def heading_used(self):
    return self._get_val(self._current_message.heading.used)

  @property
  def pressure_enabled(self):
    return self._get_val(self._current_message.pressure.enabled)

  @property
  def pressure_used(self):
    return self._get_val(self._current_message.pressure.used)

  @property
  def magnetometer_enabled(self):
    return self._get_val(self._current_message.magnetometer.enabled)

  @property
  def magnetometer_used(self):
    return self._get_val(self._current_message.magnetometer.used)

  @property
  def speed_enabled(self):
    return self._get_val(self._current_message.speed.enabled)

  @property
  def speed_used(self):
    return self._get_val(self._current_message.speed.used)

  @property
  def gnss1_enabled_string(self):
    return self._get_small_boolean_icon_string(self.gnss1_enabled)

  @property
  def gnss1_used_string(self):
    return self._get_small_boolean_icon_string(self.gnss1_used)

  @property
  def gnss2_enabled_string(self):
    return self._get_small_boolean_icon_string(self.gnss2_enabled)

  @property
  def gnss2_used_string(self):
    return self._get_small_boolean_icon_string(self.gnss2_used)

  @property
  def dual_antenna_enabled_string(self):
    return self._get_small_boolean_icon_string(self.dual_antenna_enabled)

  @property
  def dual_antenna_used_string(self):
    return self._get_small_boolean_icon_string(self.dual_antenna_used)

  @property
  def heading_enabled_string(self):
    return self._get_small_boolean_icon_string(self.heading_enabled)

  @property
  def heading_used_string(self):
    return self._get_small_boolean_icon_string(self.heading_used)

  @property
  def pressure_enabled_string(self):
    return self._get_small_boolean_icon_string(self.pressure_enabled)

  @property
  def pressure_used_string(self):
    return self._get_small_boolean_icon_string(self.pressure_used)

  @property
  def magnetometer_enabled_string(self):
    return self._get_small_boolean_icon_string(self.magnetometer_enabled)

  @property
  def magnetometer_used_string(self):
    return self._get_small_boolean_icon_string(self.magnetometer_used)

  @property
  def speed_enabled_string(self):
    return self._get_small_boolean_icon_string(self.speed_enabled)

  @property
  def speed_used_string(self):
    return self._get_small_boolean_icon_string(self.speed_used)


class GQ7LedMonitor:
  def __init__(self, filter_status_monitor, gnss_1_aiding_status_monitor, gnss_2_aiding_status_monitor):
    # This monitor is a little different in that it checks other monitors, so just save the other monitors
    self._filter_status_monitor = filter_status_monitor
    self._gnss_1_aiding_status_monitor = gnss_1_aiding_status_monitor
    self._gnss_2_aiding_status_monitor = gnss_2_aiding_status_monitor

  @property
  def gq7_led_icon(self):
    filter_state = self._filter_status_monitor.filter_state
    if filter_state is not _DEFAULT_VAL:
      if filter_state == 1:
        return _ICON_YELLOW_CHECKED_MEDIUM
      elif filter_state == 2 or filter_state == 3:
        return _ICON_YELLOW_UNCHECKED_MEDIUM
      elif filter_state == 4:
        gnss_1_differential = self._gnss_1_aiding_status_monitor.differential_corrections
        gnss_2_differential = self._gnss_2_aiding_status_monitor.differential_corrections
        if (gnss_1_differential is not _DEFAULT_VAL and gnss_1_differential) or (gnss_2_differential is not _DEFAULT_VAL and gnss_2_differential):
          return _ICON_BLUE_CHECKED_MEDIUM
        else:
          return _ICON_GREEN_CHECKED_MEDIUM
    else:
      return _ICON_GREY_UNCHECKED_MEDIUM