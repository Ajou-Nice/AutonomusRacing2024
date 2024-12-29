from threading import Thread

from microstrain_inertial_msgs.srv import DeviceReport

from .common import ServiceMonitor
from .constants import _MICROSTRAIN_ROS_VERISON
from .constants import _DEFAULT_VAL

class DeviceReportMonitor(ServiceMonitor):

  def __init__(self, node, node_name):
    super(DeviceReportMonitor, self).__init__(node, node_name, "device_report", DeviceReport, message_timeout=25, poll_interval=5)

    # Keep track of some booleans so that we can keep track between disconnects
    self._is_gq7 = False
    self._is_gx5 = False
    self._has_magnetometer = False
    self._has_gnss = False
    self._has_dual_antenna = False

  @property
  def connected(self):
    return self._current_message is not _DEFAULT_VAL and not self._message_timed_out

  @property
  def model_name(self):
    return self._get_val(self._current_message.model_name)

  @property
  def model_number(self):
    return self._get_val(self._current_message.model_number)

  @property
  def serial_number(self):
    return self._get_val(self._current_message.serial_number)

  @property
  def options(self):
    return self._get_val(self._current_message.options)

  @property
  def firmware_version(self):
    return self._get_val(self._current_message.firmware_version)

  @property
  def is_gq7(self):
    # If we are not connected, no reason to check any further
    if self.connected:
      # Check the model name to determine if this is a GQ7
      model_name = self.model_name
      if model_name is not _DEFAULT_VAL:
        self._is_gq7 = "GQ7" in model_name
    
    # Return the stored value
    return self._is_gq7
  
  @property
  def is_gx5(self):
    # If we are not connected, no reason to check any further
    if self.connected:
      # Check the model name to determine if this is a GX5
      model_name = self.model_name
      if model_name is not _DEFAULT_VAL:
        self._is_gx5 = "GX5" in model_name

    # Return the stored value
    return self._is_gx5

  @property
  def has_magnetometer(self):
    # If we are not connected, no reason to check any further
    if self.connected:
      # Check the model name to determine if this device has GNSS
      model_name = self.model_name
      if model_name is not _DEFAULT_VAL:
        self._has_magnetometer = self.is_gq7 or "GX5-45" in model_name or "GX5-25" in model_name
    
    # Return the stored value
    return self._has_magnetometer

  @property
  def has_gnss(self):
    # If we are not connected, no reason to check any further
    if self.connected:
      # Check the model name to determine if this device has GNSS
      model_name = self.model_name
      if model_name is not _DEFAULT_VAL:
        self._has_gnss = self.is_gq7 or "GX5-45" in model_name
    
    # Return the stored value
    return self._has_gnss

  @property
  def has_dual_antenna(self):
    # Right now, the GQ7 is the only device that supports dual antenna
    return self.is_gq7

  @property
  def model_name_string(self):
    return self._get_string(self.model_name)

  @property
  def model_number_string(self):
    return self._get_string(self.model_number)

  @property
  def serial_number_string(self):
    return self._get_string(self.serial_number)

  @property
  def options_string(self):
    return self._get_string(self.options)

  @property
  def firmware_version_string(self):
    return self._get_string(self.firmware_version)

  @property
  def connected_string(self):
    return self._get_small_boolean_icon_string(self.connected)
