from .utils.widgets import MicrostrainWidget, MicrostrainPlugin
from .utils.subscribers import ImuMonitor, MagMonitor

_WIDGET_NAME = 'SensorData'

class SensorDataWidget(MicrostrainWidget):

  def __init__(self, node):
    # Initialize the parent class
    super(SensorDataWidget, self).__init__(node, _WIDGET_NAME)

  def _configure(self):
    # Set up the subscriber status monitors
    self._imu_monitor = ImuMonitor(self._node, self._node_name, "imu/data")
    self._mag_monitor = MagMonitor(self._node, self._node_name, "mag")

  def run(self):
    # Update data common to all sensors
    self._update_imu()

    # Magnetometer Only widgets (GQ7, GX5-45, GX5-25)
    magnetometer_widgets = [
      self.sensor_magnetometer_widget,
    ]

    # Hide or show the widgets
    self.hide_show_widgets(magnetometer_widgets, self._device_report_monitor.has_magnetometer)

    # Update device specific data
    if self._device_report_monitor.has_magnetometer:
      self._update_mag()
    
  def _update_imu(self):
    # Accel
    self.sensor_accel_x_label.setText(self._imu_monitor.accel_x_string)
    self.sensor_accel_y_label.setText(self._imu_monitor.accel_y_string)
    self.sensor_accel_z_label.setText(self._imu_monitor.accel_z_string)

    # Gyro
    self.sensor_gyro_x_label.setText(self._imu_monitor.vel_x_string)
    self.sensor_gyro_y_label.setText(self._imu_monitor.vel_y_string)
    self.sensor_gyro_z_label.setText(self._imu_monitor.vel_z_string)

  def _update_mag(self):
    self.sensor_magnetometer_x_label.setText(self._mag_monitor.x_string)
    self.sensor_magnetometer_y_label.setText(self._mag_monitor.y_string)
    self.sensor_magnetometer_z_label.setText(self._mag_monitor.z_string)


class SensorDataPlugin(MicrostrainPlugin):

  def __init__(self, context):
    # Initialize the parent class
    super(SensorDataPlugin, self).__init__(context, _WIDGET_NAME, SensorDataWidget)
