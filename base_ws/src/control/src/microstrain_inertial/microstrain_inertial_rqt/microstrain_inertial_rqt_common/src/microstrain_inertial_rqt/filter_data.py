from .utils.widgets import MicrostrainWidget, MicrostrainPlugin
from .utils.subscribers import GNSSAidingStatusMonitor, GNSSDualAntennaStatusMonitor, FilterStatusMonitor, OdomMonitor, FilterAidingMeasurementSummaryMonitor

_WIDGET_NAME = 'FilterData'

class FilterDataWidget(MicrostrainWidget):

  def __init__(self, node):
    # Initialize the parent class
    super(FilterDataWidget, self).__init__(node, _WIDGET_NAME)

  def _configure(self):
    # Set up the subscriber status monitors
    self._absolute_odom_monitor = OdomMonitor(self._node, self._node_name, "nav/odom", llh=True)
    self._relative_odom_monitor = OdomMonitor(self._node, self._node_name, "nav/relative_pos/odom", llh=False)
    self._filter_aiding_status_summary_monitor = FilterAidingMeasurementSummaryMonitor(self._node, self._node_name, "nav/aiding_summary")
    self._gnss_1_aiding_status_monitor = GNSSAidingStatusMonitor(self._node, self._node_name, "gnss1/aiding_status")
    self._gnss_2_aiding_status_monitor = GNSSAidingStatusMonitor(self._node, self._node_name, "gnss2/aiding_status")
    self._gnss_dual_antenna_status_monitor = GNSSDualAntennaStatusMonitor(self._node, self._node_name, "nav/dual_antenna_status")
    self._filter_status_monitor = FilterStatusMonitor(self._node, self._node_name, "nav/status", self._device_report_monitor)

  def run(self):
    # Update data common to all sensors
    self._update_filter_status_data()
    self._update_odom()

    # GQ7 only widgets
    gq7_widgets = [
      self.filter_relative_position_widget,
      self.aiding_measurements_widget,
    ]
    # GNSS Only widgets (GQ7, GX5-45)    
    gnss_widgets = [
      self.filter_position_widget,
      self.filter_velocity_widget,
    ]

    # Hide or show the widgets
    self.hide_show_widgets(gq7_widgets, self._device_report_monitor.is_gq7)
    self.hide_show_widgets(gnss_widgets, self._device_report_monitor.has_gnss)

    # Update device specific data
    if self._device_report_monitor.is_gq7:
      self._update_aiding_measurements_data()
    
  def _update_filter_status_data(self):
    self.filter_state_label.setText(self._filter_status_monitor.filter_state_string)
    self.filter_status_flags_label.setText(self._filter_status_monitor.status_flags_string)

  def _update_odom(self):
    # Orientation
    self.filter_roll_label.setText(self._absolute_odom_monitor.orientation_x_string)
    self.filter_pitch_label.setText(self._absolute_odom_monitor.orientation_y_string)
    self.filter_yaw_label.setText(self._absolute_odom_monitor.orientation_z_string)

    # Orientation Uncertainty
    self.filter_roll_uncertainty_label.setText(self._absolute_odom_monitor.orientation_uncertainty_x_string)
    self.filter_pitch_uncertainty_label.setText(self._absolute_odom_monitor.orientation_uncertainty_y_string)
    self.filter_yaw_uncertainty_label.setText(self._absolute_odom_monitor.orientation_uncertainty_z_string)

    # Only available with GNSS
    if self._device_report_monitor.has_gnss:
      # Absolute Position
      self.filter_position_lat_label.setText(self._absolute_odom_monitor.position_x_string)
      self.filter_position_lon_label.setText(self._absolute_odom_monitor.position_y_string)
      self.filter_position_alt_label.setText(self._absolute_odom_monitor.position_z_string)

      # Absolute Position Uncertainty
      self.filter_position_lat_uncertainty_label.setText(self._absolute_odom_monitor.position_uncertainty_x_string)
      self.filter_position_lon_uncertainty_label.setText(self._absolute_odom_monitor.position_uncertainty_y_string)
      self.filter_position_alt_uncertainty_label.setText(self._absolute_odom_monitor.position_uncertainty_z_string)

      # Velocity
      self.filter_velocity_x_label.setText(self._absolute_odom_monitor.velocity_x_string)
      self.filter_velocity_y_label.setText(self._absolute_odom_monitor.velocity_y_string)
      self.filter_velocity_z_label.setText(self._absolute_odom_monitor.velocity_z_string)

      # Velocity Uncertainty
      self.filter_velocity_uncertainty_x_label.setText(self._absolute_odom_monitor.velocity_uncertainty_x_string)
      self.filter_velocity_uncertainty_y_label.setText(self._absolute_odom_monitor.velocity_uncertainty_y_string)
      self.filter_velocity_uncertainty_z_label.setText(self._absolute_odom_monitor.velocity_uncertainty_z_string)

    # Relative Position (only available in the GQ7)
    if self._device_report_monitor.is_gq7:
      self.filter_relative_position_lat_label.setText(self._relative_odom_monitor.position_x_string)
      self.filter_relative_position_lon_label.setText(self._relative_odom_monitor.position_y_string)
      self.filter_relative_position_alt_label.setText(self._relative_odom_monitor.position_z_string)

      self.filter_relative_position_lat_uncertainty_label.setText(self._relative_odom_monitor.position_uncertainty_x_string)
      self.filter_relative_position_lon_uncertainty_label.setText(self._relative_odom_monitor.position_uncertainty_y_string)
      self.filter_relative_position_alt_uncertainty_label.setText(self._relative_odom_monitor.position_uncertainty_z_string)

  def _update_aiding_measurements_data(self):
    # Aiding Measurements
    self.filter_aiding_measurements_gnss_1_enabled.setText(self._filter_aiding_status_summary_monitor.gnss1_enabled_string)
    self.filter_aiding_measurements_gnss_1_used.setText(self._filter_aiding_status_summary_monitor.gnss1_used_string)
    self.filter_aiding_measurements_gnss_2_enabled.setText(self._filter_aiding_status_summary_monitor.gnss2_enabled_string)
    self.filter_aiding_measurements_gnss_2_used.setText(self._filter_aiding_status_summary_monitor.gnss2_used_string)
    self.filter_aiding_measurements_dual_antenna_enabled.setText(self._filter_aiding_status_summary_monitor.dual_antenna_enabled_string)
    self.filter_aiding_measurements_dual_antenna_used.setText(self._filter_aiding_status_summary_monitor.dual_antenna_used_string)
    self.filter_aiding_measurements_heading_enabled.setText(self._filter_aiding_status_summary_monitor.heading_enabled_string)
    self.filter_aiding_measurements_heading_used.setText(self._filter_aiding_status_summary_monitor.heading_used_string)
    self.filter_aiding_measurements_pressure_enabled.setText(self._filter_aiding_status_summary_monitor.pressure_enabled_string)
    self.filter_aiding_measurements_pressure_used.setText(self._filter_aiding_status_summary_monitor.pressure_used_string)
    self.filter_aiding_measurements_mag_enabled.setText(self._filter_aiding_status_summary_monitor.magnetometer_enabled_string)
    self.filter_aiding_measurements_mag_used.setText(self._filter_aiding_status_summary_monitor.magnetometer_used_string)
    self.filter_aiding_measurements_speed_enabled.setText(self._filter_aiding_status_summary_monitor.speed_enabled_string)
    self.filter_aiding_measurements_speed_used.setText(self._filter_aiding_status_summary_monitor.speed_used_string)

    # GNSS Aiding Status
    self.filter_position_aiding_gnss_1_tight_coupling_label.setText(self._gnss_1_aiding_status_monitor.tight_coupling_string)
    self.filter_position_aiding_gnss_1_differential_label.setText(self._gnss_1_aiding_status_monitor.differential_corrections_string)
    self.filter_position_aiding_gnss_1_integer_fix_label.setText(self._gnss_1_aiding_status_monitor.integer_fix_string)

    self.filter_position_aiding_gnss_2_tight_coupling_label.setText(self._gnss_2_aiding_status_monitor.tight_coupling_string)
    self.filter_position_aiding_gnss_2_differential_label.setText(self._gnss_2_aiding_status_monitor.differential_corrections_string)
    self.filter_position_aiding_gnss_2_integer_fix_label.setText(self._gnss_2_aiding_status_monitor.integer_fix_string)

    # Dual Antenna Status
    self.filter_dual_antenna_fix_type_label.setText(self._gnss_dual_antenna_status_monitor.fix_type_string)
    self.filter_dual_antenna_heading_label.setText(self._gnss_dual_antenna_status_monitor.heading_string)
    self.filter_dual_antenna_uncertainty_label.setText(self._gnss_dual_antenna_status_monitor.heading_uncertainty_string)
    self.filter_dual_antenna_rec_1_data_valid_label.setText(self._gnss_dual_antenna_status_monitor.rec_1_data_valid_string)
    self.filter_dual_antenna_rec_2_data_valid_label.setText(self._gnss_dual_antenna_status_monitor.rec_2_data_valid_string)
    self.filter_dual_antenna_offsets_valid_label.setText(self._gnss_dual_antenna_status_monitor.antenna_offsets_valid_string)


class FilterDataPlugin(MicrostrainPlugin):

  def __init__(self, context):
    # Initialize the parent class
    super(FilterDataPlugin, self).__init__(context, _WIDGET_NAME, FilterDataWidget)
