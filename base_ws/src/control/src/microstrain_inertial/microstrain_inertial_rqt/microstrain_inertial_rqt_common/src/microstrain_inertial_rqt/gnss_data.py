from .utils.widgets import MicrostrainWidget, MicrostrainPlugin
from .utils.subscribers import GNSSFixInfoMonitor, NavSatFixMonitor

_WIDGET_NAME = 'GNSSData'

class GNSSDataWidget(MicrostrainWidget):

  def __init__(self, node):
    # Initialize the parent class
    super(GNSSDataWidget, self).__init__(node, _WIDGET_NAME)

  def _configure(self):
    # Set up the subscriber status monitors
    self._gnss_1_fix_info_monitor = GNSSFixInfoMonitor(self._node, self._node_name, "gnss1/fix_info")
    self._gnss_2_fix_info_monitor = GNSSFixInfoMonitor(self._node, self._node_name, "gnss2/fix_info")
    self._gnss_1_nav_sat_fix_monitor = NavSatFixMonitor(self._node, self._node_name, "gnss1/fix")
    self._gnss_2_nav_sat_fix_monitor = NavSatFixMonitor(self._node, self._node_name, "gnss2/fix")

    # Hide the warning label
    self.gnss_not_available_label.hide()

  def run(self):
    # Dual Antenna Only widgets (GQ7)
    dual_antenna_widgets = [
      self.gnss_2_widget
    ]

    # Hide or show the widgets
    self.hide_show_widgets(dual_antenna_widgets, self._device_report_monitor.has_dual_antenna)

    # If the device is connected and there is no GNSS data, display a warning, otherwise display GNSS data
    if self._device_report_monitor.connected and not self._device_report_monitor.has_gnss:
      self.gnss_widget.hide()
      self.gnss_not_available_label.setText('GNSS data not available for device %s' % self._device_report_monitor.model_name_string)
      self.gnss_not_available_label.show()
      return
    else:
      self.gnss_not_available_label.hide()
      self.gnss_widget.show()

    # Update device specific data
    self._update_gnss_1_data()
    self._update_gnss_2_data()
    
  def _update_gnss_1_data(self):
    # Fix Info
    self.gnss_1_fix_type_label.setText(self._gnss_1_fix_info_monitor.fix_type_string)
    self.gnss_1_sv_count_label.setText(self._gnss_1_fix_info_monitor.num_sv_string)
    self.gnss_1_position_uncertainty_label.setText(self._gnss_1_nav_sat_fix_monitor.position_uncertainty_string)

  def _update_gnss_2_data(self):
    # Fix Info
    self.gnss_2_fix_type_label.setText(self._gnss_2_fix_info_monitor.fix_type_string)
    self.gnss_2_sv_count_label.setText(self._gnss_2_fix_info_monitor.num_sv_string)
    self.gnss_2_position_uncertainty_label.setText(self._gnss_2_nav_sat_fix_monitor.position_uncertainty_string)


class GNSSDataPlugin(MicrostrainPlugin):

  def __init__(self, context):
    # Initialize the parent class
    super(GNSSDataPlugin, self).__init__(context, _WIDGET_NAME, GNSSDataWidget)
