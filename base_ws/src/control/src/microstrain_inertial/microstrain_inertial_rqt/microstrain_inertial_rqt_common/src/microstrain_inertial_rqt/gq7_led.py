from .utils.widgets import MicrostrainWidget, MicrostrainPlugin
from .utils.subscribers import FilterStatusMonitor, GNSSAidingStatusMonitor, GQ7LedMonitor

_WIDGET_NAME = 'GQ7LED'

class GQ7LEDWidget(MicrostrainWidget):

  def __init__(self, node):
    # Initialize the parent class
    super(GQ7LEDWidget, self).__init__(node, _WIDGET_NAME)

  def _configure(self):
    # Set up the subscriber status monitors
    self._gnss_1_aiding_status_monitor = GNSSAidingStatusMonitor(self._node, self._node_name, "gnss1/aiding_status")
    self._gnss_2_aiding_status_monitor = GNSSAidingStatusMonitor(self._node, self._node_name, "gnss2/aiding_status")
    self._filter_status_monitor = FilterStatusMonitor(self._node, self._node_name, "nav/status", self._device_report_monitor)

    # Set up a special monitor for the GQ7 LED
    self._gq7_led_monitor = GQ7LedMonitor(self._filter_status_monitor, self._gnss_1_aiding_status_monitor, self._gnss_2_aiding_status_monitor)

    # Hide the warning label
    self.gq7_led_not_available_label.hide()

  def run(self):
    # If the device is connected and not a GQ7, display a warning
    if self._device_report_monitor.connected and not self._device_report_monitor.is_gq7:
      self.gq7_led_widget.hide()
      self.gq7_led_not_available_label.setText('GQ7 LED not available for device %s, only available for GQ7 devices' % self._device_report_monitor.model_name_string)
      self.gq7_led_not_available_label.show()
      return
    else:
      self.gq7_led_not_available_label.hide()
      self.gq7_led_widget.show()

    # Update device specific data
    self._update_gq7_led()
    
  def _update_gq7_led(self):
    self.gq7_led_status_label.setText(self._filter_status_monitor.filter_state_led_string)
    self.gq7_led_icon_label.setText(self._gq7_led_monitor.gq7_led_icon)


class GQ7LEDPlugin(MicrostrainPlugin):

  def __init__(self, context):
    # Initialize the parent class
    super(GQ7LEDPlugin, self).__init__(context, _WIDGET_NAME, GQ7LEDWidget)
