import qt_gui.plugin
import python_qt_binding
from .utils.constants import _MICROSTRAIN_ROS_VERISON
from .utils.widgets import MicrostrainWidget
from .filter_data import FilterDataWidget
from .gq7_led import GQ7LEDWidget
from .sensor_data import SensorDataWidget
from .gnss_data import GNSSDataWidget
from .rtk_status import RTKStatusWidget

class DeviceSelectorWidget(MicrostrainWidget):
  def __init__(self, node, node_name_changed_callback):
    # Initialize the parent class
    super(DeviceSelectorWidget, self).__init__(node, 'DeviceSelector')

    # Set the node name in the line edit
    self.node_name_line_edit.setText(self._node_name)

    # Connect the submit button to the passed callback
    self._node_name_changed_callback = node_name_changed_callback
    self.node_name_submit_button.clicked.connect(self._callback_wrapper)

  def run(self):
    # Update the device data
    self._update_device_data()

  def _update_device_data(self):
    self.node_connected_label.setText(self._device_report_monitor.connected_string)
    self.model_name_label.setText(self._device_report_monitor.model_name_string)
    self.model_number_label.setText(self._device_report_monitor.model_number_string)
    self.serial_number_label.setText(self._device_report_monitor.serial_number_string)
    self.options_label.setText(self._device_report_monitor.options_string)
    self.firmware_version_label.setText(self._device_report_monitor.firmware_version_string)
  
  def _callback_wrapper(self):
    self._node_name_changed_callback(self.node_name_line_edit.text())


class Quickview(qt_gui.plugin.Plugin):

  def __init__(self, context):
    # Initialize the parent class
    super(Quickview, self).__init__(context)

    # Save a copy of the node if we need to
    if _MICROSTRAIN_ROS_VERISON == 2:
      node = context.node
    else:
      node = None

    # Set some metadata
    self.setObjectName('Quickview')

    # Initialize our widgets
    device_selector_widget = DeviceSelectorWidget(node, self.change_node_name)
    filter_data_widget = FilterDataWidget(node)
    gq7_led_widget = GQ7LEDWidget(node)
    sensor_data_widget = SensorDataWidget(node)
    gnss_data_widget = GNSSDataWidget(node)
    rtk_status_widget = RTKStatusWidget(node)

    # Organize the widgets in a grid layout, otherwise the UI will not be formatted properly
    layout = python_qt_binding.QtWidgets.QGridLayout()
    left_column_layout = python_qt_binding.QtWidgets.QGridLayout()
    right_column_layout = python_qt_binding.QtWidgets.QGridLayout()
    layout.addLayout(left_column_layout,  0, 0)
    layout.addLayout(right_column_layout, 0, 1)

    left_column_layout.addWidget(device_selector_widget,  0, 0)  # Left side
    left_column_layout.addWidget(filter_data_widget,      1, 0)
    right_column_layout.addWidget(gq7_led_widget,         0, 0)  # Right side
    right_column_layout.addWidget(sensor_data_widget,     1, 0)
    right_column_layout.addWidget(gnss_data_widget,       2, 0)
    right_column_layout.addWidget(rtk_status_widget,      3, 0)

    # Save the widgets so we can update them, and shut them down
    self._widgets = [
      device_selector_widget,
      filter_data_widget,
      gq7_led_widget,
      sensor_data_widget,
      gnss_data_widget,
      rtk_status_widget
    ]

    # The ROS wrapper only allows us to add widgets, so add a wrapper widget to the layout and pass that back to ROS
    wrapper_widget = python_qt_binding.QtWidgets.QWidget()
    wrapper_widget.setLayout(layout)
    wrapper_widget.setObjectName('Quickview')
    wrapper_widget.setWindowTitle('Quickview')

    context.add_widget(wrapper_widget)
    
  def shutdown_plugin(self):
    # Stop the update loops
    for widget in self._widgets:
      widget.shutdown_timer()

    # call the parent function
    return super(Quickview, self).shutdown_plugin()

  def change_node_name(self, node_name):
    # Reconfigure all the widgets
    for widget in self._widgets:
      widget.reconfigure(node_name)
