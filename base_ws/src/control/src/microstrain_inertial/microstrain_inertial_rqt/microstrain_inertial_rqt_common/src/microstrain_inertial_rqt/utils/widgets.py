import os
import re
import qt_gui.plugin
import python_qt_binding

from .constants import _MICROSTRAIN_ROS_VERISON
from .constants import _PACKAGE_RESOURCE_DIR
from .constants import _NODE_NAME_ENV_KEY, _DEFAULT_NODE_NAME
from .services import DeviceReportMonitor

class MicrostrainWidget(python_qt_binding.QtWidgets.QWidget):

  def __init__(self, node, name):
    # Initialize the parent class
    super(MicrostrainWidget, self).__init__()

    # Load the UI spec
    python_qt_binding.loadUi(os.path.join(_PACKAGE_RESOURCE_DIR, '%s.ui' % name), self)

    # Set some other metadata
    self.setObjectName(name)
    self.setWindowTitle(name)

    # Save a copy of the node
    self._node = node

    # Since this is loaded as an RQT plugin, we can't get config from a launch file. Instead, read an environment variable to determine what the default node name should be
    self._node_name = self.sanitize_node_name(os.getenv(_NODE_NAME_ENV_KEY, _DEFAULT_NODE_NAME))

    # Set up the service status monitors
    self._device_report_monitor = DeviceReportMonitor(self._node, self._node_name)

    # Configure with the default node name
    self._configure()

    # Start the UI update loop
    self._run_timer = python_qt_binding.QtCore.QTimer()
    self._run_timer.setSingleShot(False)
    self._run_timer.timeout.connect(self.run)
    self._run_timer.start(250)

  def shutdown_timer(self):
    self._run_timer.stop()
    del self._run_timer

  def _configure(self):
    # Expected to be implemented in the implementing class
    pass

  def run(self):
    # Expected to be implemented in the implementing class
    pass

  def reconfigure(self, node_name):
    # Update the device monitor, and our node name
    self._node_name = node_name
    self._device_report_monitor = DeviceReportMonitor(self._node, self._node_name)

    # Call the configure function again
    self._configure()

  @staticmethod
  def sanitize_node_name(node_name):
    node_name = re.sub(r'\/+', '/', node_name)  # Remove duplicate slashes
    if not node_name.startswith('/'):  # Start with a slash
      node_name = '/' + node_name
    if node_name.endswith('/'):  # Do not end with a slash
      node_name = node_name[:-1]
    return node_name

  @staticmethod
  def hide_show_widgets(widgets, show):
    for widget in widgets:
      if show and widget.isHidden():
        widget.show()
      elif not show and not widget.isHidden():
        widget.hide()


class MicrostrainPlugin(qt_gui.plugin.Plugin):

  def __init__(self, context, name, widget_type):
    # Initialize the parent class
    super(MicrostrainPlugin, self).__init__(context)

    # Set some other metadata
    self.setObjectName(name)

    # Save a copy of the node
    if _MICROSTRAIN_ROS_VERISON == 2:
      node = context.node
    else:
      node = None

    # Initialize the widget
    self._widget = widget_type(node)

    # Show _widget.windowTitle on left-top of each plugin (when 
    # it's set in _widget). This is useful when you open multiple 
    # plugins at once. Also if you open multiple instances of your 
    # plugin at once, these lines add number to make it easy to 
    # tell from pane to pane.
    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

    # Add widget to the user interface
    context.add_widget(self._widget)

  def shutdown_plugin(self):
    # Stop the update loop
    self._widget.shutdown_timer()

    # Call the parent function
    return super(MicrostrainPlugin, self).shutdown_plugin()