#!/usr/bin/env python3

import threading
import time
from pathlib import Path
from functools import partial

import rclpy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crazyflie_interfaces.msg import Status
import rowan

from nicegui import Client, app, events, ui, ui_run, Tailwind


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')

        # wait until the crazyflie_server is up and running
        self.emergencyService = self.create_client(Empty, 'all/emergency')
        self.emergencyService.wait_for_service()

        # find all crazyflies
        self.cfnames = []
        for srv_name, srv_types in self.get_service_names_and_types():
            if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
                # remove '/' and '/start_trajectory'
                cfname = srv_name[1:-17]
                if cfname != 'all':
                    self.cfnames.append(cfname)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_log = self.create_subscription(Log, 'rosout', self.on_rosout, rclpy.qos.QoSProfile(
                        depth=1000,
                        durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.logs = dict()
        self.supervisor_labels = dict()
        self.battery_labels = dict()
        self.radio_labels = dict()
        self.robotmodels = dict()

        self.normal_style = Tailwind().text_color('black').font_weight('normal')
        self.red_style = Tailwind().text_color('red-600').font_weight('bold')

        with Client.auto_index_client:

            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-full h-full'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(800, 400, on_click=self.on_vis_click) as scene:
                        for name in self.cfnames:
                            robot = scene.stl('/urdf/cf2_assembly.stl').scale(1.0).material('#ff0000').with_name(name)
                            self.robotmodels[name] = robot
                            # augment with some additional fields
                            robot.status_ok = False
                            robot.battery_ok = False
                            robot.status_watchdog = time.time()
                            robot.supervisor_text = ""
                            robot.battery_text = ""
                            robot.radio_text = ""
                    scene.camera.x = 0
                    scene.camera.y = -1
                    scene.camera.z = 2
                    scene.camera.look_at_x = 0
                    scene.camera.look_at_y = 0
                    scene.camera.look_at_z = 0

            with ui.row().classes('w-full h-lvh'):
                with ui.tabs().classes('w-full') as tabs:
                    self.tabs = []
                    for name in ["all"] + self.cfnames:
                        self.tabs.append(ui.tab(name))
                with ui.tab_panels(tabs, value=self.tabs[0], on_change=self.on_tab_change).classes('w-full') as self.tabpanels:
                    for name, tab in zip(["all"] + self.cfnames, self.tabs):
                        with ui.tab_panel(tab):
                            self.logs[name] = ui.log().classes('w-full h-96 no-wrap')
                            self.supervisor_labels[name] = ui.label("")
                            self.battery_labels[name] = ui.label("")
                            self.radio_labels[name] = ui.label("")

            for name in self.cfnames:
                self.create_subscription(Status, name + '/status', partial(self.on_status, name=name), 1)

            # Call on_timer function
            update_rate = 30 # Hz
            self.timer = self.create_timer(
                1.0/update_rate, 
                self.on_timer,
                clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME))

    def on_rosout(self, msg: Log) -> None:
        # filter by crazyflie and add to the correct log
        if msg.name == "crazyflie_server":
            if msg.msg.startswith("["):
                idx = msg.msg.find("]")
                name = msg.msg[1:idx]
                # if it was an "all" category, add only to CFs
                if name == 'all':
                    for logname ,log in self.logs.items():
                        if logname != "all":
                            log.push(msg.msg)
                elif name in self.logs:
                    self.logs[name].push(msg.msg[idx+2:])

        # add all possible messages to the 'all' tab
        self.logs['all'].push(msg.msg)

    def on_timer(self) -> None:
        for name, robotmodel in self.robotmodels.items():
            ros_time = rclpy.time.Time() # get the latest
            robot_status_ok = robotmodel.status_ok and robotmodel.battery_ok
            robot_status_text = ""
            if self.tf_buffer.can_transform("world", name, ros_time):
                t = self.tf_buffer.lookup_transform(
                                "world",
                                name,
                                ros_time)
                transform_time = rclpy.time.Time.from_msg(t.header.stamp)
                transform_age = self.get_clock().now() - transform_time
                # latest transform is older than a second indicates a problem
                if transform_age.nanoseconds * 1e-9 > 1:
                    robot_status_ok = False
                    robot_status_text += "old transform; "
                else:
                    pos = t.transform.translation
                    robotmodel.move(pos.x, pos.y, pos.z)
                    robotmodel.rotate(*rowan.to_euler([
                        t.transform.rotation.w,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z], "xyz"))
            else:
                # no available transform indicates a problem
                robot_status_ok = False
                robot_status_text += "unavailable transform; "

            # no status update for a while, indicate a problem
            if time.time() - robotmodel.status_watchdog > 2.0:
                robot_status_ok = False
                robot_status_text += "no recent status update; "

                self.supervisor_labels[name].set_text(robot_status_text)
                self.battery_labels[name].set_text("N.A.")
                self.radio_labels[name].set_text("N.A.")
            else:
                self.supervisor_labels[name].set_text(robot_status_text + robotmodel.supervisor_text)
                self.battery_labels[name].set_text(robotmodel.battery_text)
                self.radio_labels[name].set_text(robotmodel.radio_text)

            # any issues detected -> mark red, otherwise green
            if robot_status_ok:
                robotmodel.material('#00ff00')
            else:
                robotmodel.material('#ff0000')

            if robotmodel.battery_ok:
                self.normal_style.apply(self.battery_labels[name])
            else:
                self.red_style.apply(self.battery_labels[name])

    def on_vis_click(self, e: events.SceneClickEventArguments):
        hit = e.hits[0]
        name = hit.object_name or hit.object_id
        ui.notify(f'You clicked on the {name}')
        if name == 'ground':
            self.tabpanels.value = 'all'
        else:
            self.tabpanels.value = name

    def on_status(self, msg, name) -> None:
        status_ok = True
        is_flying = False
        supervisor_text = ""
        if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_BE_ARMED:
            supervisor_text += "can be armed; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_ARMED:
            supervisor_text += "is armed; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_AUTO_ARM:
            supervisor_text += "auto-arm; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_FLY:
            supervisor_text += "can fly; "
        else:
            status_ok = False
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_FLYING:
            supervisor_text += "is flying; "
            is_flying = True
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_TUMBLED:
            supervisor_text += "is tumbled; "
            status_ok = False
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_LOCKED:
            supervisor_text += "is locked; "
            status_ok = False
        self.robotmodels[name].supervisor_text = supervisor_text

        battery_text = f'{msg.battery_voltage:.2f} V'
        battery_ok = True
        # TODO (WH): We could read the voltage limits from the firmware (parameter) or crazyflies.yaml
        #            In the firmware, anything below 3.2 is warning, anything below 3.0 is critical
        if (is_flying and msg.battery_voltage < 3.2) or (not is_flying and msg.battery_voltage < 3.8):
            battery_ok = False
        if msg.pm_state == Status.PM_STATE_BATTERY:
            battery_text += " (on battery)"
        elif msg.pm_state == Status.PM_STATE_CHARGING:
            battery_text += " (charging)"
        elif msg.pm_state == Status.PM_STATE_CHARGED:
            battery_text += " (charged)"
        elif msg.pm_state == Status.PM_STATE_LOW_POWER:
            battery_text += " (low power)"
            battery_ok = False
        elif msg.pm_state == Status.PM_STATE_SHUTDOWN:
            battery_text += " (shutdown)"
            battery_ok = False
        self.robotmodels[name].battery_text = battery_text
        
        radio_text = f'{msg.rssi} dBm; Unicast: {msg.num_rx_unicast} / {msg.num_tx_unicast}; Broadcast: {msg.num_rx_broadcast} / {msg.num_tx_broadcast}; Latency: {msg.latency_unicast} ms'
        self.robotmodels[name].radio_text = radio_text

        # save status here
        self.robotmodels[name].status_ok = status_ok
        self.robotmodels[name].battery_ok = battery_ok

        # store the time when we last received any status
        self.robotmodels[name].status_watchdog = time.time()

    def on_tab_change(self, arg):
        for name, robotmodel in self.robotmodels.items():
            if name != arg.value:
                robotmodel.scale(1)
        if arg.value in self.robotmodels:
            self.robotmodels[arg.value].scale(2)


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.add_static_files("/urdf",
                     str((Path(__file__).parent.parent.parent / "share" / "crazyflie" / "urdf").resolve()),
                     follow_symlink=True)
app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
