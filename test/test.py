#

import logging
from shutil import move
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper


# uri = "usb://0"
uri = "radio://0/80/2M/"

deck_attached_event = Event()

DEFAULT_HEIGHT = 1
BOX_LIMIT = 0.5

# checks whether flowdeck is installed
def param_deck_flow(_, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print("Deck is attached!")
    else:
        print("Deck is NOT attached!")


# commands the drone to take off and then land after 3 seconds
def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()


def land_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.land()


# commands the drone to move forward, turn 180 deg, then move forward more (in the other direction)
def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)


def ascend_and_hover(cf):
    # Ascend:
    for y in range(5):
        cf.commander.send_hover_setpoint(0, 0, 0, y / 10)
        time.sleep(0.1)
    # Hover at 0.5 meters:
    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 100, 0.5)
        time.sleep(0.1)
    return


def hover_and_descend(cf):
    print("Descending:")
    # Hover at 0.5 meters:
    for _ in range(30):
        cf.commander.send_hover_setpoint(0, 0, 100, 0.5)
        time.sleep(0.1)
    # Descend:
    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
        time.sleep(0.1)
    # Stop all motion:
    for i in range(10):
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)
    return


def box_sequence(scf, velo=0.5):
    mc = MotionCommander(scf, default_height=DEFAULT_HEIGHT)
    mc.take_off(height=DEFAULT_HEIGHT, velocity=1)
    time.sleep(0.5)
    mc.forward(1, velo)
    time.sleep(0.5)
    mc.left(1, velo)
    time.sleep(0.5)
    mc.back(1, velo)
    time.sleep(0.5)
    mc.right(1, velo)
    time.sleep(0.5)
    mc.stop()


def land(scf):
    mc = MotionCommander(scf, default_height=DEFAULT_HEIGHT)
    mc.land()


if __name__ == "__main__":

    # Initialize all the CrazyFlie drivers:
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Scan for Crazyflies in range of the antenna:
    print("Scanning interfaces for Crazyflies...")
    available = cflib.crtp.scan_interfaces()

    # List local CrazyFlie devices:
    print("Crazyflies found:")
    for i in available:
        print(i[0])

    # Check that CrazyFlie devices are available:
    if len(available) == 0:
        print("No Crazyflies found, cannot run example")
    else:

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:

            cf = scf.cf

            cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
            time.sleep(1)

            t = time.time()
            elapsed = time.time() - t

            # ascend_and_hover(cf)

            # hover_and_descend(cf)
            # while(elapsed < 10):
            #     elapsed = time.time() - t
            #     cf.commander.send_hover_setpoint(0, 0, 0, 0.5)
            #     time.sleep(0.1)
            # hover_and_descend(cf)

            # box_sequence(scf, velo = 0.5)
            # take_off_simple(cf)
