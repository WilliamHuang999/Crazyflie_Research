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
from cflib.utils import uri_helper
from cflib.crazyflie.swarm import Swarm, CachedCfFactory

def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)


def run_shared_sequence(scf):
    activate_mellinger_controller(scf, False)

    box_size = 1
    flight_time = 2

    commander = scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)

    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()

DEFAULT_HEIGHT = 2
# commands the drone to take off and then land after 3 seconds
def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

def box_sequence(scf):
    with MotionCommander(scf, default_height = DEFAULT_HEIGHT) as mc:
        mc.forward(1)
        time.sleep(0.5)
        mc.left(1)
        time.sleep(0.5)
        mc.back(0.5)
        time.sleep(0.5)
        mc.right(1)
        time.sleep(0.5)
        mc.stop()


uris = {
    'radio://0/1/2M/',
    'radio://0/6/2M/',
    # Add more URIs if you want more copters in the swarm
}

if __name__ == '__main__':
    cflib.crtp.init_drivers()

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
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(uris, factory=factory) as swarm:
            print("Swarm initiated")
            # swarm.parallel_safe(activate_high_level_commander)
            # print("Commanders Activated")
            # swarm.reset_estimators()
            # print("Sequence Begun")
            # swarm.parallel_safe(run_shared_sequence)
            # print("Sequence Ended")
            swarm.parallel_safe(box_sequence)
