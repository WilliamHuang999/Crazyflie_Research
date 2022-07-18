from cflib.crazyflie import Crazyflie
import time


def propTest(cf):
    cf.param.set_value("system.forceArm", 0)
    cf.param.set_value("motorPowerSet.enable", 1)

    cf.param.set_value("motorPowerSet.m1", 5000)
    time.sleep(0.5)
    cf.param.set_value("motorPowerSet.m1", 0)

    cf.param.set_value("motorPowerSet.m2", 5000)
    time.sleep(0.5)
    cf.param.set_value("motorPowerSet.m2", 0)

    cf.param.set_value("motorPowerSet.m3", 5000)
    time.sleep(0.5)
    cf.param.set_value("motorPowerSet.m3", 0)

    cf.param.set_value("motorPowerSet.m4", 5000)
    time.sleep(0.5)
    cf.param.set_value("motorPowerSet.m4", 0)

    time.sleep(0.5)

    cf.param.set_value("motorPowerSet.enable", 0)
