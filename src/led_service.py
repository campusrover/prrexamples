#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO

LED_GPIO = 4 #  Yellow
# LED_GPIO = 3   Green
# LED_GPIO = 2 // Blue
# LED_GPIO = 17 // White
# LED_GPIO = 27 // Red


def set_led_state_callback(req):
    GPIO.output(LED_GPIO, req.data)
    rospy.loginfo("LED Command Received: %s", req.data)

    return { 'success': True,
            'message': 'Successfully changed LED state' }

if __name__ == '__main__':
    rospy.init_node('led_actuator')

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_GPIO, GPIO.OUT)

    rospy.Service('set_led_state', SetBool, set_led_state_callback)
    rospy.loginfo("Service server started. Ready to get requests.")

    rospy.spin()

    GPIO.cleanup()