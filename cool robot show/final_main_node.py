#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32
from final.msg import Scan
import RPi.GPIO as GPIO

# === ultrasound sensor === #
trigger_pin = 2	# ultrasound trigger pin
echo_pin = 3	# ultrasound echo pin
dis = 0		# initialize distance
# === RPLidar sensor === #
angle = 0
distance = 0
# === robot state === #
STATE = 0	# moving: 0, scanning: 1-300
TURN_FLAG_R = False
TURN_FLAG_L = False
TURN_COUNTER = 0
MAX_COUNTER = 34
#TIME_CHECK = 0	# scan every 50 steps

front = [0, 0, 0, 0]
right = [0, 0, 0, 0]
left = [0, 0, 0, 0]
iter_front = 0
iter_right = 0
iter_left = 0
front_sum = 0
right_sum = 0
left_sum = 0


def send_trigger_pulse():
  GPIO.output(trigger_pin, True)
  time.sleep(0.001)
  GPIO.output(trigger_pin, False)

def wait_for_echo(value, timeout):
  count = timeout
  while GPIO.input(echo_pin) != value and count > 0:
    count = count - 1

def get_distance():
  send_trigger_pulse()
  wait_for_echo(True, 5000)
  start = time.time()
  wait_for_echo(False, 5000)
  finish = time.time()
  pulse_len = finish - start
  distance = pulse_len * 17348 - 5.8844
  return distance

def callback(data):
  global angle
  angle = data.Angle
  global distance
  distance = data.Distance

if __name__ == '__main__':
  try:
    # === GPIO pins setting === #
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trigger_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)

    pub = rospy.Publisher('/mode', Int32, queue_size=10)
    rospy.init_node('final_rpi__node', anonymous=True)
    sub = rospy.Subscriber('/measurement', Scan, callback)
    rate = rospy.Rate(10)
    rate2 = rospy.Rate(350)

    while not rospy.is_shutdown():

      # === state 1: moving  === #
      if STATE == 0:
        dis = get_distance()
        if TURN_FLAG_L == False and TURN_FLAG_R == False:	# moving forward
            if dis < 6:	# testing if there is a wall in front of the robot
              STATE = 1	# enter state 2 scanning
              pub.publish(0)	# stop robot
              #if TIME_CHECK == 50:
              #  print("time check")
              print("< === Entering scanning state === >")
              #TIME_CHECK = 0	# reset TIMECHECK
              rate.sleep()
            else:
              pub.publish(1)
              print("Forward")
              #TIME_CHECK += 1
              rate.sleep()

        elif TURN_FLAG_L:	# turn left
          TURN_COUNTER += 1
          if TURN_COUNTER < 7:	# move backward
            pub.publish(2)
            rate.sleep()
          elif TURN_COUNTER < MAX_COUNTER:	# turn left
            pub.publish(3)
            rate.sleep()
          else:
            TURN_COUNTER = 0
            TURN_FLAG_L = False
            TURN_FLAG_R = False
            rate.sleep()

        else:	# turn right
          TURN_COUNTER += 1
          if TURN_COUNTER < 7:	# move backward
            pub.publish(2)
            rate.sleep()
          elif TURN_COUNTER < MAX_COUNTER:	# turn right
            pub.publish(4)
            rate.sleep()
          else:
            TURN_COUNTER = 0
            TURN_FLAG_L = False
            TURN_FLAG_R = False
            rate.sleep()

      # === state 2: scanning === #
      elif STATE == 1:
        pub.publish(0)
        STATE += 1
        rate.sleep()

      elif STATE < 600:
        print(" Scanning...")
        STATE += 1
        if angle < 20 or angle > 340:	# front distance
          front[iter_front] = distance
          if iter_front != 3:
            iter_front += 1
            rate2.sleep()
          else:
            iter_front = 0
            rate2.sleep()
        elif angle < 110 and angle > 70:	# right distance
          right[iter_right] = distance
          if iter_right != 3:
            iter_right += 1
            rate2.sleep()
          else:
            iter_right = 0
            rate2.sleep()
        elif angle < 290 or angle > 250:	# left distance
          left[iter_left] = distance
          if iter_left != 3:
            iter_left += 1
            rate2.sleep()
          else:
            iter_left = 0
            rate2.sleep()
        else:
          rate2.sleep()

      else:
        for i in range(3):
          front_sum += front[i]
          right_sum += right[i]
          left_sum += left[i]
          #print('front ', i ,': ', front[i])
          #print('right ', i ,': ', right[i])
          #print('left ', i ,': ', left[i])

        print('front_sum = ', front_sum)
        print('right_sum = ', right_sum)
        print('left_sum = ', left_sum)
        max_value = max(front_sum, right_sum, left_sum)
        if max_value == front_sum:
          print("Forward")
          TURN_FLAG_L = False
          TURN_FLAG_R = False
        elif max_value == right_sum:
          print("Turn right")
          TURN_FLAG_R = True
          TURN_FLAG_L = False
        else:
          print("Turn left")
          TURN_FLAG_R = False
          TURN_FLAG_L = True
        front_sum = 0
        right_sum = 0
        left_sum = 0
        STATE = 0
        rate.sleep()



  except rospy.ROSInterruptException:
    pass
  finally:
    print("< ===== program finished ===== >")
    GPIO.cleanup()

