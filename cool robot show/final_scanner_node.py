#!/usr/bin/env python
import rospy
from final.msg import Scan
from rplidar import RPLidar
import numpy as np

# === RPLidar sensor === #
PORT_NAME = '/dev/ttyUSB0'	# lidar port

if __name__ == '__main__':
  try:
    # === Lidar check === #
    lidar = RPLidar(PORT_NAME)
    info = lidar.get_info()	# get the device state
    print('info', info)
    health = lidar.get_health()
    print('health', health)

    pub = rospy.Publisher('/measurement', Scan, queue_size=10)
    rospy.init_node('final_lidar_rpi_node', anonymous=True)
    rate = rospy.Rate(350)
    result = Scan()

    while not rospy.is_shutdown():

      # === lidar === #
      print('Start Scanning...')
      scan = lidar.iter_measurments()
      for measurement in scan:
        print("Angle: %d\tDistance: %d" % (measurement[2], measurement[3]))
        result.Angle = measurement[2]
        result.Distance = measurement[3]
        pub.publish(result)
      rate.sleep()

  except rospy.ROSInterruptException or KeyboardInterrupt:
    pass
  finally:
    print("< ===== program finished ===== >")
    lidar.stop()	# stop scanning proccess
    lidar.stop_motor()	# stop sensor motor
    lidar.disconnect()

