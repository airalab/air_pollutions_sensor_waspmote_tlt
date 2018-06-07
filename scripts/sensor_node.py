#!/usr/bin/env python
# -*- coding: utf-8 -*-

from air_pollutions_sensor_waspmote_tlt.msg import Measurement
from std_srvs.srv import Empty, EmptyResponse
import rospy
import requests
import json


class SensorAPI:
    def __init__(self, server_address):
        self.server_address = server_address
    
    def get_latest_measurement(self):
        url = 'http://' + self.server_address[0] + ':' + str(self.server_address[1])
        measurement = json.loads(requests.get(url + '/latest_measurement').content)['data']
        return measurement


class SensorNode:
    def __init__(self):
        rospy.init_node('sensor')
        sensor_address = rospy.get_param('~server_host'), int(rospy.get_param('~server_port'))
        self.sensor = SensorAPI(sensor_address)
        self.measure = rospy.Publisher('measurement', Measurement, queue_size=20)

        def on_pub_latest(request):
            measurement = self.sensor.get_latest_measurement()
            msg = Measurement()
            if measurement:
                msg.sensor_ts = measurement['SensorTimestamp']
                msg.db_ts     = measurement['DatabaseTimestamp']
                msg.temp      = measurement['Temperature']
                msg.hum       = measurement['Humidity']
                msg.pres      = measurement['Pressure']
                msg.co        = measurement['CO']
                msg.no        = measurement['NO']
                msg.so2       = measurement['SO2']
                msg.pm1       = measurement['PM1']
                msg.pm2_5     = measurement['PM2_5']
                msg.pm10      = measurement['PM10']
            self.measure.publish(msg)
            rospy.loginfo('Publishing latest measurement', msg)
            return EmptyResponse()
        rospy.Service('pub_latest', Empty, on_pub_latest)

    def spin(self):
        rospy.spin()
    

if __name__ == '__main__':
    SensorNode().spin()
