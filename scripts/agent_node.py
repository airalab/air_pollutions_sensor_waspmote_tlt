#!/usr/bin/env python
# -*- coding: utf-8 -*-

from robonomics_lighthouse.msg import Bid
from std_msgs.msg import Empty
import rospy


class Agent:
    def __init__(self):
        rospy.init_node('agent')
        self.bids = rospy.Publisher('/infochan/signing/bid', Bid, queue_size=10)

        rospy.Subscriber('~model/process_trigger', Empty, self.on_process_trigger)

        self.sensor_pub = rospy.ServiceProxy('/sensor/pub_latest')

        rospy.wait_for_service('/liability/finish')
        self.liability_finish = rospy.ServiceProxy('/liability/finish', Empty)
        rospy.loginfo('Sensor agent node started.')

    def spin(self):
        self.make_bid()
        rospy.spin()

    def on_process_trigger(self, message):
        self.sensor_pub()
        self.liability_finish()
        self.make_bid()

    def make_bid(self):
        bid = Bid()
        bid.model = 'QmV1K3Y4sdLRz7rx2VTKbvyF35RRqat7kfmzYPwpGGr5u3'
        bid.token = '0x0Ef7fCB816fd725819e071eFB48F7EACb85c1A6A'
        bid.cost  = '1'
        bid.count = '1'
        bid.deadline = '7999999'
        rospy.loginfo('Publishing bid', bid)
        bids.publish(bid)


if __name__ == '__main__':
    Agent().spin()
