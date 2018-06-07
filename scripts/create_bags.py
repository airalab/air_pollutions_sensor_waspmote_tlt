# -*- coding: utf-8 -*-

from std_msgs.msg import Empty
from std_msgs.msg import Duration
import rosbag


def create_model():
    bag = rosbag.Bag('./model.bag', 'w')

    topic = '/agent/model/process_trigger'

    msg = Empty()
    bag.write(topic, msg)

    bag.close()


def create_objective():
    bag = rosbag.Bag('./objective.bag', 'w')

    topic = '/agent/objective/period'

    msg = Duration()
    msg.data.secs = 86400 # 24 hours

    bag.write(topic, msg)
    bag.close()


create_model()
create_objective()
