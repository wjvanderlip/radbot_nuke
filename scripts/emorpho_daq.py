#!/usr/bin/env python

import rospy
import bpiem

import numpy as np

# import rospkg
# rospack = rospkg.RosPack()

from multiprocessing import Pool

from client import client

# init the node
rospy.init_node('RadAcqusitionClient', anonymous=False)

acq_type = 'listmode'  #'listmode'
lm_mode = 0 #0 to return ts and amp
lm_maxtime = 1 #max time between lm messages, set .1s
inttime = 1 #hist param
realtime = 1 #hst param
message_type = 1  #complex msg switch

# Create detector object and client object
detector = bpiem.EMDevice(multi=True)
sns = detector.sn #used to collect all connect det SNs must be eRC####
c = client(port_num = 8082)

def multi_process(SNs):
    '''
    parallel element of code sent to multiprocess
    starts multiple instances of client
    '''
    for ts, adc, psd, in detector.listmode(max_gap=1, duration=9.E6, mode=lm_mode, input_sn=SNs):
        if len(ts) == 0:  #if the acq is bad from  the bpi package, skips ROS integration
            print 'Bad acq from detector'
            continue
        else:
            c.send_msg(SNs, ts, adc)


def main():
    """
    main function
    """

    print "Building multiprocesses engine"
    pool = Pool(processes=len(sns))
    print "Starting pool with ", len(sns), ' processes'
    pool.map(multi_process, sns)



if __name__ == '__main__':
    main()
