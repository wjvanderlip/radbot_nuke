#!/usr/bin/env python

import rospy
import bpiem
# import bpimca
import numpy as np
import os
import rospkg
# rospack = rospkg.RosPack()
from rospy.numpy_msg import numpy_msg

from radbot_nuke.msg import detector_msg
import time

# rospy.init_node('para', anonymous=False)
# sn_from_launch = rospy.get_param("~my_SN")
# det_id_from_launch = rospy.get_param("~my_det")



class AcquisitionNode(object):
    """docstring for ObjectDetectionNode."""
    def __init__(self):
        print('::: init')

        # init the node
        rospy.init_node('DAQ', anonymous=False)
        self.pub = rospy.Publisher('/detector_data', detector_msg, queue_size=10)

        # Get the parameters
        self.target_sn = rospy.get_param("~my_SN")
        self.detector_id = int(rospy.get_param("~my_det"))

        # (acq_type, inttime, realtime, lm_mode, lm_maxtime, message_type) = \
        #     self.get_parameters()
        self.acq_type = 'listmode'  #'listmode'
        self.lm_mode = 0 #0 to return ts and amp
        self.lm_maxtime = 1 #max time between lm messages, set .1s
        self.inttime = 1 #hist param
        self.realtime = 1 #hst param
        self.message_type = 1  #complex msg switch

        # Create detector object
        self.detector = bpiem.EMDevice(sn=self.target_sn)
        # self.detector_id = self.detector.sn


    def get_parameters(self):
        """
        Gets the necessary parameters from parameter server

        Returns:
        (tuple) (...)

        this is not implemented, all params are hard coded in this script.
        """
        print('::: get params')
        acq_type = rospy.get_param("~acq_type")
        inttime = rospy.get_param("~inttime")
        realtime = rospy.get_param("~realtime")
        lm_mode = rospy.get_param("~lm_mode")
        lm_maxtime = rospy.get_param("~lm_maxtime")
        message_type = rospy.get_param("~message_type")
        assert acq_type in ['listmode', 'histogram']
        assert (inttime > 0.) and (inttime < 1.E3)
        assert realtime in [0, 1]
        assert lm_mode in [0, 1]
        assert message_type in [0, 1]
        print 'finished params'
        return (acq_type, inttime, realtime, lm_mode, lm_maxtime, message_type)



    def acq_from_det(self, binmode=False, rate_data=False):
        """
        In an effort to reduce system requirements for LaBr_daq, this node only publishes ch# and ts coming from
        bpimca, it also does a quick compute to determine event, trigger and pulse rates,
         a second node will do the time analysis and publish the rntools message

        Call the listmode_hr function from the bpimca package, this function has some error handeling built in and removes
        all the sleep functions between requesting list mode data.
        """
        print('::: acq from det')

        # init
        configs = self.detector.show_settings()
        print 'CONFIG SETTINGS'
        print configs

        LMDT = np.dtype({'names': ('channel', 'ts_det', 'det_sn', 'det_id'),
                         'formats': (np.uint16, np.uint32, np.unicode_, np.uint8)})

        old_acq_tme = 0
        old_ded_tme = 0
        old_dt = 0
        old_trig = 0
        old_event = 0

        while not rospy.is_shutdown():
            # try:
            if binmode is False:
                for ts, energy, psd in self.detector.listmode(max_gap=1, duration=9.E6, mode=self.lm_mode):
                    if len(ts) == 0:  #if the acq is bad from  the bpi package, skips ROS integration
                        print 'Bad acq from detector'
                        continue
                    else:
                        print self.detector_id, self.target_sn
                        msg = detector_msg()
                        now = time.time()
                        rnow = rospy.Time.now()
                        msg.header.stamp = rnow
                        msg.detid = self.detector_id
                        msg.det_sn.data = self.target_sn
                        msg.ts_sys = now
                        msg.ts_det = ts
                        msg.channel = energy

                        if rate_data is not False:
                            if rates[3] <= 0: #prevents dt frac from going to inf
                                rates[3] = 1

                            acq_tme = (rates[0] - old_acq_tme)
                            ded_tme = (rates[3] - old_ded_tme)
                            events = rates[1] - old_event
                            trigs = rates[2] - old_trig
                            active_time = (acq_tme * np.divide(65536, 80000000, dtype=np.float64)) # 16bit encoding, 80MHz, must use np.divide to return float64
                            msg.event_rate = events/active_time #event rate in seconds
                            msg.trig_rate = trigs/active_time #trigger rate in seconds
                            msg.true_rate = np.divide(trigs, active_time, dtype=np.float64) *(1 - np.divide(ded_tme, acq_tme, dtype=np.float64))  #true incident pulse rate i n seconds

                            old_acq_tme = rates[0]
                            old_ded_tme = rates[3]
                            old_event = rates[1]
                            old_trig =  rates[2]
                        self.pub.publish(msg)
                        print "published"

            else:
                print "welcome to bin mode!!!"
                hist = np.zeros(4096)
                for info in self.detector.histogram(inttime=.1, duration=9e6, realtime=True):
                    hist += info[0]  #should i pub full spectra or the spec from the buffer?

                    msg = acq()
                    now = time.time()
                    rnow = rospy.Time.now()
                    msg.header.stamp = rnow
                    msg.detid = '0'
                    msg.ts_sys = now
                    msg.ts_det = []
                    msg.channel = hist
                    msg.event_rate = info[1][6] #event rate in seconds
                    msg.trig_rate = info[1][7] #trigger rate in seconds
                    msg.true_rate = info[1][8]
                    self.pub.publish(msg)
                # except Exception as e:
            #     print "daq exception general"
            return


def main():
    """ main function
    """
    good = False
    while not good:
        try:
            node = AcquisitionNode()
            good = True
        except ValueError:
            print('::: Could not find a detector, sleepting for 10 seconds.')
            time.sleep(10.)
            good = False

    node.acq_from_det()

    return


if __name__ == '__main__':
    main()
    # Gauss_Test_loop()
