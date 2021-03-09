#!/usr/bin/env python


import numpy as np
import rospy
import time

from radbot_nuke.msg import detector_msg, array_msg, rad_rate_msg

class combineArray(object):
    def __init__(self):
        self.detid = []
        self.detsn = []
        self.ch = []
        self.ts_raw = []
        self.ts_rolled = []
        self.ts_sync = []
        self.start_times = [False, False, False, False, False]
        self.roll_tracker = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.roll_check = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.inter_sum = 0
        self.sr = 80e6
        self.roll_val = (2**32)/self.sr
        self.pubtime = 0.0
        self.pub_time = time.time()

        self.full = np.dtype({"names":["detID", "detSN", "Channel", "raw_detTS", "rolled_detTS", "syncTS"],
                    "formats":[np.int8, np.int16, np.int32, np.int32, np.float32, np.float32]})
        self.cut_low = [531, 1653, 1653, 5840,5840]
        self.loop_rate = rospy.Rate(10)

        self.pub = rospy.Publisher('/rad_array_data', array_msg, queue_size=10)
        self.map = rospy.Publisher('/radrates', rad_rate_msg, queue_size=10)
        self.calcvector = True

        rospy.Subscriber('/detector_data', detector_msg, self.rollover, queue_size=100)


    def rollover(self, msg):
        detID = msg.detid-1
        if self.start_times[detID] is False:
            self.start_times[detID] = np.divide(msg.ts_det[0], self.sr, dtype=np.float64)

        n_inter = len(msg.ts_det)
        self.inter_sum = self.inter_sum + n_inter
        for i in range(n_inter):
            #checks to see if roll over is accounted for
            if self.roll_tracker[detID] == 0.0:
                self.current_measurement_time = np.divide(msg.ts_det[i], self.sr, dtype=np.float64)
            else:
                self.current_measurement_time = np.divide(msg.ts_det[i], self.sr, dtype=np.float64) + self.roll_tracker[detID]

            if self.current_measurement_time + 30 < self.roll_check[detID]:
                self.roll_tracker[detID] += self.roll_val
                self.current_measurement_time = np.divide(msg.ts_det[i], self.sr, dtype=np.float64) + self.roll_tracker[detID]

            self.roll_check[detID] = self.current_measurement_time
            ## rolls over at ~57 (2^32/80MHz) seconds

            self.detid.append(int(msg.detid))
            self.detsn.append(int(msg.det_sn.data[3:]))
            self.ch.append(msg.channel[i])
            self.ts_raw.append(msg.ts_det[i])
            self.ts_rolled.append(self.current_measurement_time)
            self.ts_sync.append(self.current_measurement_time-self.start_times[detID])
        self.inter_sum = 0

        if self.pub_time + .1 <= time.time():
            m = array_msg()
            m.idlist = self.detid
            m.snlist = self.detsn
            m.ch_val = self.ch
            m.raw_times = self.ts_raw
            m.rolled_times = self.ts_rolled
            m.sync_times = self.ts_sync
#             m.raw_rate = len(self.ch)
#             m.gate_rate = np.asarray(self.ch)
            self.pub.publish(m)

            if self.calcvector is True:
                main = np.zeros(len(self.detid),dtype=self.full)
                main['detID']=self.detid
#                 main['detSN']=self.detsn
                main['Channel']=self.ch
#                 main['raw_detTS'] = self.ts_raw
#                 main['rolled_detTS']=self.ts_rolled
                main['syncTS']=self.ts_sync

                main = np.sort(main, order='syncTS')
                rates_by_det = []
                for num in range(6)[1:]:
                    mask = np.all([main['detID']== num, main['Channel'] > self.cut_low[num-1]], axis=0)
                    events = len(main['Channel'][mask])
                    rates_by_det.append(events)

                rates_by_det.append(np.sum(rates_by_det[:]))

                rr = rad_rate_msg()
                rnow = rospy.Time.now()
                rr.header.stamp = rnow
                rr.cr = rates_by_det
                self.map.publish(rr)

            self.pub_time = time.time()

            self.detid = []
            self.detsn = []
            self.ch = []
            self.ts_raw = []
            self.ts_rolled = []
            self.ts_sync = []

def main():

    try:
        print 'Starting'
        rospy.init_node('combineDets', anonymous=False)
        combineArray()
        rospy.spin()
    # my_node = Nodo()
    # my_node.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
