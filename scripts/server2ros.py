#!/usr/bin/env python

import rospy
import numpy as np
import pickle
import time
import socket
from rospy.numpy_msg import numpy_msg
from radbot_nuke.msg import detector_msg


rospy.init_node('detServer', anonymous=False)
pub = rospy.Publisher('/detector_data', detector_msg, queue_size=10)

def send_ros_msg(ts, adc, det_sn):
    '''
    takes output from multi_process, packages it and publishes as message
    '''
    msg = detector_msg()
    now = time.time()
    rnow = rospy.Time.now()
    msg.header.stamp = rnow
    msg.det_sn.data = det_sn
    msg.ts_sys = now
    msg.ts_det = ts
    msg.channel = adc
    #process rate data if needed
    pub.publish(msg)
    print len(ts), len(adc), det_sn, " Published!!"

def process_rates(rates):
    # old_acq_tme = 0
    # old_ded_tme = 0
    # old_dt = 0
    # old_trig = 0
    # old_event = 0
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


def start_server(port):
    try:
        serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serv.bind(('0.0.0.0', port))
        serv.listen(5)
        buf = None
        print "Waiting to receive connection..."
        while True:
            conn, addr = serv.accept()
            while True:
                data = conn.recv(4096)
                if buf is None:
                    buf = data
                else:
                    buf+=data

                if '<-o->' in buf:
                    message = buf[:-len('<-o->')]
                    # PROCESS MESSAGE HERE
                    incoming_data = pickle.loads(message)
        #             print incoming_data[0], len(incoming_data[1]), len(incoming_data[2])
                    send_ros_msg(incoming_data[1], incoming_data[2], incoming_data[0]) #ts, acd, sn
                    buf = None
                    conn.close()
                    break
            # print 'client disconnected'

    except socket.error, exc:
        print "Caught exception socket.error : %s" % exc

def main():
    start_server(port=8082)

if __name__ == '__main__':
    main()
