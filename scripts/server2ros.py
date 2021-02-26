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
    takes output from client, packages it, and publishes as message
    '''
    msg = detector_msg()
    now = time.time()
    rnow = rospy.Time.now()
    msg.header.stamp = rnow
    msg.det_sn.data = det_sn
    msg.detid = assign_det_num(det_sn)
    msg.ts_sys = now
    msg.ts_det = ts
    msg.channel = adc
    msg.event_rate = len(adc)*1.0
    #process rate data if needed
    pub.publish(msg)
    print len(ts), len(adc), det_sn, " Published!!"

def assign_det_num(ser_no):
    assignments = {'eRC4129':1, 'eRC4216':3, 'eRC4131':2, 'eRC4214':5, 'eRC4130':4}
    return assignments[ser_no]

def process_rates(rates):
    '''
    If we start to hit the list mode buffer throughput, this parses
    the "rate data" flag returned with each eMorpho buffer readout. Using the
    live and dead time values you can assess higher count rate envs

    https://www.bridgeportinstruments.com/products/mds/mds_doc/read_rates.php
    '''
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
    '''
    Starts a server to constantly monior on some port for incoming data from one of the
    multiprocessing subprocesses. The clinet will send a pickeled list consiting of the detectors
    serial number, an array of event time stamps and an array of event ACD (channel) values

    This data is then published on the /detector_data topic using the detector_msg message
    '''
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
