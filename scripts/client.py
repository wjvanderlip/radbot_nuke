import socket
import numpy as np
import pickle
import time


class client(object):
    def __init__(self, port_num = 8080):
        self.port = port_num
        self.eom = '<-o->'

    def connect(self):
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.connect(('0.0.0.0', self.port))

        except socket.error, exc:
            print "Caught exception socket.error : %s" % exc

    def build_message(self, sn, ts, acd):
        return pickle.dumps([sn, ts, acd])

    def disconnect(self):
        self.client.close()

    def send_msg(self, sn, ts, adc):
        try:
            t_start = time.time()
            self.connect()
            msg = self.build_message(sn, ts, adc)+self.eom
            self.client.send(msg)
#             from_server = self.client.recv(4096)
            # print "message sent in ", time.time()-t_start

        except socket.error, exc:
            print "Caught exception socket.error : %s" % exc
