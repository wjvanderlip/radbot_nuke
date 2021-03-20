

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import rosbag
import rospy

def getbag(fileName):
    return rosbag.Bag(fileName)

def extractTrack(bagfile):
    mapdtype = np.dtype({"names":["x", "y", "z", "rosSec", "rosNsec","rosSysTme", "run_time", 'msg_time'],
                     "formats":[np.float32, np.float32, np.float32, np.float32, np.float32, np.float32, np.float32, np.float64]})

    xpos = []
    ypos = []
    zpos = []
    t_sec = []
    t_nsec = []
    msgtme = []
    ndx = 0
    for topic, msg, ts in bagfile.read_messages(topics=['/trajectory_node_list', '/gx5/imu/data', '/map']):
        if topic == '/trajectory_node_list':
            hold = msg

            if len(hold.markers[2].points)-ndx != 0:

                for i in range(len(hold.markers[2].points))[ndx:]:
                    x = hold.markers[2].points[i].x
                    y = hold.markers[2].points[i].y
                    z = hold.markers[2].points[i].z
                    xpos.append(x)
                    ypos.append(y)
                    zpos.append(z)
                    t_sec.append(hold.markers[2].header.stamp.secs)
                    t_nsec.append(hold.markers[2].header.stamp.nsecs)
                    msgtme.append(rospy.Time(hold.markers[2].header.stamp.secs, hold.markers[2].header.stamp.nsecs).to_sec())
                ndx = len(hold.markers[2].points)

    s = np.asarray(t_sec)
    ns = np.asarray(t_nsec)/1e9
    runtm = s+ns
    runtm = runtm-runtm[0]

    path = np.zeros(len(xpos), dtype=mapdtype)
    path["x"] =xpos
    path["y"] = ypos
    path["z"] = zpos
    path["rosSec"] = s
    path["rosNsec"] = ns
    path['rosSysTme'] = s+ns
    path["run_time"] = runtm
    path['msg_time'] = msgtme
    mask = path["run_time"]>0.0
    path = path[mask]

    return path

def extractRadData(bagfile):
    '''
    Reads the /radrates topic and returns a numpy structured away with total count rate and time stamp data
    '''
    raddtype = np.dtype({"names":["det1", "det2", "det3", "det4", "det5",'total', "cr_norm", "x", "y", "rosSec", "rosNsec",'rosSysTme', "run_time", "msg_time"],
                         "formats":[np.int16, np.int16, np.int16, np.int16, np.int16, np.int16, np.float32, np.float32, np.float32, np.float32, np.float32, np.float32, np.float32, np.float64]})

    # bag2 = rosbag.Bag('arraytopic.bag')
    det1 = []
    det2 = []
    det3 = []
    det4 = []
    det5 = []
    total = []
    rsec = []
    rnsec = []
    msg_times = []
    for topic, msg, ts in bagfile.read_messages(topics=['/radrates']):
        det1.append(msg.cr[0])#np.sum(msg.cr[:3])+msg.cr[4])
        det2.append(msg.cr[1])
        det3.append(msg.cr[2])
        det4.append(msg.cr[3])
        det5.append(msg.cr[4])
        total.append(msg.cr[5])
        rsec.append(msg.header.stamp.secs)
        rnsec.append(msg.header.stamp.nsecs)
        t = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec() #needs to be 64bit
        msg_times.append(t)

    arr = np.asarray(total)
    normCR = np.divide(arr, arr.max(), dtype=np.float32)

    rs = np.asarray(rsec)
    rns = np.asarray(rnsec)/1e9
    Rruntm = rs+rns
    Rruntm = Rruntm-Rruntm[0]

    radmap = np.zeros(len(arr), dtype=raddtype)
    radmap["det1"] = det1
    radmap["det2"] = det2
    radmap["det3"] = det3
    radmap["det4"] = det4
    radmap["det5"] = det5
    radmap["total"] = total # total of counts published every 10Hz
    radmap['cr_norm'] = normCR
    radmap["rosSec"] = rs
    radmap["rosNsec"] = rns
    radmap['rosSysTme'] = rs+rns
    radmap["run_time"] =Rruntm
    radmap['msg_time'] = msg_times

    return radmap

def match_data_length(trackArr, radArr):
    '''
    Because the detector publishes at a higher rate there are more detector data poinits, the path topic appears to publish at a dynamic rate based on
    speed leaving varying time gaps between points. This funcation creats x, y position arrays and fills zero where there is detector data but no position data.
    This will then go into an interpolation funcation to fill in the missing position data.
    '''
    c=0
    rad_x = []
    rad_y = []
    n = 0
    # for d in radmap["run_time"]:
    for d in radArr["msg_time"]:
        n+=1
        if c >=len(trackArr):
            rad_x.append(0)
            rad_y.append(0)
    #     elif d < path['run_time'][c]:
        elif d < trackArr['msg_time'][c]:
            rad_x.append(0)
            rad_y.append(0)
        else:
            rad_x.append(trackArr['x'][c])
            rad_y.append(trackArr['y'][c])
            c+=1
    return rad_x, rad_y

def interpolatePath(rx, ry):
    aa = pd.Series(rx)
    bb = pd.Series(ry)

    aa.replace(0.0, np.NaN, inplace=True)
    bb.replace(0.0, np.NaN, inplace=True)

    rad_x_full = aa.interpolate()
    rad_y_full = bb.interpolate()

    return rad_x_full, rad_y_full

def dataMerge(trackArr, radArr):
    a, b = match_data_length(trackArr, radArr)
    x, y = interpolatePath(a, b)
    return x, y

def colorizer(data, circles=2.2, data_param='total', log=False):
    colors = data[data_param]
    circ_size = (data['total']/10)**circles
    return colors, circ_size

def plotter(x, y, colors, circparam):

    fig, ax1 = plt.subplots(figsize=(10,6))
    line, = ax1.plot(-1*y, x)
    scat = ax1.scatter(-1*y, x,  c=colors, s=circparam, cmap='viridis', alpha=.5)
    cbar = plt.colorbar(scat)
    cbar.set_label('CPS', rotation=0)

    plt.title("A Map!")
    ax1.set_xlabel("X Position")
    ax1.set_ylabel("Y Position")
    # plt.savefig('localization_map.pdf', dpi = 800)
    plt.show()

def giveMeMap(f_name):
    bag = getbag(f_name)
    track = extractTrack(bag)
    raddata = extractRadData(bag)
    xpos, ypos = dataMerge(track, raddata)
    colors, circles = colorizer(raddata, circles=2.2, data_param='total', log=False)
    plotter(xpos, ypos, colors, circles)
