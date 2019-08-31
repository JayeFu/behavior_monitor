#!/usr/bin/env python

import rospy
import json

from std_msgs.msg import String

class subContainer:
    def __init__(self):
        self.container = dict()
        self.node = rospy.init_node("subContainer")
        self.pos = dict()
        self.r_ballpos = dict()

    def start_sub(self, name):
        new_callback = self.genCallback(name)
        new_sub = rospy.Subscriber("/"+name+"/positionInfo", String, new_callback, (self))
        # to pass args to callbak, add cb_args=(arg1, arg2, ...)
        # Note if only one arg is added to cb_args, i.e. cb_args=(saywhat), then cb_args is not a tuple but just saywhat
        self.container[name] = new_sub
        self.pos[name] = dict()
        self.pos[name]['x'] = 0
        self.pos[name]['y'] = 0
        self.pos[name]['t'] = 0
        
        self.r_ballpos[name] = dict()
        self.r_ballpos[name]['x'] = 0
        self.r_ballpos[name]['y'] = 0

    def genCallback(self, name):

        def callback(inJson, args):
            inJson = inJson.data
            json_data = json.loads(inJson)
            new_pos = json_data['pos']
            new_r_ballpos = json_data['ballpos']
            ref_name = json_data['name'].encode("utf-8")
            # type(args)=behavior_monitor.subContainer.subContainer
            ref_Container = args
            ref_Container.pos[ref_name]['x'] = new_pos['x']
            ref_Container.pos[ref_name]['y'] = new_pos['y']
            ref_Container.pos[ref_name]['t'] = new_pos['t']
            print "receive {}'s pos is x:{}, y:{}, theta:{}".format(json_data['name'], new_pos['x'], new_pos['y'], new_pos['t'])
            ref_Container.r_ballpos[ref_name]['x'] = new_r_ballpos['x']
            ref_Container.r_ballpos[ref_name]['y'] = new_r_ballpos['y']
            print "receive {}' seen BALL pos is x:{}, y:{}".format(json_data['name'], new_r_ballpos['x'], new_r_ballpos['y'])

        return callback

    def get_pos(self):
        return self.pos.copy()

    def get_r_ballpos(self):
        return self.r_ballpos.copy()

    def get_ballpos(self):
        ballpos = dict()
        for rname, rpos in self.pos.iteritems():
            ballpos[rname] = dict()
            ballpos[rname]['x'] = self.pos[rname]['x'] + self.r_ballpos[rname]['x']
            ballpos[rname]['y'] = self.pos[rname]['y'] + self.r_ballpos[rname]['y']
        return ballpos
