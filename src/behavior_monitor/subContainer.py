#!/usr/bin/env python

import rospy
import json

from std_msgs.msg import String

class subContainer:
    def __init__(self):
        self.container = dict()
        self.node = rospy.init_node("subContainer")
        self.pos = dict()
        self.ballpos = dict()

    def start_sub(self, name):
        new_callback = self.genCallback(name)
        new_sub = rospy.Subscriber("/"+name+"/pos", String, new_callback, (self))
        # to pass args to callbak, add cb_args=(arg1, arg2, ...)
        # Note if only one arg is added to cb_args, i.e. cb_args=(saywhat), then cb_args is not a tuple but just saywhat
        self.container[name] = new_sub
        self.pos[name] = dict()
        self.pos[name]['x'] = 0
        self.pos[name]['y'] = 0
        
        self.ballpos[name] = dict()
        self.ballpos[name]['x'] = 0
        self.ballpos[name]['y'] = 0

    def genCallback(self, name):

        def callback(inJson, args):
            inJson = inJson.data
            json_data = json.loads(inJson)
            new_pos = json_data['pos']
            new_ballpos = json_data['ballpos']
            ref_name = json_data['name'].encode("utf-8")
            # type(args)=behavior_monitor.subContainer.subContainer
            ref_Container = args
            ref_Container.pos[ref_name]['x'] = new_pos['x']
            ref_Container.pos[ref_name]['y'] = new_pos['y']
            print "receive {}'s pos is x:{}, y:{}".format(json_data['name'], new_pos['x'], new_pos['y'])
            ref_Container.ballpos[ref_name]['x'] = new_ballpos['x']
            ref_Container.ballpos[ref_name]['y'] = new_ballpos['y']
            print "receive {}' BALL pos is x:{}, y:{}".format(json_data['name'], new_ballpos['x'], new_ballpos['y'])

        return callback

    def get_pos(self):
        return self.pos

    def get_ballpos(self):
        return self.ballpos
