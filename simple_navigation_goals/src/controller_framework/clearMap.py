#!/usr/bin/env python

import rospy
import std_srvs.srv
import dynamic_reconfigure.client #this lets you reconfigure node parameters

#This code is written with some guidance from Dr. Eric Schneider concerning obstacle layer reconfigurement 


#cleans the obstacle layer of the map in two different ways
class mapCleaner:
    def __init__(self):
        self.srv_name = "/move_base/clear_costmaps"  # this is the full path name of the service
        self.clear_costmaps = rospy.ServiceProxy(self.srv_name, std_srvs.srv.Empty)

        self.static_layer = "/move_base/global_costmap/static_layer"
        self.static_layer_enabled = "{0}/enabled".format(self.static_layer)
        self.obs_layer = "/move_base/global_costmap/obstacle_layer"
        self.obs_layer_enabled = "{0}/enabled".format(self.obs_layer)

        self.static_client = dynamic_reconfigure.client.Client(self.static_layer, timeout=5)
        self.obs_client = dynamic_reconfigure.client.Client(self.obs_layer, timeout=5)

        self.r = rospy.Rate(5)

#clears obstacles in robot's laser radius
    def clearMapClient(self):
        try:
            rospy.wait_for_service(self.srv_name)
            self.clear_costmaps()
            return True
        except rospy.ServiceException:
            rospy.logerr("{0} service call failed: {1}".format(self.srv_name))
            return False

#reconfigures the obstacle layer by switching it on or off
    def set_layers(self, enabled):
        self.static_client.update_configuration({"enabled": enabled})
        # Wait until the parameter has been set
        rospy.Duration(2)

        self.obs_client.update_configuration({"enabled": enabled})
        # Wait until the parameter has been set
        rospy.Duration(2)

        return True

#resets the map to its initial state by turning off and on the obstacle layer
    def reset_costmap_layers(self):
        self.set_layers(False)
        self.set_layers(True)
        return True
'''
if __name__ == "__main__":
    clearMap = mapCleaner()
    #clearMap.clearMapClient() # this is a small clear for the area around a robot
    #full cleanser:
    success = clearMap.reset_costmap_layers() # full map cleanser
'''
