'''
Created on Sep 10, 2016

@author: consultit
'''

from panda3d.core import ClockObject
from p3opensteer import OSSteerManager
#
from common import startFramework, mask, loadPlane

globalClock = None

def vehicleCallback(vehicle):
    """vehicle update callback function"""
    
    print(vehicle.get_name() + " params:")
    print(vehicle.get_settings())

def plugInCallback(plugIn):
    """plug-in update callback function"""
    
    global globalClock
    print(plugIn.get_name() + " real time and dt: "
          + str(globalClock.get_real_time()) + str(globalClock.get_dt()))
            
if __name__ == '__main__':

    app = startFramework("'one turning'")
       
    # # here is room for your own code
    
    print("create a steer manager; set root and mask to manage 'kinematic' vehicles")
    steerMgr = OSSteerManager(app.render, mask)

    print("reparent the reference node to render")
    steerMgr.get_reference_node_path().reparent_to(app.render)

    print("get a sceneNP and reparent to the reference node")
    sceneNP = loadPlane("SceneNP")
    sceneNP.reparent_to(steerMgr.get_reference_node_path())
    
    print("set sceneNP's collide mask")
    sceneNP.set_collide_mask(mask)
    
    print("create the default plug-in (attached to the reference node): 'one turning'")
    plugInNP = steerMgr.create_steer_plug_in()
    plugIn = plugInNP.node()
    plugIn.set_update_callback(plugInCallback)
    
    print("get the model")
    modelNP = app.loader.load_model("eve.egg")
    modelNP.set_scale(0.25)

    print("create the steer vehicle (it is attached to the reference node) and set its position")
    vehicleNP = steerMgr.create_steer_vehicle("vehicle")
    vehicle = vehicleNP.node()
    vehicleNP.set_pos(5.0, -8.0, 0.1)
    vehicle.set_update_callback(vehicleCallback)
    globalClock = ClockObject.get_global_clock()
    
    print("attach the model to steer vehicle")
    modelNP.reparent_to(vehicleNP)
    
    print("add the steer vehicle to the plug-in")
    plugIn.add_steer_vehicle(vehicleNP)

    print("start the default update task for all plug-ins")
    steerMgr.start_default_update()

    print("DEBUG DRAWING: make the debug reference node paths sibling of the reference node")
    steerMgr.get_reference_node_path_debug().reparent_to(app.render)
    steerMgr.get_reference_node_path_debug_2d().reparent_to(app.aspect2d)
    print("enable debug drawing")
    plugIn.enable_debug_drawing(app.camera)

    print("toggle debug draw")
    plugIn.toggle_debug_drawing(True)
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(0.0, 30.0, 0.0)
    trackball.set_hpr(0.0, 20.0, 0.0)
   
    # app.run(), equals to do the main loop in C++
    app.run()

