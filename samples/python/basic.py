'''
Created on Jun 18, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager
from panda3d.core import load_prc_file_data, LPoint3f
from direct.showbase.ShowBase import ShowBase
#
from common import dataDir, getCollisionEntryFromCamera, loadTerrain, mask, \
            loadPlane
            
# global data
app = None

if __name__ == '__main__':
    # Load your application's configuration
    load_prc_file_data("", "model-path " + dataDir)
    load_prc_file_data("", "win-size 1024 768")
    load_prc_file_data("", "show-frame-rate-meter #t")
    load_prc_file_data("", "sync-video #t")
        
    # Setup your application
    app = ShowBase()
       
    # # here is room for your own code
    
    print("create a steer manager; set root and mask to manage 'kinematic' vehicles")
    steerMgr = OSSteerManager(app.render, mask)

    print("reparent the reference node to render")
    steerMgr.get_reference_node_path().reparent_to(app.render)

    print("get a sceneNP and reparent to the reference node")
    sceneNP = loadPlane()
    sceneNP.reparent_to(steerMgr.get_reference_node_path())
    
    print("set sceneNP's collide mask")
    sceneNP.set_collide_mask(mask)
    
    print("create a plug in (it is attached to the reference node)")
    plugInNP = steerMgr.create_steer_plug_in()
    plugIn = plugInNP.node()
    
    print("get the model")
    modelNP = app.loader.load_model("eve.egg")
    modelNP.set_scale(10.0)

    print("create the steer vehicle (it is attached to the reference node) and set its position")
    vehicleNP = steerMgr.create_steer_vehicle("vehicle")
    vehicle = vehicleNP.node()
    vehicleNP.set_pos(750, 750.0, 0.0)
    
    print("attach the model to steer vehicle")
    modelNP.reparent_to(vehicleNP)
    
    print("attach the steer vehicle to the plug-in")
    plugIn.add_steer_vehicle(vehicleNP)

    print("start the default update task for all plug-ins")
    steerMgr.start_default_update()

    print("DEBUG DRAWING: make the debug reference node paths sibling of the reference node")
    steerMgr.get_reference_node_path_debug().reparent_to(app.render)
    steerMgr.get_reference_node_path_debug_2d().reparent_to(app.aspect2d);
    print("enable debug drawing")
    plugIn.enable_debug_drawing(app.camera)

    print("toggle debug draw")
    plugIn.toggle_debug_drawing(True)
    
#     print("set crowd vehicle move target on scene surface")
#     vehicle.set_move_target(LPoint3f(-20.5, 5.2, -2.36))
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-750.0, 400.0, -400.0);
    trackball.set_hpr(0.0, 30.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

