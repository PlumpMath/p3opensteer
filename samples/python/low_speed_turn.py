'''
Created on Jun 26, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager, ValueList_string
from panda3d.core import LPoint3f, TextNode
#
from common import startFramework, dataDir, getCollisionEntryFromCamera, \
            loadTerrain, mask, loadPlane, printCreationParameters, \
            handleVehicleEvent
            
# global data/functions
app = None
plugIn = None
#
def setParametersBeforeCreation():
    """set parameters as strings before plug-ins/vehicles creation"""
    
    steerMgr = OSSteerManager.get_global_ptr()
    valueList = ValueList_string()
    # set plug-in type
    steerMgr.set_parameter_value(OSSteerManager.STEERPLUGIN, "plugin_type",
            "low_speed_turn")

    # set vehicle type
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "vehicle_type",
            "low_speed_turn")

    # set vehicle throwing events
    valueList.clear()
    valueList.add_value("move@move-event@0.5")
    steerMgr.set_parameter_values(OSSteerManager.STEERVEHICLE,
            "thrown_events", valueList)
    #
    printCreationParameters()

def toggleDebugDraw():
    """toggle debug draw"""
    
    global toggleDebugFlag, plugIn
    if not plugIn:
        return

    toggleDebugFlag = not toggleDebugFlag
    plugIn.toggle_debug_drawing(toggleDebugFlag)

if __name__ == '__main__':

    app = startFramework()
       
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            "- press \"d\" to toggle debug drawing\n"
            "- press \"s\" to toggle setup cleanup\n"
            "- press \"p\" to place agents randomly\n"
            "- press \"t\", \"y\" to set agents' targets under mouse cursor\n"
            "- press \"o\" to add obstacle under mouse cursor\n"
            "- press \"shift-o\" to remove obstacle under mouse cursor\n");
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-1.25, 0.0, 0.9)
    textNodePath.set_scale(0.035)
    
    # create a steer manager; set root and mask to manage 'kinematic' vehicles
    steerMgr = OSSteerManager(app.render, mask)

    # print creation parameters: defult values
    print("\n" + "Default creation parameters:")
    printCreationParameters()

    # set creation parameters as strings before plug-in/vehicles creation
    print("\n" + "Current creation parameters:")
    setParametersBeforeCreation()

    # reparent the reference node to render
    steerMgr.get_reference_node_path().reparent_to(app.render)

    # get a sceneNP and reparent to the reference node
    sceneNP = loadPlane()
    sceneNP.reparent_to(steerMgr.get_reference_node_path())
    
    # set sceneNP's collide mask
    sceneNP.set_collide_mask(mask)
    
    # create the default plug-in (attached to the reference node)
    plugInNP = steerMgr.create_steer_plug_in()
    plugIn = plugInNP.node()
    
    # get the model
    modelNP = app.loader.load_model("eve.egg")
    modelNP.set_scale(0.25)

    # create the steer vehicle (it is attached to the reference node) and set its position
    vehicleNP = steerMgr.create_steer_vehicle("vehicle")
    vehicle = vehicleNP.node()
    vehicleNP.set_pos(-5.0, -8.0, 0.1)
    
    # attach the model to steer vehicle
    modelNP.reparent_to(vehicleNP)
    
    # add the steer vehicle to the plug-in
    plugIn.add_steer_vehicle(vehicleNP)

    # start the default update task for all plug-ins
    steerMgr.start_default_update()

    # DEBUG DRAWING: make the debug reference node paths sibling of the reference node
    steerMgr.get_reference_node_path_debug().reparent_to(app.render)
    steerMgr.get_reference_node_path_debug_2d().reparent_to(app.aspect2d);
    # enable debug drawing
    plugIn.enable_debug_drawing(app.camera)

    # # set events' callbacks
    # toggle debug draw
    toggleDebugFlag = False
    app.accept("d", toggleDebugDraw)

    # handle OSSteerVehicle(s)' events
    app.accept("move-event", handleVehicleEvent)
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(0.0, 30.0, 0.0);
    trackball.set_hpr(0.0, 20.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

