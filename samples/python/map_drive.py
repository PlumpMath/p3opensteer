'''
Created on Jun 26, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager, ValueList_string, ValueList_LPoint3f, \
        ValueList_float
from panda3d.core import TextNode, ClockObject, AnimControlCollection, \
        auto_bind, LPoint3f, LVecBase3f
#
from common import startFramework, toggleDebugFlag, toggleDebugDraw, mask, \
        loadTerrain, printCreationParameters, handleVehicleEvent, \
        changeVehicleMaxForce, changeVehicleMaxSpeed, getVehicleModelAnims, \
        animRateFactor, writeToBamFileAndExit, readFromBamFile, bamFileName, \
        getCollisionEntryFromCamera, obstacleFile, HandleObstacleData, \
        handleObstacles, HandleVehicleData, handleVehicles, loadPlane, \
        loadTerrainLowPoly
import sys, random
        
# # specific data/functions declarations/definitions
sceneNP = None
vehicleAnimCtls = []
steerPlugIn = None
steerVehicles = []
#
def setParametersBeforeCreation():
    """set parameters as strings before plug-ins/vehicles creation"""
    
    steerMgr = OSSteerManager.get_global_ptr()
    valueList = ValueList_string()
    # set plug-in type
    steerMgr.set_parameter_value(OSSteerManager.STEERPLUGIN, "plugin_type",
            "pedestrian")

    # set vehicle's type, mass, speed
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "vehicle_type",
            "pedestrian")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "mass",
            "2.0")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "speed",
            "0.01")

    # set vehicle throwing events
    valueList.clear()
    valueList.add_value("avoid_obstacle@avoid_obstacle@1.0:avoid_close_neighbor@avoid_close_neighbor@")
    steerMgr.set_parameter_values(OSSteerManager.STEERVEHICLE,
            "thrown_events", valueList)
    #
    printCreationParameters()

def toggleWanderBehavior():
    """toggle wander behavior of last inserted vehicle"""
    
    global steerVehicles
    if len(steerVehicles) == 0:
        return
    
    if steerVehicles[-1].get_wander_behavior():
        steerVehicles[-1].set_wander_behavior(False)
    else:
        steerVehicles[-1].set_wander_behavior(True)
    print(str(steerVehicles[-1]) + "'s wander behavior is " + str(steerVehicles[-1].get_wander_behavior()))

def updatePlugIn(steerPlugIn, task):
    """custom update task for plug-ins"""
    
    global steerVehicles, vehicleAnimCtls
    # call update for plug-in
    dt = ClockObject.get_global_clock().get_dt()
    steerPlugIn.update(dt)
    # handle vehicle's animation
    for i in range(len(vehicleAnimCtls)):
        # get current velocity size
        currentVelSize = steerVehicles[i].get_speed()
        if currentVelSize > 0.0:
            if currentVelSize < 4.0: 
                animOnIdx = 0
            else:
                animOnIdx = 1
            animOffIdx = (animOnIdx + 1) % 2
            # Off anim (0:walk, 1:run)
            if vehicleAnimCtls[i][animOffIdx].is_playing():
                vehicleAnimCtls[i][animOffIdx].stop()
            # On amin (0:walk, 1:run)
            vehicleAnimCtls[i][animOnIdx].set_play_rate(currentVelSize / animRateFactor[animOnIdx])
            if not vehicleAnimCtls[i][animOnIdx].is_playing():
                vehicleAnimCtls[i][animOnIdx].loop(True)
        else:
            # stop any animation
            vehicleAnimCtls[i][0].stop()
            vehicleAnimCtls[i][1].stop()
    #
    return task.cont
        
if __name__ == '__main__':

    msg = "'map drive'"
    app = startFramework(msg)
      
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            msg + "\n\n"      
            "- press \"d\" to toggle debug drawing\n"
            "- press \"a\"/\"k\" to add 'opensteer'/'kinematic' vehicle\n"
            "- press \"s\"/\"shift-s\" to increase/decrease last inserted vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease last inserted vehicle's max force\n"
            "- press \"t\" to toggle last inserted vehicle's wander behavior\n"
            "- press \"o\"/\"shift-o\" to add/remove obstacle\n")
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-1.25, 0.0, 0.6)
    textNodePath.set_scale(0.035)
    
    # create a steer manager; set root and mask to manage 'kinematic' vehicles
    steerMgr = OSSteerManager(app.render, mask)

    # print creation parameters: defult values
    print("\n" + "Default creation parameters:")
    printCreationParameters()

    # load or restore all scene stuff: if passed an argument
    # try to read it from bam file
    if (not len(sys.argv) > 1) or (not readFromBamFile(sys.argv[1])):
        # no argument or no valid bamFile
        # reparent the reference node to render
        steerMgr.get_reference_node_path().reparent_to(app.render)
    
        # get a sceneNP, naming it with "SceneNP" to ease restoring from bam 
        # file
        sceneNP = loadTerrainLowPoly("SceneNP", 64, 24)
        # and reparent to the reference node
        sceneNP.reparent_to(steerMgr.get_reference_node_path())
        
        # set sceneNP's collide mask
        sceneNP.set_collide_mask(mask)

        # set creation parameters as strings before plug-in/vehicles creation
        print("\n" + "Current creation parameters:")
        setParametersBeforeCreation()
        
        # create the plug-in (attached to the reference node)
        plugInNP = steerMgr.create_steer_plug_in()
        steerPlugIn = plugInNP.node()
    
        # set the pathway
        pointList = ValueList_LPoint3f()
        radiusList = ValueList_float()
        pointList.add_value(LPoint3f(-33.2366, 20.8779, 0.224516))
        radiusList.add_value(4)
        pointList.add_value(LPoint3f(-2.17519, 31.4712, 0.157774))
        radiusList.add_value(5)
        pointList.add_value(LPoint3f(7.57217, 11.0209, -0.147097))
        radiusList.add_value(6)
        pointList.add_value(LPoint3f(31.4319, 6.31322, -0.142874))
        radiusList.add_value(6)
        pointList.add_value(LPoint3f(33.0984, -21.5953, -0.158049))
        radiusList.add_value(5)
        pointList.add_value(LPoint3f(10.8928, -33.0309, -0.331787))
        radiusList.add_value(5)
        pointList.add_value(LPoint3f(0.649722, -16.8083, 0.227074))
        radiusList.add_value(4)
        pointList.add_value(LPoint3f(-25.1445, -28.6898, 0.0573864))
        radiusList.add_value(3)
        pointList.add_value(LPoint3f(-43.6806, -14.6532, -0.0712051))
        radiusList.add_value(3)
        pointList.add_value(LPoint3f(-46.9489, 8.38837, -0.222353))
        radiusList.add_value(4)
        steerPlugIn.set_pathway(pointList, radiusList, True, True)
    else:
        # valid bamFile
        # restore plug-in: through steer manager
        steerPlugInNP = OSSteerManager.get_global_ptr().get_steer_plug_in(0)
        steerPlugIn = steerPlugInNP.node()
        # restore sceneNP: through panda3d
        sceneNP = OSSteerManager.get_global_ptr().get_reference_node_path().find("**/SceneNP")
        # reparent the reference node to render
        OSSteerManager.get_global_ptr().get_reference_node_path().reparent_to(app.render)
    
        # restore steer vehicles
        NUMVEHICLES = OSSteerManager.get_global_ptr().get_num_steer_vehicles()
        tmpList = [None for i in range(NUMVEHICLES)]
        steerVehicles.extend(tmpList)
        vehicleAnimCtls.extend(tmpList)
        for i in range(NUMVEHICLES):
            # restore the steer vehicle: through steer manager
            steerVehicleNP = OSSteerManager.get_global_ptr().get_steer_vehicle(i)
            steerVehicles[i] = steerVehicleNP.node()
            # restore animations
            tmpAnims = AnimControlCollection()
            auto_bind(steerVehicles[i], tmpAnims)
            vehicleAnimCtls[i] = [None, None];
            for j in range(tmpAnims.get_num_anims()):
                vehicleAnimCtls[i][j] = tmpAnims.get_anim(j)

        # set creation parameters as strings before other plug-ins/vehicles creation
        print("\n" + "Current creation parameters:")
        setParametersBeforeCreation()

    # # first option: start the default update task for all plug-ins
#     steerMgr.start_default_update()

    # # second option: start the custom update task for all plug-ins
    app.taskMgr.add(updatePlugIn, "updatePlugIn", extraArgs=[steerPlugIn], 
                    appendTask=True)

    # DEBUG DRAWING: make the debug reference node paths sibling of the reference node
    steerMgr.get_reference_node_path_debug().reparent_to(app.render)
    steerMgr.get_reference_node_path_debug_2d().reparent_to(app.aspect2d);
    # enable debug drawing
    steerPlugIn.enable_debug_drawing(app.camera)
    # print debug draw texture
    steerPlugIn.debug_drawing_to_texture(sceneNP, app.win)

    # # set events' callbacks
    # toggle debug draw
    toggleDebugFlag = False
    app.accept("d", toggleDebugDraw, [steerPlugIn])

    # handle addition steer vehicles, models and animations 
    vehicleData = HandleVehicleData(0.7, 0, "opensteer", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("a", handleVehicles, [vehicleData])
    vehicleDataKinematic = HandleVehicleData(0.7, 1, "kinematic", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("k", handleVehicles, [vehicleDataKinematic])

    # handle obstacle addition
    obstacleAddition = HandleObstacleData(True, sceneNP, steerPlugIn,
                        LVecBase3f(0.03, 0.03, 0.03))
    app.accept("o", handleObstacles, [obstacleAddition])
    # handle obstacle removal
    obstacleRemoval = HandleObstacleData(False, sceneNP, steerPlugIn)
    app.accept("shift-o", handleObstacles, [obstacleRemoval]);

    # increase/decrease last inserted vehicle's max speed
    app.accept("s", changeVehicleMaxSpeed, ["s", steerVehicles])
    app.accept("shift-s", changeVehicleMaxSpeed, ["shift-s", steerVehicles])
    # increase/decrease last inserted vehicle's max force
    app.accept("f", changeVehicleMaxForce, ["f", steerVehicles])
    app.accept("shift-f", changeVehicleMaxForce, ["shift-f", steerVehicles])
    
    # handle OSSteerVehicle(s)' events
    app.accept("avoid_obstacle", handleVehicleEvent, ["avoid_obstacle"])
    app.accept("avoid_close_neighbor", handleVehicleEvent, ["avoid_close_neighbor"])
    
    # write to bam file on exit
    app.win.set_close_request_event("close_request_event")
    app.accept("close_request_event", writeToBamFileAndExit, [bamFileName])

    # 'pedestrian' specific: toggle wander behavior
    app.accept("t", toggleWanderBehavior)
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(0.0, 180.0, -10.0);
    trackball.set_hpr(0.0, 20.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

