'''
Created on Jun 26, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager, ValueList_string, ValueList_LPoint3f, \
        ValueList_float
from panda3d.core import TextNode, ClockObject, AnimControlCollection, \
        auto_bind, LPoint3f
#
from common import startFramework, toggleDebugFlag, toggleDebugDraw, mask, \
        loadTerrain, printCreationParameters, handleVehicleEvent, \
        changeVehicleMaxForce, changeVehicleMaxSpeed, getVehicleModelAnims, \
        rateFactor, writeToBamFileAndExit, readFromBamFile, bamFileName, \
        getCollisionEntryFromCamera, obstacleFile
import sys, random
        
# # specific data/functions declarations/definitions
sceneNP = None
vehicleNP = []
vehicleAnimCtls = []
steerPlugIn = None
steerVehicle = []
#
def setParametersBeforeCreation():
    """set parameters as strings before plug-ins/vehicles creation"""
    
    steerMgr = OSSteerManager.get_global_ptr()
    valueList = ValueList_string()
    # set plug-in type
    steerMgr.set_parameter_value(OSSteerManager.STEERPLUGIN, "plugin_type",
            "pedestrian")

    # set vehicle type, mass, speed
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "vehicle_type",
            "pedestrian")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "mass",
            "2.0")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "speed",
            "0.01")

    # set vehicle throwing events
    valueList.clear()
    valueList.add_value("move@move-event@0.5")
    steerMgr.set_parameter_values(OSSteerManager.STEERVEHICLE,
            "thrown_events", valueList)
    #
    printCreationParameters()

def toggleSteeringSpeed():
    """toggle steering speed"""
    
    global steerVehicle
    if steerVehicle[0].get_steering_speed() < 4.9:
        steerVehicle[0].set_steering_speed(5.0)
    else:
        steerVehicle[0].set_steering_speed(1.0)
    print(str(steerVehicle[0]) + "'s steering speed is " + str(steerVehicle[0].get_steering_speed()))

def updatePlugIn(steerPlugIn, task):
    """custom update task for plug-ins"""
    
    global steerVehicle, vehicleAnimCtls
    # call update for plug-in
    dt = ClockObject.get_global_clock().get_dt()
    steerPlugIn.update(dt)
    # handle vehicle's animation
    for i in range(len(vehicleAnimCtls)):
        # get current velocity size
        currentVelSize = steerVehicle[i].get_speed()
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
            vehicleAnimCtls[i][animOnIdx].set_play_rate(currentVelSize / rateFactor[animOnIdx])
            if not vehicleAnimCtls[i][animOnIdx].is_playing():
                vehicleAnimCtls[i][animOnIdx].loop(True)
        else:
            # stop any animation
            vehicleAnimCtls[i][0].stop()
            vehicleAnimCtls[i][1].stop()
    #
    return task.cont


def handleObstacles(data):
    """handle add/remove obstacles""" #XXX
    
    global app, sceneNP, steerPlugIn

    addObstacle = data
    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        # get the hit object
        hitObject = entry0.get_into_node_path()
        print("hit " + str(hitObject) + " object")

        # check if we want add obstacle and
        # if sceneNP is the hitObject or an ancestor thereof
        if addObstacle and ((sceneNP == hitObject) or sceneNP.is_ancestor_of(hitObject)):
            # the hit object is the scene: add an obstacle to the scene
            # get a model as obstacle
            obstacleNP = app.loader.load_model(obstacleFile)
            obstacleNP.set_collide_mask(mask)
            # set random scale (0.03 - 0.04)
            scale = 0.03 + 0.01 * random.uniform(0.0, 1.0)
            obstacleNP.set_scale(scale)
            # set obstacle position
            pos = entry0.get_surface_point(sceneNP)
            obstacleNP.set_pos(sceneNP, pos)
            # try to add to plug-in
            if steerPlugIn.add_obstacle(obstacleNP, "box") < 0:
                # something went wrong remove from scene
                obstacleNP.remove_node()
                return
            print("added " + str(obstacleNP) + " obstacle.")
        # check if we want remove obstacle
        elif not addObstacle:
            # cycle through the local obstacle list
            for index in range(steerPlugIn.get_num_obstacles()):
                # get the obstacle's NodePath
                ref = steerPlugIn.get_obstacle(index)
                obstacleNP = OSSteerManager.get_global_ptr().get_obstacle_by_ref(ref)
                # check if obstacleNP is the hitObject or an ancestor thereof
                if (obstacleNP == hitObject) or obstacleNP.is_ancestor_of(hitObject):
                    # try to remove from plug-in
                    if not steerPlugIn.remove_obstacle(ref).is_empty():
                        # all ok remove from scene
                        print("removed " + str(obstacleNP) + " obstacle.")
                        obstacleNP.remove_node()
                        break
        
if __name__ == '__main__':

    msg = "'pedestrian'"
    app = startFramework(msg)
      
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            msg + "\n\n"      
            "- press \"d\" to toggle debug drawing\n"
            "- press \"s\"/\"shift-s\" to increase/decrease vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease vehicle's max force\n"
            "- press \"t\" to toggle steering speed\n")
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-1.25, 0.0, -0.5)
    textNodePath.set_scale(0.035)
    
    # create a steer manager; set root and mask to manage 'kinematic' vehicles
    steerMgr = OSSteerManager(app.render, mask)

    # print creation parameters: defult values
    print("\n" + "Default creation parameters:")
    printCreationParameters()

    # set creation parameters as strings before plug-in/vehicles creation
    print("\n" + "Current creation parameters:")
    setParametersBeforeCreation()

    # load or restore all scene stuff: if passed an argument
    # try to read it from bam file
    if (not len(sys.argv) > 1) or (not readFromBamFile(sys.argv[1])):
        # no argument or no valid bamFile
        # reparent the reference node to render
        steerMgr.get_reference_node_path().reparent_to(app.render)
    
        # get a sceneNP, naming it with "SceneNP" to ease restoring from bam 
        # file
        sceneNP = loadTerrain("SceneNP")
        # and reparent to the reference node
        sceneNP.reparent_to(steerMgr.get_reference_node_path())
        
        # set sceneNP's collide mask
        sceneNP.set_collide_mask(mask)
        
        # create the plug-in (attached to the reference node)
        plugInNP = steerMgr.create_steer_plug_in()
        steerPlugIn = plugInNP.node()
    
        # set the pathway
        pointList = ValueList_LPoint3f()
        pointList.add_value(LPoint3f(79.474, 51.7236, 2.0207))
        pointList.add_value(LPoint3f(108.071, 51.1972, 2.7246))
        pointList.add_value(LPoint3f(129.699, 30.1742, 0.720501))
        pointList.add_value(LPoint3f(141.597, 73.496, 2.14218))
        pointList.add_value(LPoint3f(105.917, 107.032, 3.06428))
        pointList.add_value(LPoint3f(61.2637, 109.622, 3.03588))
        # note: pedestrian handles single radius pathway only 
        radiusList = ValueList_float()
        radiusList.add_value(4)
        steerPlugIn.set_pathway(pointList, radiusList, True, True)
    
        # get steer vehicles, models and animations   
        #1: get the models and attach animations to them
        #2: create the steer vehicles (attached to the reference node)
        #3: set steer vehicles' positions
        #4: attach the models to steer vehicles
        #5: add the steer vehicles to the plug-in
        getVehicleModelAnims(0.7, 0, "kinematic", sceneNP, vehicleNP, steerPlugIn, 
                       steerVehicle, vehicleAnimCtls)
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
        steerVehicle.extend(tmpList)
        vehicleAnimCtls.extend(tmpList)
        for i in range(NUMVEHICLES):
            # restore the steer vehicle: through steer manager
            steerVehicleNP = OSSteerManager.get_global_ptr().get_steer_vehicle(i)
            steerVehicle[i] = steerVehicleNP.node()
            # restore animations
            tmpAnims = AnimControlCollection()
            auto_bind(steerVehicle[i], tmpAnims)
            vehicleAnimCtls[i] = [None, None];
            for j in range(tmpAnims.get_num_anims()):
                vehicleAnimCtls[i][j] = tmpAnims.get_anim(j)

    # show the added vehicles
    print("Vehicles added to plug-in:")
    for vehicle in steerPlugIn:
        print("\t- " + str(vehicle))

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

    # # set events' callbacks
    # toggle debug draw
    toggleDebugFlag = False
    app.accept("d", toggleDebugDraw, [steerPlugIn])

    # handle obstacle addition
    app.accept("o", handleObstacles, [True])
    # handle obstacle removal
    app.accept("shift-o", handleObstacles, [False]);

    # increase/decrease vehicle's max speed
    app.accept("s", changeVehicleMaxSpeed, ["s", steerVehicle[0]])
    app.accept("shift-s", changeVehicleMaxSpeed, ["shift-s", steerVehicle[0]])
    # increase/decrease vehicle's max force
    app.accept("f", changeVehicleMaxForce, ["f", steerVehicle[0]])
    app.accept("shift-f", changeVehicleMaxForce, ["shift-f", steerVehicle[0]])
    
    # handle OSSteerVehicle(s)' events
#     app.accept("move-event", handleVehicleEvent) XXX
    
    # write to bam file on exit
    app.win.set_close_request_event("close_request_event")
    app.accept("close_request_event", writeToBamFileAndExit, [bamFileName])

    # 'low speed turn' specific: toggle steering speed
    app.accept("t", toggleSteeringSpeed)
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-128.0, 120.0, -40.0);
    trackball.set_hpr(0.0, 20.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

