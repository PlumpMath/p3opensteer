'''
Created on Aug 19, 2016

@author: consultit
'''

from panda3d.core import TextNode, ClockObject, AnimControlCollection, \
        auto_bind, LPoint3f, LVecBase3f, PartGroup, LVector3f
from p3opensteer import OSSteerManager, ValueList_string, ValueList_LPoint3f, \
        ValueList_float
#
from common import startFramework, toggleDebugFlag, toggleDebugDraw, mask, \
        loadTerrain, printCreationParameters, handleVehicleEvent, \
        changeVehicleMaxForce, changeVehicleMaxSpeed, getVehicleModelAnims, \
        animRateFactor, writeToBamFileAndExit, readFromBamFile, bamFileName, \
        getCollisionEntryFromCamera, obstacleFile, HandleObstacleData, \
        handleObstacles, HandleVehicleData, handleVehicles, vehicleFile, \
        vehicleAnimFiles, Driver
import sys, random
        
# # specific data/functions declarations/definitions
sceneNP = None
vehicleAnimCtls = []
steerPlugIn = None
steerVehicles = []
playerNP = None
playerDriver = None
playerHeightRayCast = LVector3f()
forwardMove = 1
forwardMoveStop = -1
leftMove = 2
leftMoveStop = -2
backwardMove = 3
backwardMoveStop = -3
rightMove = 4
rightMoveStop = -4
#
def setParametersBeforeCreation():
    """set parameters as strings before plug-ins/vehicles creation"""
    
    steerMgr = OSSteerManager.get_global_ptr()
    valueList = ValueList_string()
    # set vehicle's mass, speed
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "external_update", 
            "false")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "mass",
            "2.0")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "speed",
            "0.01")

    # set vehicle throwing events
    valueList.clear()
    valueList.add_value("avoid_obstacle@avoid_obstacle@1.0:"
            "avoid_close_neighbor@avoid_close_neighbor@1.0:"
            "avoid_neighbor@avoid_neighbor@1.0")
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
    # make playerNP kinematic (ie stand on floor)
    if playerNP.node().get_speed() > 0.0:
        # get steer manager
        steerMgr = OSSteerManager.get_global_ptr()
        # correct panda's Z: set the collision ray origin wrt collision root
        pOrig = steerMgr.get_collision_root().get_relative_point(
                steerMgr.get_reference_node_path(), playerNP.get_pos()) + playerHeightRayCast * 2.0
        # get the collision height wrt the reference node path
        gotCollisionZ = steerMgr.get_collision_height(pOrig, steerMgr.get_reference_node_path())
        if gotCollisionZ.get_first():
            #updatedPos.z needs correction
            playerNP.set_z(gotCollisionZ.get_second())
    #
    return task.cont

def getPlayerModelAnims(name, scale, vehicleFileIdx, steerPlugIn, 
                           steerVehicles, vehicleAnimCtls, pos):
    """get the player, model and animations"""
    
    global app, vehicleAnimFiles
    # get some models, with animations, to attach to vehicles
    # get the model
    vehicleNP = app.loader.load_model(vehicleFile[vehicleFileIdx])
    # associate an anim with a given anim control
    tmpAnims = AnimControlCollection()
    vehicleAnimNP = [None, None]
    vehicleAnimCtls.append([None, None])
    if(len(vehicleAnimFiles[vehicleFileIdx][0]) != 0) and \
            (len(vehicleAnimFiles[vehicleFileIdx][1]) != 0):
        # first anim -> modelAnimCtls[i][0]
        vehicleAnimNP[0] = app.loader.load_model(vehicleAnimFiles[vehicleFileIdx][0])
        vehicleAnimNP[0].reparent_to(vehicleNP)
        auto_bind(vehicleNP.node(), tmpAnims, 
                  PartGroup.HMF_ok_part_extra |
                  PartGroup.HMF_ok_anim_extra |
                  PartGroup.HMF_ok_wrong_root_name)
        vehicleAnimCtls[-1][0] = tmpAnims.get_anim(0)
        tmpAnims.clear_anims()
        vehicleAnimNP[0].detach_node()
        # second anim -> modelAnimCtls[i][1]
        vehicleAnimNP[1] = app.loader.load_model(vehicleAnimFiles[vehicleFileIdx][1])
        vehicleAnimNP[1].reparent_to(vehicleNP)
        auto_bind(vehicleNP.node(), tmpAnims, 
                  PartGroup.HMF_ok_part_extra |
                  PartGroup.HMF_ok_anim_extra |
                  PartGroup.HMF_ok_wrong_root_name)
        vehicleAnimCtls[-1][1] = tmpAnims.get_anim(0)
        tmpAnims.clear_anims()
        vehicleAnimNP[1].detach_node()
        # reparent all node paths
        vehicleAnimNP[0].reparent_to(vehicleNP)
        vehicleAnimNP[1].reparent_to(vehicleNP)
    #
    steerMgr = OSSteerManager.get_global_ptr()
    # create the steer vehicle (attached to the reference node)
    # note: vehicle's move type is ignored
    steerVehicleNP = steerMgr.create_steer_vehicle("PlayerVehicle")
    steerVehicles.append(steerVehicleNP.node())
    # set the name
    steerVehicleNP.set_name(name)
    # set scale
    steerVehicleNP.set_scale(scale)
    # set the position
    steerVehicleNP.set_pos(pos)
    # attach some geometry (a model) to steer vehicle
    vehicleNP.reparent_to(steerVehicleNP)
    # add the steer vehicle to the plug-in
    steerPlugIn.add_steer_vehicle(steerVehicleNP)
    # return the steerVehicleNP
    return steerVehicleNP

def movePlayer(data):
    """player's movement callback"""
    
    global playerDriver, forwardMove, leftMove, backwardMove, rightMove
    if not playerDriver:
        return

    action = data
    if action > 0:
        #start movement
        enable = True
    else:
        action = -action
        #stop movement
        enable = False
    #
    if action == forwardMove:
        playerDriver.enable_forward(enable)
    elif action == leftMove:
        playerDriver.enable_head_left(enable)
    elif action == backwardMove:
        playerDriver.enable_backward(enable)
    elif action == rightMove:
        playerDriver.enable_head_right(enable)
        
if __name__ == '__main__':

    msg = "'pedestrian external'"
    app = startFramework(msg)
      
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            msg + "\n\n"      
            "- press \"d\" to toggle debug drawing\n"
            "- press \"up\"/\"left\"/\"down\"/\"right\" arrows to move the player\n"
            "- press \"a\"/\"k\" to add 'opensteer'/'kinematic' vehicle\n"
            "- press \"s\"/\"shift-s\" to increase/decrease last inserted vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease last inserted vehicle's max force\n"
            "- press \"t\" to toggle last inserted vehicle's wander behavior\n"
            "- press \"o\"/\"shift-o\" to add/remove obstacle\n")
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-1.25, 0.0, 0.8)
    textNodePath.set_scale(0.035)
    
    # create a steer manager set root and mask to manage 'kinematic' vehicles
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
        sceneNP = loadTerrain("SceneNP")
        # and reparent to the reference node
        sceneNP.reparent_to(steerMgr.get_reference_node_path())
        
        # set sceneNP's collide mask
        sceneNP.set_collide_mask(mask)

        # set plug-in type and create it (attached to the reference node)
        steerMgr.set_parameter_value(OSSteerManager.STEERPLUGIN, "plugin_type",
                "pedestrian")
        plugInNP = steerMgr.create_steer_plug_in()
        steerPlugIn = plugInNP.node()
        
        # set player's creation parameters as string: type and externally updated
        steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE,
                "vehicle_type", "pedestrian")
        steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE,
                "external_update", "true")
        # add the player and set a reference to it
        playerNP = getPlayerModelAnims("PlayerNP", 0.8, 0, steerPlugIn,
                steerVehicles, vehicleAnimCtls,
                LPoint3f(141.597, 73.496, 2.14218))
        # highlight the player
        playerNP.set_color(1.0, 1.0, 0.0, 0)
        
        # set remaining creation parameters as strings before 
        # the other vehicles' creation
        print("\n" + "Current creation parameters:")
        setParametersBeforeCreation()
          
        # set the pathway
        pointList = ValueList_LPoint3f()
        pointList.add_value(LPoint3f(79.474, 51.7236, 2.0207))
        pointList.add_value(LPoint3f(108.071, 51.1972, 2.7246))
        pointList.add_value(LPoint3f(129.699, 30.1742, 0.720501))
        pointList.add_value(LPoint3f(141.597, 73.496, 2.14218))
        pointList.add_value(LPoint3f(105.917, 107.032, 3.06428))
        pointList.add_value(LPoint3f(61.2637, 109.622, 3.03588))
        #  use single radius pathway
        radiusList = ValueList_float()
        radiusList.add_value(4)
        steerPlugIn.set_pathway(pointList, radiusList, True, True)
    else:
        # valid bamFile
        # restore plug-in: through steer manager
        steerPlugIn = OSSteerManager.get_global_ptr().get_steer_plug_in(0)
        # restore sceneNP: through panda3d
        sceneNP = OSSteerManager.get_global_ptr().get_reference_node_path().find("**/SceneNP")
        # reparent the reference node to render
        OSSteerManager.get_global_ptr().get_reference_node_path().reparent_to(app.render)

        # restore the player's reference
        playerNP = OSSteerManager.get_global_ptr().get_reference_node_path().find(
                "**/PlayerNP")
    
        # restore all steer vehicles (including the player)
        NUMVEHICLES = OSSteerManager.get_global_ptr().get_num_steer_vehicles()
        tmpList = [None for i in range(NUMVEHICLES)]
        steerVehicles.extend(tmpList)
        vehicleAnimCtls.extend(tmpList)
        for i in range(NUMVEHICLES):
            # restore the steer vehicle: through steer manager
            steerVehicles[i] = OSSteerManager.get_global_ptr().get_steer_vehicle(i)
            # restore animations
            tmpAnims = AnimControlCollection()
            auto_bind(steerVehicles[i], tmpAnims)
            vehicleAnimCtls[i] = [None, None]
            for j in range(tmpAnims.get_num_anims()):
                vehicleAnimCtls[i][j] = tmpAnims.get_anim(j)

        # set creation parameters as strings before other plug-ins/vehicles creation
        print("\n" + "Current creation parameters:")
        steerMgr.set_parameter_value(OSSteerManager.STEERPLUGIN, "plugin_type",
                "pedestrian")
        steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE,
                "vehicle_type", "pedestrian")
        setParametersBeforeCreation()

    # # first option: start the default update task for all plug-ins
#     steerMgr.start_default_update()

    # # second option: start the custom update task for all plug-ins
    app.taskMgr.add(updatePlugIn, "updatePlugIn", extraArgs=[steerPlugIn], 
                    appendTask=True)

    # DEBUG DRAWING: make the debug reference node paths sibling of the reference node
    steerMgr.get_reference_node_path_debug().reparent_to(app.render)
    steerMgr.get_reference_node_path_debug_2d().reparent_to(app.aspect2d)
    # enable debug drawing
    steerPlugIn.enable_debug_drawing(app.camera)

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
    app.accept("shift-o", handleObstacles, [obstacleRemoval])

    # increase/decrease last inserted vehicle's max speed
    app.accept("s", changeVehicleMaxSpeed, ["s", steerVehicles])
    app.accept("shift-s", changeVehicleMaxSpeed, ["shift-s", steerVehicles])
    # increase/decrease last inserted vehicle's max force
    app.accept("f", changeVehicleMaxForce, ["f", steerVehicles])
    app.accept("shift-f", changeVehicleMaxForce, ["shift-f", steerVehicles])
    
    # handle OSSteerVehicle(s)' events
    app.accept("avoid_obstacle", handleVehicleEvent, ["avoid_obstacle"])
    app.accept("avoid_close_neighbor", handleVehicleEvent, ["avoid_close_neighbor"])
    app.accept("avoid_neighbor", handleVehicleEvent, ["avoid_neighbor"])
    
    # write to bam file on exit
    app.win.set_close_request_event("close_request_event")
    app.accept("close_request_event", writeToBamFileAndExit, [bamFileName])

    # 'pedestrian' specific: toggle wander behavior
    app.accept("t", toggleWanderBehavior)
    
    # get player dims for kinematic ray cast
    modelDims = LVecBase3f() 
    modelDeltaCenter = LVector3f()
    steerMgr.get_bounding_dimensions(playerNP, modelDims, modelDeltaCenter)
    playerHeightRayCast = LVector3f(0.0, 0.0, modelDims.get_z())

    # player will be driven by arrows keys
    playerDriver = Driver(app, playerNP, 10)
    playerDriver.set_max_angular_speed(100.0)
    playerDriver.set_angular_accel(50.0)
    playerDriver.set_max_linear_speed(LVector3f(8.0, 8.0, 8.0))
    playerDriver.set_linear_accel(LVecBase3f(1.0, 1.0, 1.0))
    playerDriver.set_linear_friction(1.5)
    playerDriver.enable()
    app.accept("arrow_up", movePlayer, [forwardMove]) 
    app.accept("arrow_up-up", movePlayer, [forwardMoveStop]) 
    app.accept("arrow_left", movePlayer, [leftMove]) 
    app.accept("arrow_left-up", movePlayer, [leftMoveStop]) 
    app.accept("arrow_down", movePlayer, [backwardMove]) 
    app.accept("arrow_down-up", movePlayer, [backwardMoveStop]) 
    app.accept("arrow_right", movePlayer, [rightMove]) 
    app.accept("arrow_right-up", movePlayer, [rightMoveStop]) 
   
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-128.0, 120.0, -40.0)
    trackball.set_hpr(0.0, 20.0, 0.0)
   
    # app.run(), equals to do the main loop in C++
    app.run()

