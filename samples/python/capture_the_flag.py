'''
Created on Jun 26, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager, ValueList_string, ValueList_LPoint3f, \
        ValueList_float, OSSteerVehicle
from panda3d.core import TextNode, ClockObject, AnimControlCollection, \
        auto_bind, LPoint3f, LVecBase3f, NodePath
#
from common import startFramework, toggleDebugFlag, toggleDebugDraw, mask, \
        loadTerrain, printCreationParameters, handleVehicleEvent, \
        changeVehicleMaxForce, changeVehicleMaxSpeed, getVehicleModelAnims, \
        animRateFactor, writeToBamFileAndExit, readFromBamFile, bamFileName, \
        getCollisionEntryFromCamera, obstacleFile, HandleObstacleData, \
        handleObstacles, HandleVehicleData, handleVehicles, loadTerrainLowPoly
import sys, random
        
# # specific data/functions declarations/definitions
sceneNP = None
vehicleAnimCtls = []
steerPlugIn = None
steerVehicles = []
#
flagNP = NodePath()
flagAnims = AnimControlCollection()

def setParametersBeforeCreation():
    """set parameters as strings before plug-ins/vehicles creation"""
    
    steerMgr = OSSteerManager.get_global_ptr()
    valueList = ValueList_string()
    # set plug-in type
    steerMgr.set_parameter_value(OSSteerManager.STEERPLUGIN, "plugin_type",
            "capture_the_flag")

    # set vehicle throwing events
    valueList.clear()
    valueList.add_value("avoid_obstacle@avoid_obstacle@1.0")
    steerMgr.set_parameter_values(OSSteerManager.STEERVEHICLE,
            "thrown_events", valueList)
    #
    printCreationParameters()

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

def createCtfVehicle(data, vehicleType):
    """creates a generic vehicle for ctf plug-in"""

    global steerVehicles
    maxForce = 1.0
    maxSpeed = 5.0
    # set vehicle's type == ctf_enemy
    typeStr = "ctf_enemy"
    speed = 1.0
    if vehicleType == OSSteerVehicle.CTF_SEEKER:
        typeStr = "ctf_seeker"
        speed = 0.0
    OSSteerManager.get_global_ptr().set_parameter_value(
                    OSSteerManager.STEERVEHICLE, "vehicle_type", typeStr)

    oldPlayerNum = len(steerVehicles)
    # handle vehicle
    handleVehicles(data)
    if len(steerVehicles) > oldPlayerNum:
        # set vehicle's parameters
        steerVehicles[-1].set_max_force(maxForce)
        steerVehicles[-1].set_max_speed(maxSpeed)
        steerVehicles[-1].set_speed(speed)
        steerVehicles[-1].enable_up_axis_fixed(True)
        return True
    return False

def addSeeker(data=None):
    """adds a seeker""" 
    
    global steerVehicles, steerPlugIn
    if data == None:
        return

    createCtfVehicle(data, OSSteerVehicle.CTF_SEEKER)

def addEnemy(data=None):
    """adds an enemy"""
    
    global steerVehicles, steerPlugIn
    if data == None:
        return

    createCtfVehicle(data, OSSteerVehicle.CTF_ENEMY)

def setHomeBaseCenter(flag):
    """set home base center"""

    global steerPlugIn, app
    if steerPlugIn == None:
        return
    
    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        # get the hit object
        hitObject = entry0.get_into_node_path()
        print("hit " + str(hitObject) + " object")
        
        # set home base center's position
        center = entry0.get_surface_point(app.render)
        steerPlugIn.set_home_base_center(center)
        flag.set_pos(center)
        print("set home base center at: " + str(center))
        
def getFlag(name):
    """load the flag"""

    global app
    
    flag = app.loader.load_model("flag_oga.egg")
    flag.set_two_sided(True)
    flag.set_scale(1.5)
    flag.set_name(name)
    flag.reparent_to(OSSteerManager.get_global_ptr().get_reference_node_path())
    flagWave = app.loader.load_model("flag_oga-wave.egg")
    flagWave.reparent_to(flag)
    auto_bind(flag.node(), flagAnims)
    flagAnims.get_anim(0).loop(True)
    return flag
        
if __name__ == '__main__':

    msg = "'capture_the_flag'"
    app = startFramework(msg)
      
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            msg + "\n\n"      
            "- press \"d\" to toggle debug drawing\n"
            "- press \"a\"/\"e\" to add a seeker/enemy\n"
            "- press \"h\" to set home base center\n"
            "- press \"s\"/\"shift-s\" to increase/decrease last inserted vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease last inserted vehicle's max force\n"
            "- press \"o\"/\"shift-o\" to add/remove obstacle\n")
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-1.25, 0.0, 0.8)
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
        sceneNP = loadTerrainLowPoly("SceneNP", 128, 64)
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

        # set the capture the flag common settings
        steerPlugIn.set_home_base_radius(6.0)
        steerPlugIn.set_braking_rate(0.75)
        steerPlugIn.set_avoidance_predict_time_min(0.9)
        steerPlugIn.set_avoidance_predict_time_max(2.0)
        
        # load flag model naming it with "FlagNP" to ease restoring from bam
        # file
        flagNP = getFlag("FlagNP")
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

        # restore flag and its animation
        flagNP = OSSteerManager.get_global_ptr().get_reference_node_path().find(
                        "**/FlagNP")
        auto_bind(flagNP.node(), flagAnims)
        flagAnims.get_anim(0).loop(True)
        
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

    # # set events' callbacks
    # toggle debug draw
    toggleDebugFlag = False
    app.accept("d", toggleDebugDraw, [steerPlugIn])

    # handle addition steer vehicles, models and animations 
    seekerData = HandleVehicleData(1.2, 0, "kinematic", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("a", addSeeker, [seekerData])
    enemyData = HandleVehicleData(1.2, 1, "kinematic", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("e", addEnemy, [enemyData])
    
    # set home base center
    app.accept("h", setHomeBaseCenter, [flagNP])

    # handle obstacle addition
    obstacleAddition = HandleObstacleData(True, sceneNP, steerPlugIn,
                        LVecBase3f(0.05, 0.05, 0.08))
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
    
    # write to bam file on exit
    app.win.set_close_request_event("close_request_event")
    app.accept("close_request_event", writeToBamFileAndExit, [bamFileName])
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(0.0, 320.0, -10.0);
    trackball.set_hpr(0.0, 20.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

