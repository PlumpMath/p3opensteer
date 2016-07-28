'''
Created on Jun 26, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager, ValueList_string, ValueList_LPoint3f, \
        ValueList_float, OSSteerPlugIn, OSSteerVehicle
from panda3d.core import TextNode, ClockObject, AnimControlCollection, \
        auto_bind, LPoint3f, LVecBase3f
#
from common import startFramework, toggleDebugFlag, toggleDebugDraw, mask, \
        loadTerrain, printCreationParameters, handleVehicleEvent, \
        changeVehicleMaxForce, changeVehicleMaxSpeed, getVehicleModelAnims, \
        rateFactor, writeToBamFileAndExit, readFromBamFile, bamFileName, \
        getCollisionEntryFromCamera, obstacleFile, HandleObstacleData, \
        handleObstacles, HandleVehicleData, handleVehicles, loadPlane
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
            "soccer")

    # set vehicle throwing events
    valueList.clear()
    valueList.add_value("avoid_neighbor@avoid_neighbor@")
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
        if (vehicleAnimCtls[i][0] != None) and \
              (vehicleAnimCtls[i][1] != None):
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
                vehicleAnimCtls[i][animOnIdx].set_play_rate(currentVelSize / rateFactor[animOnIdx])
                if not vehicleAnimCtls[i][animOnIdx].is_playing():
                    vehicleAnimCtls[i][animOnIdx].loop(True)
            else:
                # stop any animation
                vehicleAnimCtls[i][0].stop()
                vehicleAnimCtls[i][1].stop()
    #
    return task.cont

def createSoccerVehicle(data, vehicleType):
    """creates a generic vehicle for soccer plug-in"""

    global steerVehicles
    # set vehicle's type == player
    typeStr = "player"
    maxForce = 3000.7
    maxSpeed = 10.0
    speed = 0.0
    if vehicleType == OSSteerVehicle.BALL:
        typeStr = "ball"
        maxForce = 9.0
        maxSpeed = 9.0
        speed = 1.0
    OSSteerManager.get_global_ptr().set_parameter_value(
                    OSSteerManager.STEERVEHICLE, "vehicle_type", typeStr)

    oldPlayerNum = len(steerVehicles)
    # handle vehicle
    handleVehicles(data)
    if len(steerVehicles) > oldPlayerNum:
        # set player's parameters
        steerVehicles[-1].set_max_force(maxForce)
        steerVehicles[-1].set_max_speed(maxSpeed)
        steerVehicles[-1].set_speed(speed)
        steerVehicles[-1].enable_up_axis_fixed(True)
        return True
    return False

def addPlayerA(data=None):
    """adds last created player to teamA""" 
    
    global steerVehicles, steerPlugIn
    if data == None:
        return

    if createSoccerVehicle(data, OSSteerVehicle.PLAYER):
        # add to teamA
        steerPlugIn.add_player_to_team(steerVehicles[-1], OSSteerPlugIn.TEAM_A)

def addPlayerB(data=None):
    """adds last created player to teamB""" 
    
    global steerVehicles, steerPlugIn
    if data == None:
        return

    if createSoccerVehicle(data, OSSteerVehicle.PLAYER):
        # add to teamB
        steerPlugIn.add_player_to_team(steerVehicles[-1], OSSteerPlugIn.TEAM_B)

def addBall(data = None):
    """adds a ball""" 
    
    global steerVehicles, steerPlugIn
    if data == None:
        return

    createSoccerVehicle(data, OSSteerVehicle.BALL)
        
if __name__ == '__main__':

    msg = "'soccer'"
    app = startFramework(msg)
      
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            msg + "\n\n"      
            "- press \"d\" to toggle debug drawing\n"
            "- press \"a\"/\"b\" to add a player to teamA/teamB\n"
            "- press \"p\" to add a ball\n")
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(-1.25, 0.0, 0.90)
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
        sceneNP = loadPlane("SceneNP", 300, 300)
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
    
        # set playing field
        steerPlugIn.set_playing_field(LPoint3f(20.0, 50.0, 0.0),
                LPoint3f(100.0, 100.0, 0.0), 0.7)
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

    # # set events' callbacks
    # toggle debug draw
    toggleDebugFlag = False
    app.accept("d", toggleDebugDraw, [steerPlugIn])

    # handle addition steer vehicles, models and animations 
    playerAData = HandleVehicleData(0.7, 0, "kinematic", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("a", addPlayerA, [playerAData])
    playerBData = HandleVehicleData(0.7, 1, "kinematic", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("b", addPlayerB, [playerBData])
    ballData = HandleVehicleData(0.7, 3, "kinematic", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("p", addBall, [ballData])
    
    # handle OSSteerVehicle(s)' events
    app.accept("avoid_neighbor", handleVehicleEvent, ["avoid_neighbor"])
    
    # write to bam file on exit
    app.win.set_close_request_event("close_request_event")
    app.accept("close_request_event", writeToBamFileAndExit, [bamFileName])
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-55.0, 80.0, -22.0)
    trackball.set_hpr(10.0, 10.0, 0.0)
   
    # app.run(), equals to do the main loop in C++
    app.run()

