'''
Created on Jun 26, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager, ValueList_string, ValueList_LPoint3f, \
        ValueList_float, OSSteerPlugIn
from panda3d.core import TextNode, ClockObject, AnimControlCollection, \
        auto_bind, LPoint3f, LVecBase3f, TextureStage, TexGenAttrib
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
rttTexStage = None
#
def setParametersBeforeCreation():
    """set parameters as strings before plug-ins/vehicles creation"""
    
    steerMgr = OSSteerManager.get_global_ptr()
    valueList = ValueList_string()
    # set plug-in type
    steerMgr.set_parameter_value(OSSteerManager.STEERPLUGIN, "plugin_type",
            "map_drive")

    # set vehicle's type, mass, speed
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "vehicle_type",
            "map_driver")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "max_speed",
            "20.0")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "max_force",
            "8.0")
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "up_axis_fixed",
            "true")

    # set vehicle throwing events
    valueList.clear()
    valueList.add_value(
            "avoid_obstacle@avoid_obstacle@1.0:path_following@path_following@1.0")
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

def debugDrawToTexture():
    "debug draw to texture"

    global steerPlugIn, sceneNP, app
    steerPlugIn.debug_drawing_to_texture(sceneNP, app.win)

def onTextureReady(data, texture):
    "debug drawing texture is ready"
    
    global sceneNP
    rttTexStage = data
    # set up texture where to render
    sceneNP.clear_texture(rttTexStage)
    rttTexStage.set_mode(TextureStage.M_modulate)
    # take into account sceneNP dimensions
    sceneNP.set_tex_offset(rttTexStage, 0.5, 0.5)
    sceneNP.set_tex_scale(rttTexStage, 1.0 / 128.0, 1.0 / 128.0)
    sceneNP.set_tex_gen(rttTexStage, TexGenAttrib.M_world_position)
    sceneNP.set_texture(rttTexStage, texture, 10)

def togglePredictionType():
    """toggle prediction type"""
    
    global steerPlugIn
    predictionType = steerPlugIn.get_map_prediction_type()
    if predictionType == OSSteerPlugIn.CURVED_PREDICTION:
        steerPlugIn.set_map_prediction_type(OSSteerPlugIn.LINEAR_PREDICTION)
        print ("prediction type: linear")
    else:
        steerPlugIn.set_map_prediction_type(OSSteerPlugIn.CURVED_PREDICTION)
        print ("prediction type: curved")
       
if __name__ == '__main__':

    msg = "'map drive'"
    app = startFramework(msg)
      
    # # here is room for your own code
    # print some help to screen
    text = TextNode("Help")
    text.set_text(
            msg + "\n\n"      
            "- press \"d\" to toggle debug drawing\n"
            "- press \"o\"/\"shift-o\" to add/remove obstacle\n"
            "- press \"t\" to (re)draw the map of the path\n"
            "- press \"a\" to add vehicle\n"
            "- press \"p\" to toggle map prediction type\n")
    textNodePath = app.aspect2d.attach_new_node(text)
    textNodePath.set_pos(0.25, 0.0, 0.8)
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
        
        # set the texture stage used for debug draw texture
        rttTexStage = TextureStage("rttTexStage")

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
        pointList.add_value(LPoint3f(-41.80, 34.46, -0.17))
        radiusList.add_value(7.0)
        pointList.add_value(LPoint3f(-2.21, 49.15, -0.36))
        radiusList.add_value(8.0)
        pointList.add_value(LPoint3f(10.78, 16.65, 0.14))
        radiusList.add_value(9.0)
        pointList.add_value(LPoint3f(40.44, 17.58, -0.22))
        radiusList.add_value(9.0)
        pointList.add_value(LPoint3f(49.04, -22.15, -0.60))
        radiusList.add_value(8.0)
        pointList.add_value(LPoint3f(13.99, -52.70, 0.39))
        radiusList.add_value(8.0)
        pointList.add_value(LPoint3f(-3.46, -31.90, 0.71))
        radiusList.add_value(7.0)
        pointList.add_value(LPoint3f(-30.0, -39.97, -0.35))
        radiusList.add_value(6.0)
        pointList.add_value(LPoint3f(-47.12, -17.31, -0.43))
        radiusList.add_value(6.0)
        pointList.add_value(LPoint3f(-51.31, 9.08, -0.25))
        radiusList.add_value(7.0)
        steerPlugIn.set_pathway(pointList, radiusList, False, True)
        # make the map
        steerPlugIn.make_map(200)
    else:
        # valid bamFile
        # restore plug-in: through steer manager
        steerPlugInNP = OSSteerManager.get_global_ptr().get_steer_plug_in(0)
        steerPlugIn = steerPlugInNP.node()
        # restore sceneNP: through panda3d
        sceneNP = OSSteerManager.get_global_ptr().get_reference_node_path().find("**/SceneNP")
        # reparent the reference node to render
        OSSteerManager.get_global_ptr().get_reference_node_path().reparent_to(app.render)

        # restore the texture stage used for debug draw texture
        rttTexStage = sceneNP.find_all_texture_stages().find_texture_stage(
                "rttTexStage")
    
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
            vehicleAnimCtls[i] = [None, None]
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
    steerMgr.get_reference_node_path_debug_2d().reparent_to(app.aspect2d)
    # enable debug drawing
    steerPlugIn.enable_debug_drawing(app.camera)
    # print debug draw texture
    app.accept("t", debugDrawToTexture)
    app.accept("debug_drawing_texture_ready", onTextureReady, [rttTexStage])

    # # set events' callbacks
    # toggle debug draw
    toggleDebugFlag = False
    app.accept("d", toggleDebugDraw, [steerPlugIn])

    # handle addition steer vehicles, models and animations 
    vehicleData = HandleVehicleData(0.4, 4, "kinematic", sceneNP, 
                        steerPlugIn, steerVehicles, vehicleAnimCtls)
    app.accept("a", handleVehicles, [vehicleData])

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
    app.accept("path_following", handleVehicleEvent, ["path_following"])
    
    # write to bam file on exit
    app.win.set_close_request_event("close_request_event")
    app.accept("close_request_event", writeToBamFileAndExit, [bamFileName])

    # map drive specific: toggle prediction type
    app.accept("p", togglePredictionType)
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(0.0, 160.0, -5.0)
    trackball.set_hpr(0.0, 20.0, 0.0)
   
    # app.run(), equals to do the main loop in C++
    app.run()

