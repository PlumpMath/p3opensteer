'''
Created on Jun 20, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager
from panda3d.core import load_prc_file_data, GeoMipTerrain, PNMImage, \
                Filename, TextureStage, TexturePool, BitMask32, CardMaker, \
                NodePath, WindowProperties, AnimControlCollection, auto_bind, \
                LVecBase3f, LVector3f, LPoint3f
from direct.showbase.ShowBase import ShowBase
import sys, random

# global data
dataDir = "../data"
app = None
mask = BitMask32(0x10)
toggleDebugFlag = False
terrain = None
terrainRootNetPos = None
DEFAULT_MAXVALUE = 1.0
maxSpeedValue = DEFAULT_MAXVALUE
maxForceValue = DEFAULT_MAXVALUE / 10.0
# models and animations
vehicleFile = ["eve.egg", "ralph.egg"]
vehicleAnimFiles = [["eve-walk.egg", "eve-run.egg"],
                  ["ralph-walk.egg", "ralph-run.egg"]]
rateFactor = [1.20, 3.40]
# obstacle model
obstacleFile = "plants2.egg"
# bame file
bamFileName = "plug_in.boo"

# # functions' declarations and definitions

def startFramework(msg):
    """start base framework"""

    global app   
    # Load your application's configuration
    load_prc_file_data("", "model-path " + dataDir)
    load_prc_file_data("", "win-size 1024 768")
    load_prc_file_data("", "show-frame-rate-meter #t")
    load_prc_file_data("", "sync-video #t")
#     load_prc_file_data("", "want-directtools #t")
#     load_prc_file_data("", "want-tk #t")
        
    # Setup your application
    app = ShowBase()
    props = WindowProperties()
    props.setTitle("p3opensteer: " + msg)
    app.win.requestProperties(props)
 
    #common callbacks     
    #
    return app

def loadPlane(name):
    """load plane stuff"""
    
    cm = CardMaker("plane")
    cm.set_frame(-15, 15, -15, 15)
    plane = NodePath(cm.generate())
    plane.set_p(-90.0)
    plane.set_z(0.0)
    plane.set_color(0.15, 0.35, 0.35)
    plane.set_collide_mask(mask)
    plane.set_name(name)
    return plane

def loadTerrain(name):
    """load terrain stuff"""

    global app, terrain, terrainRootNetPos
        
    steerMgr = OSSteerManager.get_global_ptr()

    terrain = GeoMipTerrain("terrain")
    heightField = PNMImage(Filename(dataDir + "/heightfield.png"))
    terrain.set_heightfield(heightField)
    # sizing
    widthScale, heightScale = (0.5, 10.0)
    environmentWidthX = (heightField.get_x_size() - 1) * widthScale
    environmentWidthY = (heightField.get_y_size() - 1) * widthScale
    environmentWidth = (environmentWidthX + environmentWidthY) / 2.0
    terrain.get_root().set_sx(widthScale)
    terrain.get_root().set_sy(widthScale)
    terrain.get_root().set_sz(heightScale)
    # set other terrain's properties
    blockSize, minimumLevel = (64, 0)
    nearPercent, farPercent = (0.1, 0.7)
    terrainLODmin = min(minimumLevel, terrain.get_max_level())
    flattenMode = GeoMipTerrain.AFM_off
    terrain.set_block_size(blockSize)
    terrain.set_near(nearPercent * environmentWidth)
    terrain.set_far(farPercent * environmentWidth)
    terrain.set_min_level(terrainLODmin)
    terrain.set_auto_flatten(flattenMode)
    # terrain texturing
    textureStage0 = TextureStage("TextureStage0")
    textureImage = TexturePool.load_texture(Filename("terrain.png"))
    terrain.get_root().set_tex_scale(textureStage0, 1.0, 1.0)
    terrain.get_root().set_texture(textureStage0, textureImage, 1)
    # reparent this Terrain node path to the object node path
    terrain.get_root().reparent_to(steerMgr.get_reference_node_path())
    terrain.get_root().set_collide_mask(mask)
    terrain.get_root().set_name(name)
    # brute force generation
    bruteForce = True
    terrain.set_bruteforce(bruteForce)
    # Generate the terrain
    terrain.generate()
    # check if terrain needs update or not
    if not bruteForce:
        # save the net pos of terrain root
        terrainRootNetPos = terrain.get_root().get_net_transform().get_pos()
        # Add a task to keep updating the terrain
        app.taskMgr.add(terrainUpdate, "terrainUpdate", appendTask=True)
    #
    return terrain.get_root()

def terrainUpdate(task):
    """terrain update"""

    global app, terrain, terrainRootNetPos
    # set focal point
    # see https://www.panda3d.org/forums/viewtopic.php?t=5384
    focalPointNetPos = app.camera.get_net_transform().get_pos()
    terrain.set_focal_point(focalPointNetPos - terrainRootNetPos)
    # update every frame
    terrain.update()
    #
    return task.cont

def getCollisionEntryFromCamera():
    """throws a ray and returns the first collision entry or nullptr"""    
    
    global app
    # get steer manager
    steerMgr = OSSteerManager.get_global_ptr()
    # get the mouse watcher
    mwatcher = app.mouseWatcherNode
    if mwatcher.has_mouse():
        # Get to and from pos in camera coordinates
        pMouse = mwatcher.get_mouse()
        #
        pFrom, pTo = (LPoint3f(), LPoint3f())
        if app.camLens.extrude(pMouse, pFrom, pTo):
            # Transform to global coordinates
            pFrom = app.render.get_relative_point(app.cam, pFrom)
            pTo = app.render.get_relative_point(app.cam, pTo)
            direction = (pTo - pFrom).normalized()
            steerMgr.get_collision_ray().set_origin(pFrom)
            steerMgr.get_collision_ray().set_direction(direction)
            steerMgr.get_collision_traverser().traverse(app.render)
            # check collisions
            if steerMgr.get_collision_handler().get_num_entries() > 0:
                # Get the closest entry
                steerMgr.get_collision_handler().sort_entries()
                return steerMgr.get_collision_handler().get_entry(0)
    return None

def printCreationParameters():
    """print creation parameters"""
    
    steerMgr = OSSteerManager.get_global_ptr()
    #
    valueList = steerMgr.get_parameter_name_list(OSSteerManager.STEERPLUGIN)
    print("\n" + "OSSteerPlugIn creation parameters:")
    for name in valueList:
        print ("\t" + name + " = " + 
               steerMgr.get_parameter_value(OSSteerManager.STEERPLUGIN, name))
    #
    valueList = steerMgr.get_parameter_name_list(OSSteerManager.STEERVEHICLE)
    print("\n" + "OSSteerVehicle creation parameters:")
    for name in valueList:
        print ("\t" + name + " = " + 
               steerMgr.get_parameter_value(OSSteerManager.STEERVEHICLE, name))

def handleVehicleEvent(name, vehicle):
    """handle vehicle's events"""
    
    vehicleNP = NodePath.any_path(vehicle)
    print ("got " + name + " event from '"+ vehicleNP.get_name() + "' at " + str(vehicleNP.get_pos()))

def toggleDebugDraw(plugIn):
    """toggle debug draw"""
    
    global toggleDebugFlag
    if not plugIn:
        return

    toggleDebugFlag = not toggleDebugFlag
    plugIn.toggle_debug_drawing(toggleDebugFlag)

def changeVehicleMaxSpeed(e, vehicle):
    """change vehicle's max speed"""
    
    global maxSpeedValue, DEFAULT_MAXVALUE
    if not vehicle:
        return

    if e[:6] == "shift-":
        maxSpeedValue = maxSpeedValue - 1
        if maxSpeedValue < DEFAULT_MAXVALUE:
            maxSpeedValue = DEFAULT_MAXVALUE
    else:
        maxSpeedValue = maxSpeedValue + 1

    vehicle.set_max_speed(maxSpeedValue)
    print(str(vehicle) + "'s max speed is " + str(vehicle.get_max_speed()))  

def changeVehicleMaxForce(e, vehicle):
    """change vehicle's max force"""
    
    global maxForceValue, DEFAULT_MAXVALUE
    if not vehicle:
        return

    if e[:6] == "shift-":
        maxForceValue = maxForceValue - 0.1
        if maxForceValue < DEFAULT_MAXVALUE / 10.0:
            maxForceValue = DEFAULT_MAXVALUE / 10.0
    else:
        maxForceValue = maxForceValue + 0.1

    vehicle.set_max_force(maxForceValue)
    print(str(vehicle) + "'s max force is " + str(vehicle.get_max_force()))  

def getRandomPos(modelNP):
    """return a random point on the facing upwards surface of the model"""
    
    # collisions are made wrt render
    steerMgr = OSSteerManager.get_global_ptr()
    # get the bounding box of scene
    modelDims, modelDeltaCenter = (LVecBase3f(), LVector3f())
    # modelRadius not used
    steerMgr.get_bounding_dimensions(modelNP, modelDims, modelDeltaCenter)
    # throw a ray downward from a point with z = double scene's height
    # and x,y randomly within the scene's (x,y) plane
    # set the ray origin at double of maximum height of the model
    zOrig = ((-modelDeltaCenter.get_z() + modelDims.get_z() / 2.0) + modelNP.get_z()) * 2.0
    while True:
        x = modelDims.get_x() * (random.uniform(0.0, 1.0) - 0.5) - modelDeltaCenter.get_x() + modelNP.get_x()
        y = modelDims.get_y() * (random.uniform(0.0, 1.0) - 0.5) - modelDeltaCenter.get_y() + modelNP.get_y()
        gotCollisionZ = steerMgr.get_collision_height(LPoint3f(x, y, zOrig))
        if gotCollisionZ.get_first():
            break
    return LPoint3f(x, y, gotCollisionZ.get_second())

class HandleVehicleData:
    """ data passed to vehicle's handling callback"""
    
    def __init__(self, meanScale, vehicleFileIdx, moveType, sceneNP, vehicleNP, 
                 steerPlugIn, steerVehicle, vehicleAnimCtls):
        self.meanScale = meanScale
        self.vehicleFileIdx = vehicleFileIdx
        self.moveType = moveType
        self.sceneNP = sceneNP
        self.vehicleNP = vehicleNP
        self.steerPlugIn = steerPlugIn
        self.steerVehicle = steerVehicle
        self.vehicleAnimCtls = vehicleAnimCtls
        
def handleVehicles(data):
    """handle add/remove obstacles""" 
    
    global app

    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        # get the hit object
        hitObject = entry0.get_into_node_path()
        print("hit " + str(hitObject) + " object")

        sceneNP = data.sceneNP
        # check if sceneNP is the hitObject or an ancestor thereof
        if (sceneNP == hitObject) or sceneNP.is_ancestor_of(hitObject):
            # the hit object is the scene: add an vehicle to the scene
            meanScale = data.meanScale
            vehicleFileIdx = data.vehicleFileIdx
            moveType = data.moveType
            vehicleNP = data.vehicleNP
            steerPlugIn = data.steerPlugIn
            steerVehicle = data.steerVehicle
            vehicleAnimCtls = data.vehicleAnimCtls            
            # add vehicle
            pos = entry0.get_surface_point(NodePath())
            getVehicleModelAnims(meanScale, vehicleFileIdx, moveType, sceneNP, 
                                 vehicleNP, steerPlugIn, steerVehicle, 
                                 vehicleAnimCtls, pos)
            # show the added vehicles
            print("Vehicles added to plug-in so far:")
            for vehicle in steerPlugIn:
                print("\t- " + str(vehicle))

def getVehicleModelAnims(meanScale, vehicleFileIdx, moveType, sceneNP, vehicleNP, steerPlugIn, 
                           steerVehicle, vehicleAnimCtls, pos = None):
    """get a vehicle, model and animations"""
    
    global app, vehicleAnimFiles
    # get some models, with animations, to attach to vehicles
    # get the model
    vehicleNP.append(app.loader.load_model(vehicleFile[vehicleFileIdx]))
    # set random scale (0.35 - 0.45)
    scale = meanScale + 0.1 * random.uniform(0.0, 1.0)
    vehicleNP[-1].set_scale(scale)
    # associate an anim with a given anim control
    tmpAnims = AnimControlCollection()
    vehicleAnimNP = [None, None]
    # first anim -> modelAnimCtls[i][0]
    vehicleAnimNP[0] = app.loader.load_model(vehicleAnimFiles[vehicleFileIdx][0])
    vehicleAnimNP[0].reparent_to(vehicleNP[-1])
    auto_bind(vehicleNP[-1].node(), tmpAnims)
    vehicleAnimCtls.append([None, None])
    vehicleAnimCtls[-1][0] = tmpAnims.get_anim(0)
    tmpAnims.clear_anims()
    vehicleAnimNP[0].detach_node()
    # second anim -> modelAnimCtls[i][1]
    vehicleAnimNP[1] = app.loader.load_model(vehicleAnimFiles[vehicleFileIdx][1])
    vehicleAnimNP[1].reparent_to(vehicleNP[-1])
    auto_bind(vehicleNP[-1].node(), tmpAnims)
    vehicleAnimCtls[-1][1] = tmpAnims.get_anim(0)
    tmpAnims.clear_anims()
    vehicleAnimNP[1].detach_node()
    # reparent all node paths
    vehicleAnimNP[0].reparent_to(vehicleNP[-1])
    vehicleAnimNP[1].reparent_to(vehicleNP[-1])
    # set parameter for vehicle's move type (OPENSTEER or OPENSTEER_KINEMATIC)
    steerMgr = OSSteerManager.get_global_ptr()
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "mov_type",
            moveType)
    # create the steer vehicle (attached to the reference node)
    steerVehicleNP = steerMgr.create_steer_vehicle("vehicle" + str(len(vehicleNP) - 1))
    steerVehicle.append(steerVehicleNP.node())
    randPos = pos
    if randPos == None:
        # set the position randomly
        randPos = getRandomPos(sceneNP)
    steerVehicleNP.set_pos(randPos)
    # attach some geometry (a model) to steer vehicle
    vehicleNP[-1].reparent_to(steerVehicleNP)
    # add the steer vehicle to the plug-in
    steerPlugIn.add_steer_vehicle(steerVehicleNP)

def readFromBamFile(fileName):
    """read scene from a file"""
    
    return OSSteerManager.get_global_ptr().read_from_bam_file(fileName)

def writeToBamFileAndExit(fileName):
    """write scene to a file (and exit)"""
    
    OSSteerManager.get_global_ptr().write_to_bam_file(fileName)
    #
    sys.exit(0)

class HandleObstacleData:
    """ data passed to obstacle's handling callback"""
    
    def __init__(self, addObstacle, sceneNP, steerPlugIn):
        self.addObstacle = addObstacle
        self.sceneNP = sceneNP
        self.steerPlugIn = steerPlugIn

def handleObstacles(data):
    """handle add/remove obstacles"""
    
    global app

    addObstacle = data.addObstacle
    sceneNP = data.sceneNP
    steerPlugIn = data.steerPlugIn
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
