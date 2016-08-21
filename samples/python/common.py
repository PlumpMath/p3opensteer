'''
Created on Jun 20, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager, ValueList_float
from panda3d.core import load_prc_file_data, GeoMipTerrain, PNMImage, \
                Filename, TextureStage, TexturePool, BitMask32, CardMaker, \
                NodePath, WindowProperties, AnimControlCollection, auto_bind, \
                LVecBase3f, LVector3f, LPoint3f, PartGroup, ClockObject
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
# models and animations
vehicleFile = ["eve.egg", "ralph.egg", "sparrow.egg", "ball.egg", "red_car.egg"]
vehicleAnimFiles = [["eve-walk.egg", "eve-run.egg"],
                  ["ralph-walk.egg", "ralph-run.egg"],
                  ["sparrow-flying.egg", "sparrow-flying2.egg"],
                  ["", ""],
                  ["red_car-anim.egg", "red_car-anim2.egg"]]
animRateFactor = [1.20, 4.80]
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

def loadPlane(name, widthX = 30.0, widthY = 30.0):
    """load plane stuff"""
    
    cm = CardMaker("plane")
    cm.set_frame(-widthX / 2.0, widthX / 2.0, -widthY / 2.0, widthY / 2.0)
    plane = NodePath(cm.generate())
    plane.set_p(-90.0)
    plane.set_z(0.0)
    plane.set_color(0.15, 0.35, 0.35)
    plane.set_collide_mask(mask)
    plane.set_name(name)
    return plane

def loadTerrainLowPoly(name, widthScale=128, heightScale=64.0,
                       texture="dry-grass.png"):
    """load terrain low poly stuff"""
    
    global app
    terrainNP = app.loader.load_model("terrain-low-poly.egg")
    terrainNP.set_name(name)
    terrainNP.set_scale(widthScale, widthScale, heightScale)
    tex = app.loader.load_texture(texture)
    terrainNP.set_texture(tex)
    return terrainNP

def loadTerrain(name, widthScale = 0.5, heightScale = 10.0):
    """load terrain stuff"""

    global app, terrain, terrainRootNetPos
        
    steerMgr = OSSteerManager.get_global_ptr()

    terrain = GeoMipTerrain("terrain")
    heightField = PNMImage(Filename(dataDir + "/heightfield.png"))
    terrain.set_heightfield(heightField)
    # sizing
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
    # see https:#www.panda3d.org/forums/viewtopic.php?t=5384
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

def toggleDebugDraw(plugIn = None):
    """toggle debug draw"""
    
    global toggleDebugFlag
    if plugIn == None:
        return

    toggleDebugFlag = not toggleDebugFlag
    plugIn.toggle_debug_drawing(toggleDebugFlag)

def changeVehicleMaxSpeed(e, vehicles = None):
    """change vehicle's max speed"""
    
    global DEFAULT_MAXVALUE
    if (vehicles == None) or (len(vehicles) == 0):
        return

    maxSpeedValue = vehicles[-1].get_max_speed()
    if e[:6] == "shift-":
        maxSpeedValue = maxSpeedValue - 1
        if maxSpeedValue < DEFAULT_MAXVALUE:
            maxSpeedValue = DEFAULT_MAXVALUE
    else:
        maxSpeedValue = maxSpeedValue + 1

    vehicles[-1].set_max_speed(maxSpeedValue)
    print(str(vehicles[-1]) + "'s max speed is " + str(vehicles[-1].get_max_speed()))  

def changeVehicleMaxForce(e, vehicles = None):
    """change vehicle's max force"""
    
    global DEFAULT_MAXVALUE
    if (vehicles == None) or (len(vehicles) == 0):
        return

    maxForceValue = vehicles[-1].get_max_force()
    if e[:6] == "shift-":
        maxForceValue = maxForceValue - 0.1
        if maxForceValue < DEFAULT_MAXVALUE / 10.0:
            maxForceValue = DEFAULT_MAXVALUE / 10.0
    else:
        maxForceValue = maxForceValue + 0.1

    vehicles[-1].set_max_force(maxForceValue)
    print(str(vehicles[-1]) + "'s max force is " + str(vehicles[-1].get_max_force()))  

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
    
    def __init__(self, meanScale, vehicleFileIdx, moveType, sceneNP, 
                 steerPlugIn, steerVehicles, vehicleAnimCtls, 
                 deltaPos = LVector3f.zero()):
        self.meanScale = meanScale
        self.vehicleFileIdx = vehicleFileIdx
        self.moveType = moveType
        self.sceneNP = sceneNP
        self.steerPlugIn = steerPlugIn
        self.steerVehicles = steerVehicles
        self.vehicleAnimCtls = vehicleAnimCtls
        self.deltaPos = deltaPos
        
def handleVehicles(data = None):
    """handle add vehicles""" 
    
    global app
    if data == None:
        return
    
    # get the collision entry, if any
    entry0 = getCollisionEntryFromCamera()
    if entry0:
        # get the hit object
        hitObject = entry0.get_into_node_path()
        print("hit " + str(hitObject) + " object")
        
        vehicleData = data
        sceneNP = vehicleData.sceneNP
        # check if sceneNP is the hitObject or an ancestor thereof
        if (sceneNP == hitObject) or sceneNP.is_ancestor_of(hitObject):
            # the hit object is the scene: add an vehicle to the scene
            meanScale = vehicleData.meanScale
            vehicleFileIdx = vehicleData.vehicleFileIdx
            moveType = vehicleData.moveType
            steerPlugIn = vehicleData.steerPlugIn
            steerVehicles = vehicleData.steerVehicles
            vehicleAnimCtls = vehicleData.vehicleAnimCtls
            deltaPos = vehicleData.deltaPos          
            # add vehicle
            pos = entry0.get_surface_point(NodePath()) + deltaPos
            getVehicleModelAnims(meanScale, vehicleFileIdx, moveType, sceneNP, 
                                 steerPlugIn, steerVehicles, 
                                 vehicleAnimCtls, pos)
            # show the added vehicles
            print("Vehicles added to plug-in so far:")
            for vehicle in steerPlugIn:
                print("\t- " + str(vehicle))

def getVehicleModelAnims(meanScale, vehicleFileIdx, moveType, sceneNP, steerPlugIn, 
                           steerVehicles, vehicleAnimCtls, pos = None):
    """get a vehicle, model and animations"""
    
    global app, vehicleAnimFiles
    # get some models, with animations, to attach to vehicles
    # get the model
    vehicleNP = app.loader.load_model(vehicleFile[vehicleFileIdx])
    # set random scale
    scale = meanScale * (1 + 0.2 * (2 * random.uniform(0.0, 1.0) - 1))
    vehicleNP.set_scale(scale)
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
    # set parameter for vehicle's move type (OPENSTEER or OPENSTEER_KINEMATIC)
    steerMgr = OSSteerManager.get_global_ptr()
    steerMgr.set_parameter_value(OSSteerManager.STEERVEHICLE, "mov_type",
            moveType)
    # create the steer vehicle (attached to the reference node)
    steerVehicleNP = steerMgr.create_steer_vehicle("vehicle" + str(len(steerVehicles)))
    steerVehicles.append(steerVehicleNP.node())
    randPos = pos
    if randPos == None:
        # set the position randomly
        randPos = getRandomPos(sceneNP)
    steerVehicleNP.set_pos(randPos)
    # attach some geometry (a model) to steer vehicle
    vehicleNP.reparent_to(steerVehicleNP)
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
    
    def __init__(self, addObstacle, sceneNP, steerPlugIn,
                 scale = LVecBase3f(1.0, 1.0, 1.0)):
        self.addObstacle = addObstacle
        self.sceneNP = sceneNP
        self.steerPlugIn = steerPlugIn
        self.scale = scale

def handleObstacles(data):
    """handle add/remove obstacles"""
    
    global app
    if data == None:
        return

    obstacleData = data
    addObstacle = obstacleData.addObstacle
    sceneNP = obstacleData.sceneNP
    steerPlugIn = obstacleData.steerPlugIn
    scale = obstacleData.scale
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
            # set random scale
            scale = scale * (1 + 0.2 * (2 * random.uniform(0.0, 1.0) - 1))
            obstacleNP.set_scale(scale)
            # set obstacle position
            pos = entry0.get_surface_point(sceneNP)
            obstacleNP.set_pos(sceneNP, pos)
            # try to add to plug-in
            if steerPlugIn.add_obstacle(obstacleNP, "box") < 0:
                # something went wrong remove from scene
                obstacleNP.remove_node()
                return
            print("added " + str(obstacleNP) + " obstacle at " + 
                  str(obstacleNP.get_pos()))
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

class Driver:
    """Driver class"""

    def __init__(self, app, ownerObjectNP, taskSort = 0):
        self.app = app
        self.mWin = self.app.win
        self.mOwnerObjectNP = ownerObjectNP
        self.mTaskSort = taskSort
        self.__do_reset()
        self.__do_initialize()

    def __del__(self):            
        self.__do_finalize()
        self.__do_reset()
        self.mOwnerObjectNP.clear()
        self.mWin = None

    def update(self, task):
        #
        dt = ClockObject.get_global_clock().get_dt()
        #handle mouse
        if self.mMouseMove and (self.mMouseEnabledH or self.mMouseEnabledP):
            md = self.mWin.get_pointer(0)
            deltaX = md.get_x() - self.mCentX
            deltaY = md.get_y() - self.mCentY
    
            if self.mWin.move_pointer(0, self.mCentX, self.mCentY):
                if self.mMouseEnabledH and (deltaX != 0.0):
                    self.mOwnerObjectNP.set_h(
                            self.mOwnerObjectNP.get_h() - deltaX * self.mSensX * self.mSignOfMouse)
                if self.mMouseEnabledP and (deltaY != 0.0):
                    self.mOwnerObjectNP.set_p(
                            self.mOwnerObjectNP.get_p() - deltaY * self.mSensY * self.mSignOfMouse)
            #if self.mMouseMoveKey is True we are controlling mouse movements
            #so we need to reset self.mMouseMove to False
            if self.mMouseMoveKey:
                self.mMouseMove = False
        #update position/orientation
        self.mOwnerObjectNP.set_y(self.mOwnerObjectNP,
                self.mActualSpeedXYZ.get_y() * dt * self.mSignOfTranslation)
        self.mOwnerObjectNP.set_x(self.mOwnerObjectNP,
                self.mActualSpeedXYZ.get_x() * dt * self.mSignOfTranslation)
        self.mOwnerObjectNP.set_z(self.mOwnerObjectNP, self.mActualSpeedXYZ.get_z() * dt)
        #head
        if self.mHeadLimitEnabled:
            head = self.mOwnerObjectNP.get_h() + self.mActualSpeedH * dt * self.mSignOfMouse
            if head > self.mHLimit:
                head = self.mHLimit
            elif head < -self.mHLimit:
                head = -self.mHLimit
            self.mOwnerObjectNP.set_h(head)
        else:
            self.mOwnerObjectNP.set_h(
                    self.mOwnerObjectNP.get_h() + self.mActualSpeedH * dt * self.mSignOfMouse)
        #pitch
        if self.mPitchLimitEnabled:
            pitch = self.mOwnerObjectNP.get_p() + self.mActualSpeedP * dt * self.mSignOfMouse
            if pitch > self.mPLimit:
                pitch = self.mPLimit
            elif pitch < -self.mPLimit:
                pitch = -self.mPLimit
            self.mOwnerObjectNP.set_p(pitch)
        else:
            self.mOwnerObjectNP.set_p(
                    self.mOwnerObjectNP.get_p() + self.mActualSpeedP * dt * self.mSignOfMouse)
    
        #update speeds
        #y axis
        if self.mForward and (not self.mBackward):
            if self.mAccelXYZ.get_y() != 0.0:
                #accelerate
                self.mActualSpeedXYZ.set_y(
                        self.mActualSpeedXYZ.get_y() - self.mAccelXYZ.get_y() * dt)
                if self.mActualSpeedXYZ.get_y() < -self.mMaxSpeedXYZ.get_y():
                    #limit speed
                    self.mActualSpeedXYZ.set_y(-self.mMaxSpeedXYZ.get_y())
            else:
                #kinematic
                self.mActualSpeedXYZ.set_y(-self.mMaxSpeedXYZ.get_y())
        elif self.mBackward and (not self.mForward):
            if self.mAccelXYZ.get_y() != 0.0:
                #accelerate
                self.mActualSpeedXYZ.set_y(
                        self.mActualSpeedXYZ.get_y() + self.mAccelXYZ.get_y() * dt)
                if self.mActualSpeedXYZ.get_y() > self.mMaxSpeedXYZ.get_y():
                    #limit speed
                    self.mActualSpeedXYZ.set_y(self.mMaxSpeedXYZ.get_y())
            else:
                #kinematic
                self.mActualSpeedXYZ.set_y(self.mMaxSpeedXYZ.get_y())
        elif self.mActualSpeedXYZ.get_y() != 0.0:
            if self.mActualSpeedXYZ.get_y() * self.mActualSpeedXYZ.get_y() \
                    < self.mMaxSpeedSquaredXYZ.get_y() * self.mStopThreshold:
                #stop
                self.mActualSpeedXYZ.set_y(0.0)
            else:
                #decelerate
                self.mActualSpeedXYZ.set_y(
                        self.mActualSpeedXYZ.get_y() * (1.0 - min(self.mFrictionXYZ * dt, 1.0)))
        #x axis
        if self.mStrafeLeft and (not self.mStrafeRight):
            if self.mAccelXYZ.get_x() != 0.0:
                #accelerate
                self.mActualSpeedXYZ.set_x(
                        self.mActualSpeedXYZ.get_x() + self.mAccelXYZ.get_x() * dt)
                if self.mActualSpeedXYZ.get_x() > self.mMaxSpeedXYZ.get_x():
                    #limit speed
                    self.mActualSpeedXYZ.set_x(self.mMaxSpeedXYZ.get_x())
            else:
                #kinematic
                self.mActualSpeedXYZ.set_x(self.mMaxSpeedXYZ.get_x())
        elif self.mStrafeRight and (not self.mStrafeLeft):
            if self.mAccelXYZ.get_x() != 0.0:
                #accelerate
                self.mActualSpeedXYZ.set_x(
                        self.mActualSpeedXYZ.get_x() - self.mAccelXYZ.get_x() * dt)
                if self.mActualSpeedXYZ.get_x() < -self.mMaxSpeedXYZ.get_x():
                    #limit speed
                    self.mActualSpeedXYZ.set_x(-self.mMaxSpeedXYZ.get_x())
            else:
                #kinematic
                self.mActualSpeedXYZ.set_x(-self.mMaxSpeedXYZ.get_y())
        elif self.mActualSpeedXYZ.get_x() != 0.0:
            if self.mActualSpeedXYZ.get_x() * self.mActualSpeedXYZ.get_x() \
                    < self.mMaxSpeedSquaredXYZ.get_x() * self.mStopThreshold:
                #stop
                self.mActualSpeedXYZ.set_x(0.0)
            else:
                #decelerate
                self.mActualSpeedXYZ.set_x(
                        self.mActualSpeedXYZ.get_x() * (1.0 - min(self.mFrictionXYZ * dt, 1.0)))
        #z axis
        if self.mUp and (not self.mDown):
            if self.mAccelXYZ.get_z() != 0.0:
                #accelerate
                self.mActualSpeedXYZ.set_z(
                        self.mActualSpeedXYZ.get_z() + self.mAccelXYZ.get_z() * dt)
                if self.mActualSpeedXYZ.get_z() > self.mMaxSpeedXYZ.get_z():
                    #limit speed
                    self.mActualSpeedXYZ.set_z(self.mMaxSpeedXYZ.get_z())
            else:
                #kinematic
                self.mActualSpeedXYZ.set_z(self.mMaxSpeedXYZ.get_z())
        elif self.mDown and (not self.mUp):
            if self.mAccelXYZ.get_z() != 0.0:
                #accelerate
                self.mActualSpeedXYZ.set_z(
                        self.mActualSpeedXYZ.get_z() - self.mAccelXYZ.get_z() * dt)
                if self.mActualSpeedXYZ.get_z() < -self.mMaxSpeedXYZ.get_z():
                    #limit speed
                    self.mActualSpeedXYZ.set_z(-self.mMaxSpeedXYZ.get_z())
            else:
                #kinematic
                self.mActualSpeedXYZ.set_z(-self.mMaxSpeedXYZ.get_z())
        elif self.mActualSpeedXYZ.get_z() != 0.0:
            if self.mActualSpeedXYZ.get_z() * self.mActualSpeedXYZ.get_z() \
                    < self.mMaxSpeedSquaredXYZ.get_z() * self.mStopThreshold:
                #stop
                self.mActualSpeedXYZ.set_z(0.0)
            else:
                #decelerate
                self.mActualSpeedXYZ.set_z(
                        self.mActualSpeedXYZ.get_z() * (1.0 - min(self.mFrictionXYZ * dt, 1.0)))
        #rotation h
        if self.mHeadLeft and (not self.mHeadRight):
            if self.mAccelHP != 0.0:
                #accelerate
                self.mActualSpeedH += self.mAccelHP * dt
                if self.mActualSpeedH > self.mMaxSpeedHP:
                    #limit speed
                    self.mActualSpeedH = self.mMaxSpeedHP
            else:
                #kinematic
                self.mActualSpeedH = self.mMaxSpeedHP
        elif self.mHeadRight and (not self.mHeadLeft):
            if self.mAccelHP != 0.0:
                #accelerate
                self.mActualSpeedH -= self.mAccelHP * dt
                if self.mActualSpeedH < -self.mMaxSpeedHP:
                    #limit speed
                    self.mActualSpeedH = -self.mMaxSpeedHP
            else:
                #kinematic
                self.mActualSpeedH = -self.mMaxSpeedHP
        elif self.mActualSpeedH != 0.0:
            if self.mActualSpeedH * self.mActualSpeedH < self.mMaxSpeedSquaredHP * self.mStopThreshold:
                #stop
                self.mActualSpeedH = 0.0
            else:
                #decelerate
                self.mActualSpeedH = self.mActualSpeedH * (1.0 - min(self.mFrictionHP * dt, 1.0))
        #rotation p
        if self.mPitchUp and (not self.mPitchDown):
            if self.mAccelHP != 0.0:
                #accelerate
                self.mActualSpeedP += self.mAccelHP * dt
                if self.mActualSpeedP > self.mMaxSpeedHP:
                    #limit speed
                    self.mActualSpeedP = self.mMaxSpeedHP
            else:
                #kinematic
                self.mActualSpeedP = self.mMaxSpeedHP
        elif self.mPitchDown and (not self.mPitchUp):
            if self.mAccelHP != 0.0:
                #accelerate
                self.mActualSpeedP -= self.mAccelHP * dt
                if self.mActualSpeedP < -self.mMaxSpeedHP:
                    #limit speed
                    self.mActualSpeedP = -self.mMaxSpeedHP
            else:
                #kinematic
                self.mActualSpeedP = -self.mMaxSpeedHP
        elif self.mActualSpeedP != 0.0:
            if self.mActualSpeedP * self.mActualSpeedP < self.mMaxSpeedSquaredHP * self.mStopThreshold:
                #stop
                self.mActualSpeedP = 0.0
            else:
                #decelerate
                self.mActualSpeedP = self.mActualSpeedP * (1.0 - min(self.mFrictionHP * dt, 1.0))        
                #
        return task.cont

    #enable/disable
    def enable(self):
        #if enabled return
        if self.mEnabled:
            return False
        #actual enabling
        self.__do_enable()
        #
        return True
    
    def disable(self):
        #if not enabled return
        if not self.mEnabled:
            return False
    
        #actual disabling
        self.__do_disable()
        #
        return True
    
    def is_enabled(self):
        return self.mEnabled

    #enable/disable controls    
    def enable_forward(self, enable):
        if self.mForwardKey:
            self.mForward = enable
    
    def is_forward_enabled(self):
        return self.mForward
    
    def enable_backward(self, enable):
        if self.mBackwardKey:
            self.mBackward = enable
    
    def is_backward_enabled(self):
        return self.mBackward
    
    def enable_strafe_left(self, enable):
        if self.mStrafeLeftKey:
            self.mStrafeLeft = enable
    
    def is_strafe_left_enabled(self):
    
        return self.mStrafeLeft
    
    def enable_strafe_right(self, enable):
        if self.mStrafeRightKey:
            self.mStrafeRight = enable
    
    def is_strafe_right_enabled(self):
        return self.mStrafeRight
    
    def enable_up(self, enable):
        if self.mUpKey:
            self.mUp = enable
    
    def is_up_enabled(self):
        return self.mUp
    
    def enable_down(self, enable):
        if self.mDownKey:
            self.mDown = enable
    
    def is_down_enabled(self):
        return self.mDown
    
    def enable_head_left(self, enable):
        if self.mHeadLeftKey:
            self.mHeadLeft = enable
    
    def is_head_left_enabled(self):
        return self.mHeadLeft
    
    def enable_head_right(self, enable):
        if self.mHeadRightKey:
            self.mHeadRight = enable
    
    def is_head_right_enabled(self):
        return self.mHeadRight
    
    def enable_pitch_up(self, enable):
        if self.mPitchUpKey:
            self.mPitchUp = enable
    
    def is_pitch_up_enabled(self):
        return self.mPitchUp
    
    def enable_pitch_down(self, enable):
        if self.mPitchDownKey:
            self.mPitchDown = enable
    
    def is_pitch_down_enabled(self):
        return self.mPitchDown
    
    def enable_mouse_move(self, enable):
        if self.mMouseMoveKey:
            self.mMouseMove = enable
    
    def is_mouse_move_enabled(self):
        return self.mMouseMove

    #max values    
    def set_head_limit(self, enabled, hLimit):
        self.mHeadLimitEnabled = enabled
        if hLimit >= 0.0:
            self.mHLimit = hLimit
        else:
            self.mHLimit = -hLimit
    
    def set_pitch_limit(self, enabled, pLimit):
        self.mPitchLimitEnabled = enabled
        if pLimit >= 0.0:
            self.mPLimit = pLimit
        else:
            self.mPLimit = -pLimit
    
    def set_max_linear_speed(self, maxLinearSpeed):
        self.mMaxSpeedXYZ = LVector3f(
                abs(maxLinearSpeed.get_x()),
                abs(maxLinearSpeed.get_y()),
                abs(maxLinearSpeed.get_z()))
        self.mMaxSpeedSquaredXYZ = LVector3f(
                maxLinearSpeed.get_x() * maxLinearSpeed.get_x(),
                maxLinearSpeed.get_y() * maxLinearSpeed.get_y(),
                maxLinearSpeed.get_z() * maxLinearSpeed.get_z())
    
    def set_max_angular_speed(self, maxAngularSpeed):
        self.mMaxSpeedHP = abs(maxAngularSpeed)
        self.mMaxSpeedSquaredHP = maxAngularSpeed * maxAngularSpeed
    
    def get_max_speeds(self):
        return (self.mMaxSpeedXYZ, self.mMaxSpeedHP)
    
    def set_linear_accel(self, linearAccel):
        self.mAccelXYZ = LVector3f(
                abs(linearAccel.get_x()),
                abs(linearAccel.get_y()),
                abs(linearAccel.get_z()));
    
    def set_angular_accel(self, angularAccel):
        self.mAccelHP = abs(angularAccel)
    
    def get_accels(self):
        return (self.mAccelXYZ, self.mAccelHP)
    
    def set_linear_friction(self, linearFriction):
        self.mFrictionXYZ = abs(linearFriction)
    
    def set_angular_friction(self, angularFriction):
        self.mFrictionHP = abs(angularFriction)
    
    def get_frictions(self):
        frictions = ValueList_float()
        frictions.add_value(self.mFrictionXYZ)
        frictions.add_value(self.mFrictionHP)
        return frictions
    
    def set_sens(self, sensX, sensY):
        self.mSensX = abs(sensX)
        self.mSensY = abs(sensY)
    
    def get_sens(self):
        sens = ValueList_float()
        sens.add_value(self.mSensX)
        sens.add_value(self.mSensY)
        return sens
    
    def set_fast_factor(self, factor):
        self.mFastFactor = abs(factor)
    
    def get_fast_factor(self):
        return self.mFastFactor

    #speed current values    
    def get_current_speeds(self):
        angularSpeeds = [] 
        angularSpeeds.append(self.mActualSpeedH)
        angularSpeeds.append(self.mActualSpeedP)
        return (self.mActualSpeedXYZ, angularSpeeds)

    ##private member functions    
    def __do_reset(self):
        #
        self.mEnabled = False
        self.mForward = self.mBackward = self.mStrafeLeft = self.mStrafeRight = self.mUp = self.mDown = \
                self.mHeadLeft = self.mHeadRight = self.mPitchUp = self.mPitchDown = False
        #by default we consider mouse moved on every update, because
        #we want mouse poll by default this can be changed by calling
        #the enabler (for example by an handler responding to mouse-move
        #event if it is possible. See: http:#www.panda3d.org/forums/viewtopic.php?t=9326
        # http:#www.panda3d.org/forums/viewtopic.php?t=6049)
        self.mMouseMove = True
        self.mForwardKey = self.mBackwardKey = self.mStrafeLeftKey = self.mStrafeRightKey = self.mUpKey = self.mDownKey = \
                self.mHeadLeftKey = self.mHeadRightKey = self.mPitchUpKey = self.mPitchDownKey = \
                        self.mMouseMoveKey = False
        self.mSpeedKey = "shift"
        self.mMouseEnabledH = self.mMouseEnabledP = self.mHeadLimitEnabled = self.mPitchLimitEnabled = \
                False
        self.mHLimit = self.mPLimit = 0.0
        self.mSignOfTranslation = self.mSignOfMouse = 1
        self.mFastFactor = 0.0
        self.mActualSpeedXYZ = self.mMaxSpeedXYZ = self.mMaxSpeedSquaredXYZ = LVecBase3f()
        self.mActualSpeedH = self.mActualSpeedP = self.mMaxSpeedHP = self.mMaxSpeedSquaredHP = 0.0
        self.mAccelXYZ = LVecBase3f()
        self.mAccelHP = 0.0
        self.mFrictionXYZ = self.mFrictionHP = 0.0
        self.mStopThreshold = 0.0
        self.mSensX = self.mSensY = 0.0
        self.mCentX = self.mCentY = 0.0

    def __do_initialize(self):
        #inverted setting (1/-1): not inverted -> 1, inverted -> -1
        self.mSignOfTranslation = 1
        self.mSignOfMouse = 1
        #head limit: enabled@[limit] limit >= 0.0
        self.mHeadLimitEnabled = False
        self.mHLimit = 0.0
        #pitch limit: enabled@[limit] limit >= 0.0
        self.mPitchLimitEnabled = False
        self.mPLimit = 0.0
        #mouse movement setting
        self.mMouseEnabledH = False
        self.mMouseEnabledP = False
        #key events setting
        #backward key
        self.mBackwardKey = True
        #down key
        self.mDownKey = True
        #forward key
        self.mForwardKey = True
        #strafeLeft key
        self.mStrafeLeftKey = True
        #strafeRight key
        self.mStrafeRightKey = True
        #headLeft key
        self.mHeadLeftKey = True
        #headRight key
        self.mHeadRightKey = True
        #pitchUp key
        self.mPitchUpKey = True
        #pitchDown key
        self.mPitchDownKey = True
        #up key
        self.mUpKey = True
        #mouseMove key: enabled/disabled
        self.mMouseMoveKey = False
        #speedKey
        if not (self.mSpeedKey == "control" \
                or self.mSpeedKey == "alt" \
                or self.mSpeedKey == "shift"):
            self.mSpeedKey = "shift"
        #
        #max linear speed (>=0)
        self.mMaxSpeedXYZ = LVecBase3f(5.0, 5.0, 5.0)
        self.mMaxSpeedSquaredXYZ = LVector3f(self.mMaxSpeedXYZ.get_x() * self.mMaxSpeedXYZ.get_x(),
                self.mMaxSpeedXYZ.get_y() * self.mMaxSpeedXYZ.get_y(),
                self.mMaxSpeedXYZ.get_z() * self.mMaxSpeedXYZ.get_z())
        #max angular speed (>=0)
        self.mMaxSpeedHP = 5.0
        self.mMaxSpeedSquaredHP = self.mMaxSpeedHP * self.mMaxSpeedHP
        #linear accel (>=0)
        self.mAccelXYZ = LVecBase3f(5.0, 5.0, 5.0)
        #angular accel (>=0)
        self.mAccelHP = 5.0
        #reset actual speeds
        self.mActualSpeedXYZ = LVector3f()
        self.mActualSpeedH = 0.0
        self.mActualSpeedP = 0.0
        #linear friction (>=0)
        self.mFrictionXYZ = 5.0
        #angular friction (>=0)
        self.mFrictionHP = 5.0
        #stop threshold ([0.0, 1.0])
        self.mStopThreshold = 0.01
        #fast factor (>=0)
        self.mFastFactor = 5.0
        #sens x (>=0)
        self.mSensX = 0.2
        #sens_y (>=0)
        self.mSensY = 0.2
        #
        self.mCentX = self.mWin.get_properties().get_x_size() / 2
        self.mCentY = self.mWin.get_properties().get_y_size() / 2

    def __do_finalize(self):
        #if enabled: disable
        if self.mEnabled:
            #actual disabling
            self.__do_disable()
    
    def __do_enable(self):
        if self.mMouseEnabledH or self.mMouseEnabledP or self.mMouseMoveKey:
            #we want control through mouse movements
            #hide mouse cursor
            props = WindowProperties()
            props.set_cursor_hidden(True)
            self.mWin.request_properties(props)
            #reset mouse to start position
            self.mWin.move_pointer(0, self.mCentX, self.mCentY)
        #
        self.mEnabled = True
    
        #Add self.mUpdateTask to the active queue.
        self.app.taskMgr.add(self.update, "Driver::update", self.mTaskSort, appendTask=True)
    
    def __do_disable(self):
        if self.mMouseEnabledH or self.mMouseEnabledP or self.mMouseMoveKey:
            #we have control through mouse movements
            #show mouse cursor
            props = WindowProperties()
            props.set_cursor_hidden(False)
            self.mWin.request_properties(props)
        #
        self.mEnabled = False
    
        #Remove self.mUpdateTask from the active queue.
        self.app.taskMgr.remove("Driver::update")
