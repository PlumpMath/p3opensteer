'''
Created on Jun 20, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager
from panda3d.core import GeoMipTerrain, PNMImage, Filename, TextureStage, \
                TexturePool, BitMask32, CardMaker, NodePath

# global data
dataDir = "../data"
mask = BitMask32(0x10);
terrain = None
terrainRootNetPos = None
# # functions' declarations and definitions

def loadPlane():
    """load plane stuff"""
    
    cm = CardMaker("plane")
    cm.set_frame(-15, 15, -15, 15)
    plane = NodePath(cm.generate())
    plane.set_p(-90.0)
    plane.set_z(0.0)
    plane.set_color(0.15, 0.35, 0.35)
    return plane

def loadTerrain():
    """load terrain stuff"""

    global app, terrain, terrainRootNetPos
        
    steerMgr = OSSteerManager.get_global_ptr()

    terrain = GeoMipTerrain("terrain")
    heightField = PNMImage(Filename(dataDir + "/heightfield.png"))
    terrain.set_heightfield(heightField)
    # sizing
    widthScale, heightScale = (3.0, 100.0)
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
    # get nav mesh manager
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
