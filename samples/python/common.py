'''
Created on Jun 20, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager
from panda3d.core import GeoMipTerrain, PNMImage, Filename, TextureStage, \
                TexturePool

dataDir = "../data"
# # functions' declarations and definitions
 
def loadTerrain():
    """load all scene stuff"""

    global app
        
    steerMgr = OSSteerManager.get_global_ptr()

    myTerrain = GeoMipTerrain("myTerrain")
    heightField = PNMImage(Filename(dataDir + "/heightfield.png"))
    myTerrain.set_heightfield(heightField)
    # sizing
    widthScale, heightScale = (3.0, 100.0)
    environmentWidthX = (heightField.get_x_size() - 1) * widthScale
    environmentWidthY = (heightField.get_y_size() - 1) * widthScale
    environmentWidth = (environmentWidthX + environmentWidthY) / 2.0
    myTerrain.get_root().set_sx(widthScale)
    myTerrain.get_root().set_sy(widthScale)
    myTerrain.get_root().set_sz(heightScale)
    # set other terrain's properties
    blockSize, minimumLevel = (64, 0)
    nearPercent, farPercent = (0.1, 0.7)
    terrainLODmin = min(minimumLevel, myTerrain.get_max_level())
    flattenMode = GeoMipTerrain.AFM_off
    myTerrain.set_block_size(blockSize)
    myTerrain.set_near(nearPercent * environmentWidth)
    myTerrain.set_far(farPercent * environmentWidth)
    myTerrain.set_min_level(terrainLODmin)
    myTerrain.set_auto_flatten(flattenMode)
    # myTerrain texturing
    textureStage0 = TextureStage("TextureStage0")
    textureImage = TexturePool.load_texture(Filename("terrain.png"))
    myTerrain.get_root().set_tex_scale(textureStage0, 1.0, 1.0)
    myTerrain.get_root().set_texture(textureStage0, textureImage, 1)
    # reparent this Terrain node path to the object node path
    myTerrain.get_root().reparent_to(steerMgr.get_reference_node_path())
    # brute force generation
    bruteForce = True
    myTerrain.set_bruteforce(bruteForce)
    # Generate the myTerrain
    myTerrain.generate()
    # check if terrain needs update or not
    if not bruteForce:
        # save the net pos of myTerrain root
        terrainRootNetPos = terrain.get_root().get_net_transform().get_pos()
        # Add a task to keep updating the myTerrain
        app.taskMgr.add(terrainUpdate, "terrainUpdate", appendTask=True)
    #
    return myTerrain

def terrainUpdate(task):
    """terrain update"""

    global app, terrain
    # set focal point
    # see https://www.panda3d.org/forums/viewtopic.php?t=5384
    focalPointNetPos = app.camera.get_net_transform().get_pos()
    terrain.set_focal_point(focalPointNetPos - terrainRootNetPos)
    # update every frame
    terrain.update()
    #
    return task.cont

