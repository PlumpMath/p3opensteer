'''
Created on Jun 18, 2016

@author: consultit
'''

import panda3d.core
from p3opensteer import OSSteerManager
from panda3d.core import load_prc_file_data, PNMImage, GeoMipTerrain, \
                LPoint3f, Filename, TextureStage, TexturePool
#                 BitMask32, LVector3f, LVecBase3f, LPoint3f, \
#                 AnimControlCollection, auto_bind, TextNode, 
from direct.showbase.ShowBase import ShowBase

dataDir = "../data"
# global data
app = None
# models and animations
terrain = None
terrainRootNetPos = LPoint3f()

# # functions' declarations and definitions
 
def loadAllScene():
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

if __name__ == '__main__':
    # Load your application's configuration
    load_prc_file_data("", "model-path " + dataDir)
    load_prc_file_data("", "win-size 1024 768")
    load_prc_file_data("", "show-frame-rate-meter #t")
    load_prc_file_data("", "sync-video #t")
        
    # Setup your application
    app = ShowBase()
       
    # # here is room for your own code
    
    print("create a steer manager")
    steerMgr = OSSteerManager()

    print("reparent the reference node to render")
    steerMgr.get_reference_node_path().reparent_to(app.render)

    print("get a terrain reparented to the reference node")
    terrain = loadAllScene()
    
    print("create a plug in (it is attached to the reference node)")
    plugInNP = steerMgr.create_steer_plug_in()
    plugIn = plugInNP.node()
    
#     print("mandatory: set sceneNP as owner of plugIn")
#     plugIn.set_owner_node_path(sceneNP)
    
#     print("setup the plugIn with sceneNP as its owner object")
#     plugIn.setup()

#     print("reparent sceneNP to the reference node")
#     sceneNP.reparent_to(steerMgr.get_reference_node_path())
    
#     print("get the agent model")
#     agentNP = app.loader.load_model("eve.egg")
#     agentNP.set_scale(0.40)

#     print("create the crowd agent (it is attached to the reference node) and set its position")
#     crowdAgentNP = steerMgr.create_crowd_agent("crowdAgent")
#     crowdAgent = crowdAgentNP.node()
#     crowdAgentNP.set_pos(24.0, -20.4, -2.37)
    
#     print("attach the agent model to crowdAgent")
#     agentNP.reparent_to(crowdAgentNP)
    
#     print("attach the crowd agent to the nav mesh")
#     plugIn.add_crowd_agent(crowdAgentNP)

    print("start the default update task for all plug-ins")
    steerMgr.start_default_update()

    print("DEBUG DRAWING: make the debug reference node path sibling of the reference node")
    steerMgr.get_reference_node_path_debug().reparent_to(app.render)
    print("enable debug drawing")
    plugIn.enable_debug_drawing(app.camera)

    print("toggle debug draw")
    plugIn.toggle_debug_drawing(True)
    
#     print("set crowd agent move target on scene surface")
#     crowdAgent.set_move_target(LPoint3f(-20.5, 5.2, -2.36))
    
    # place camera
    trackball = app.trackball.node()
    trackball.set_pos(-750.0, 400.0, -400.0);
    trackball.set_hpr(0.0, 30.0, 0.0);
   
    # app.run(), equals to do the main loop in C++
    app.run()

