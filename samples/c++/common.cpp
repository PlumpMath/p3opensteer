/**
 * \file common.cpp
 *
 * \date 2016-06-20
 * \author consultit
 */

#include "common.h"
#include "data.h"

LPoint3f terrainRootNetPos;

///functions' definitions
// load all scene stuff
GeoMipTerrain* loadTerrain()
{
	OSSteerManager* steerMgr = OSSteerManager::get_global_ptr();

	GeoMipTerrain *myTerrain = new GeoMipTerrain("myTerrain");
	PNMImage heightField(Filename(dataDir + string("/heightfield.png")));
	myTerrain->set_heightfield(heightField);
	//sizing
	float widthScale = 3.0, heightScale = 100.0;
	float environmentWidthX = (heightField.get_x_size() - 1) * widthScale;
	float environmentWidthY = (heightField.get_y_size() - 1) * widthScale;
	float environmentWidth = (environmentWidthX + environmentWidthY) / 2.0;
	myTerrain->get_root().set_sx(widthScale);
	myTerrain->get_root().set_sy(widthScale);
	myTerrain->get_root().set_sz(heightScale);
	//set other terrain's properties
	unsigned short blockSize = 64, minimumLevel = 0;
	float nearPercent = 0.1, farPercent = 0.7;
	float terrainLODmin = min<float>(minimumLevel, myTerrain->get_max_level());
	GeoMipTerrain::AutoFlattenMode flattenMode = GeoMipTerrain::AFM_off;
	myTerrain->set_block_size(blockSize);
	myTerrain->set_near(nearPercent * environmentWidth);
	myTerrain->set_far(farPercent * environmentWidth);
	myTerrain->set_min_level(terrainLODmin);
	myTerrain->set_auto_flatten(flattenMode);
	//myTerrain texturing
	PT(TextureStage)textureStage0 =
	new TextureStage("TextureStage0");
	PT(Texture)textureImage = TexturePool::load_texture(
			Filename(string("terrain.png")));
	myTerrain->get_root().set_tex_scale(textureStage0, 1.0, 1.0);
	myTerrain->get_root().set_texture(textureStage0, textureImage, 1);
	//reparent this Terrain node path to the object node path
	myTerrain->get_root().reparent_to(steerMgr->get_reference_node_path());
	//brute force generation
	bool bruteForce = true;
	myTerrain->set_bruteforce(bruteForce);
	//Generate the myTerrain
	myTerrain->generate();
	//check if terrain needs update or not
	if (! bruteForce)
	{
		//save the net pos of myTerrain root
		terrainRootNetPos = terrain->get_root().get_net_transform()->get_pos();
		// Add a task to keep updating the myTerrain
		framework.get_task_mgr().add(
				new GenericAsyncTask("terrainUpdate", &terrainUpdate,
						(void*) NULL));
	}
	//
	return myTerrain;
}

// terrain update
AsyncTask::DoneStatus terrainUpdate(GenericAsyncTask* task, void* data)
{
	//set focal point
	//see https://www.panda3d.org/forums/viewtopic.php?t=5384
	LPoint3f focalPointNetPos =
			window->get_camera_group().get_net_transform()->get_pos();
	terrain->set_focal_point(focalPointNetPos - terrainRootNetPos);
	//update every frame
	terrain->update();
	//
	return AsyncTask::DS_cont;
}


