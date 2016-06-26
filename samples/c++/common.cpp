/**
 * \file common.cpp
 *
 * \date 2016-06-20
 * \author consultit
 */

#include "common.h"

///global data
#include "data.h"
CollideMask mask = BitMask32(0x10);
static GeoMipTerrain* terrain;
static LPoint3f terrainRootNetPos;

///functions' definitions

// start base framework
void startFramework(int argc, char *argv[])
{
	// Load your application's configuration
	load_prc_file_data("", "model-path " + dataDir);
	load_prc_file_data("", "win-size 1024 768");
	load_prc_file_data("", "show-frame-rate-meter #t");
	load_prc_file_data("", "sync-video #t");
	// Setup your application
	framework.open_framework(argc, argv);
	framework.set_window_title("p3opensteer");
	window = framework.open_window();
	if (window != (WindowFramework *) NULL)
	{
		cout << "Opened the window successfully!\n";
		window->enable_keyboard();
		window->setup_trackball();
	}

	/// typed object init; not needed if you build inside panda source tree
	OSSteerPlugIn::init_type();
	OSSteerVehicle::init_type();
	OSSteerManager::init_type();
	OSSteerPlugIn::register_with_read_factory();
	OSSteerVehicle::register_with_read_factory();
	///
}

// load plane stuff
NodePath loadPlane()
{
	CardMaker cm("plane");
	cm.set_frame(-15, 15, -15, 15);
	NodePath plane(cm.generate());
	plane.set_p(-90.0);
	plane.set_z(0.0);
	plane.set_color(0.15, 0.35, 0.35);
	return plane;
}

// terrain update
static AsyncTask::DoneStatus terrainUpdate(GenericAsyncTask* task, void* data)
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

// load terrain stuff
NodePath loadTerrain()
{
	GeoMipTerrain *terrain = new GeoMipTerrain("terrain");
	PNMImage heightField(Filename(dataDir + string("/heightfield.png")));
	terrain->set_heightfield(heightField);
	//sizing
	float widthScale = 3.0, heightScale = 100.0;
	float environmentWidthX = (heightField.get_x_size() - 1) * widthScale;
	float environmentWidthY = (heightField.get_y_size() - 1) * widthScale;
	float environmentWidth = (environmentWidthX + environmentWidthY) / 2.0;
	terrain->get_root().set_sx(widthScale);
	terrain->get_root().set_sy(widthScale);
	terrain->get_root().set_sz(heightScale);
	//set other terrain's properties
	unsigned short blockSize = 64, minimumLevel = 0;
	float nearPercent = 0.1, farPercent = 0.7;
	float terrainLODmin = min<float>(minimumLevel, terrain->get_max_level());
	GeoMipTerrain::AutoFlattenMode flattenMode = GeoMipTerrain::AFM_off;
	terrain->set_block_size(blockSize);
	terrain->set_near(nearPercent * environmentWidth);
	terrain->set_far(farPercent * environmentWidth);
	terrain->set_min_level(terrainLODmin);
	terrain->set_auto_flatten(flattenMode);
	//terrain texturing
	PT(TextureStage)textureStage0 = new TextureStage("TextureStage0");
	PT(Texture)textureImage = TexturePool::load_texture(
			Filename(string("terrain.png")));
	terrain->get_root().set_tex_scale(textureStage0, 1.0, 1.0);
	terrain->get_root().set_texture(textureStage0, textureImage, 1);
	//brute force generation
	bool bruteForce = true;
	terrain->set_bruteforce(bruteForce);
	//Generate the terrain
	terrain->generate();
	//check if terrain needs update or not
	if (!bruteForce)
	{
		//save the net pos of terrain root
		terrainRootNetPos = terrain->get_root().get_net_transform()->get_pos();
		// Add a task to keep updating the terrain
		framework.get_task_mgr().add(
				new GenericAsyncTask("terrainUpdate", &terrainUpdate,
						(void*) NULL));
	}
	//
	return terrain->get_root();
}

// throws a ray and returns the first collision entry or nullptr
PT(CollisionEntry)getCollisionEntryFromCamera()
{
	// get nav mesh manager
	OSSteerManager* steerMgr = OSSteerManager::get_global_ptr();
	// get the mouse watcher
	PT(MouseWatcher)mwatcher = DCAST(MouseWatcher, window->get_mouse().node());
	if (mwatcher->has_mouse())
	{
		// Get to and from pos in camera coordinates
		LPoint2f pMouse = mwatcher->get_mouse();
		//
		LPoint3f pFrom, pTo;
		NodePath mCamera = window->get_camera_group();
		PT(Lens)mCamLens = DCAST(Camera, mCamera.get_child(0).node())->get_lens();
		if (mCamLens->extrude(pMouse, pFrom, pTo))
		{
			// Transform to global coordinates
			pFrom = window->get_render().get_relative_point(mCamera,
					pFrom);
			pTo = window->get_render().get_relative_point(mCamera, pTo);
			LVector3f direction = (pTo - pFrom).normalized();
			steerMgr->get_collision_ray()->set_origin(pFrom);
			steerMgr->get_collision_ray()->set_direction(direction);
			steerMgr->get_collision_traverser()->traverse(window->get_render());
			// check collisions
			if (steerMgr->get_collision_handler()->get_num_entries() > 0)
			{
				// Get the closest entry
				steerMgr->get_collision_handler()->sort_entries();
				return steerMgr->get_collision_handler()->get_entry(0);
			}
		}
	}
	return nullptr;
}

// print creation parameters
void printCreationParameters()
{
	OSSteerManager* steerMgr = OSSteerManager::get_global_ptr();
	//
	ValueList<string> valueList = steerMgr->get_parameter_name_list(
			OSSteerManager::STEERPLUGIN);
	cout << endl << "OSSteerPlugIn creation parameters:" << endl;
	for (int i = 0; i < valueList.get_num_values(); ++i)
	{
		cout << "\t" << valueList[i] << " = "
				<< steerMgr->get_parameter_value(OSSteerManager::STEERPLUGIN,
						valueList[i]) << endl;
	}
	//
	valueList = steerMgr->get_parameter_name_list(
			OSSteerManager::STEERVEHICLE);
	cout << endl << "OSSteerVehicle creation parameters:" << endl;
	for (int i = 0; i < valueList.get_num_values(); ++i)
	{
		cout << "\t" << valueList[i] << " = "
				<< steerMgr->get_parameter_value(OSSteerManager::STEERVEHICLE,
						valueList[i]) << endl;
	}
}

// handle vehicle's events
void handleVehicleEvent(const Event* e, void* data)
{
	PT(OSSteerVehicle)vehicle = DCAST(OSSteerVehicle, e->get_parameter(0).get_ptr());
	NodePath vehicleNP = NodePath::any_path(vehicle);
	cout << "move-event - '" << vehicleNP.get_name() << "' - "<< vehicleNP.get_pos() << endl;
}
