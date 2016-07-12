/**
 * \file common.cpp
 *
 * \date 2016-06-20
 * \author consultit
 */

#include "common.h"

///global data
#include "data.h"
PandaFramework framework;
WindowFramework *window;
CollideMask mask = BitMask32(0x10);
AsyncTask* updateTask;
bool toggleDebugFlag = false;
static GeoMipTerrain* terrain;
static LPoint3f terrainRootNetPos;
#define DEFAULT_MAXVALUE 1.0
//models and animations
string vehicleFile[2] =
{ "eve.egg", "ralph.egg" };
string vehicleAnimFiles[2][2] =
{
{ "eve-walk.egg", "eve-run.egg" },
{ "ralph-walk.egg", "ralph-run.egg" } };
const float rateFactor[2] =
{ 1.20, 3.40 };
//obstacle model
string obstacleFile("plants2.egg");
//bame file
string bamFileName("plug_in.boo");
//support
random_device rd;

///functions' definitions
// start base framework
void startFramework(int argc, char *argv[], const string& msg)
{
	// Load your application's configuration
	load_prc_file_data("", "model-path " + dataDir);
	load_prc_file_data("", "win-size 1024 768");
	load_prc_file_data("", "show-frame-rate-meter #t");
	load_prc_file_data("", "sync-video #t");
	// Setup your application
	framework.open_framework(argc, argv);
	framework.set_window_title("p3opensteer: " + msg);
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

	//common callbacks
}

// load plane stuff
NodePath loadPlane(const string& name)
{
	CardMaker cm("plane");
	cm.set_frame(-15, 15, -15, 15);
	NodePath plane(cm.generate());
	plane.set_p(-90.0);
	plane.set_z(0.0);
	plane.set_color(0.15, 0.35, 0.35);
	plane.set_collide_mask(mask);
	plane.set_name(name);
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
NodePath loadTerrain(const string& name)
{
	GeoMipTerrain *terrain = new GeoMipTerrain("terrain");
	PNMImage heightField(Filename(dataDir + string("/heightfield.png")));
	terrain->set_heightfield(heightField);
	//sizing
	float widthScale = 0.5, heightScale = 10.0;
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
	terrain->get_root().set_collide_mask(mask);
	terrain->get_root().set_name(name);
	//brute force generation
	bool bruteForce = true;
	terrain->set_bruteforce(bruteForce);
	//Generate the terrain
	terrain->generate();
	//check if terrain needs update or not
	if (not bruteForce)
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
	// get steer manager
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
	valueList = steerMgr->get_parameter_name_list(OSSteerManager::STEERVEHICLE);
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
	PT(OSSteerVehicle)vehicle = DCAST(OSSteerVehicle,
			e->get_parameter(0).get_ptr());
	NodePath vehicleNP = NodePath::any_path(vehicle);

	cout << "got " << e->get_name() << " event from '" << vehicleNP.get_name()
			<< "' at " << vehicleNP.get_pos() << endl;
}

// toggle debug draw
void toggleDebugDraw(const Event* e, void* data)
{
	PT(OSSteerPlugIn)plugIn = reinterpret_cast<OSSteerPlugIn*>(data);
	if(not plugIn)
	{
		return;
	}

	toggleDebugFlag = not toggleDebugFlag;
	plugIn->toggle_debug_drawing(toggleDebugFlag);
}

// change vehicle's max speed
void changeVehicleMaxSpeed(const Event* e, void* data)
{
	vector<PT(OSSteerVehicle)>*vehicles =
			reinterpret_cast<vector<PT(OSSteerVehicle)>*>(data);
	if((not vehicles) or (vehicles->size() == 0))
	{
		return;
	}

	float maxSpeedValue = vehicles->back()->get_max_speed();
	if (e->get_name().substr(0, 6) == string("shift-"))
	{
		maxSpeedValue = maxSpeedValue - 1;
		if (maxSpeedValue < DEFAULT_MAXVALUE)
		{
			maxSpeedValue = DEFAULT_MAXVALUE;
		}
	}
	else
	{
		maxSpeedValue = maxSpeedValue + 1;
	}

	vehicles->back()->set_max_speed(maxSpeedValue);
	cout << *(vehicles->back()) << "'s max speed is " <<
			(vehicles->back())->get_max_speed() << endl;
}

// change vehicle's max force
void changeVehicleMaxForce(const Event* e, void* data)
{
	vector<PT(OSSteerVehicle)>*vehicles =
			reinterpret_cast<vector<PT(OSSteerVehicle)>*>(data);
	if((not vehicles) or (vehicles->size() == 0))
	{
		return;
	}

	float maxForceValue = vehicles->back()->get_max_force();
	if (e->get_name().substr(0, 6) == string("shift-"))
	{
		maxForceValue = maxForceValue - 0.1;
		if (maxForceValue < DEFAULT_MAXVALUE / 10.0)
		{
			maxForceValue = DEFAULT_MAXVALUE / 10.0;
		}
	}
	else
	{
		maxForceValue = maxForceValue + 0.1;
	}

	vehicles->back()->set_max_force(maxForceValue);
	cout << *(vehicles->back()) << "'s max force is " <<
			(vehicles->back())->get_max_force() << endl;
}

// return a random point on the facing upwards surface of the model
LPoint3f getRandomPos(NodePath modelNP)
{
	// collisions are made wrt render
	OSSteerManager* steerMgr = OSSteerManager::get_global_ptr();
	// get the bounding box of scene
	LVecBase3f modelDims;
	LVector3f modelDeltaCenter;
	// modelRadius not used
	steerMgr->get_bounding_dimensions(modelNP, modelDims, modelDeltaCenter);
	// throw a ray downward from a point with z = double scene's height
	// and x,y randomly within the scene's (x,y) plane
	float x, y = 0.0;
	Pair<bool, float> gotCollisionZ;
	// set the ray origin at double of maximum height of the model
	float zOrig = ((-modelDeltaCenter.get_z() + modelDims.get_z() / 2.0)
			+ modelNP.get_z()) * 2.0;
	do
	{
		x = modelDims.get_x() * ((float) rd() / (float) rd.max() - 0.5)
				- modelDeltaCenter.get_x() + modelNP.get_x();
		y = modelDims.get_y() * ((float) rd() / (float) rd.max() - 0.5)
				- modelDeltaCenter.get_y() + modelNP.get_y();
		gotCollisionZ = steerMgr->get_collision_height(LPoint3f(x, y, zOrig));

	} while (not gotCollisionZ.get_first());
	return LPoint3f(x, y, gotCollisionZ.get_second());
}

// handle add/remove obstacles
void handleVehicles(const Event* e, void* data)
{
	if (not data)
	{
		return;
	}

	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		// get the hit object
		NodePath hitObject = entry0->get_into_node_path();
		cout << "hit " << hitObject << " object" << endl;

		NodePath sceneNP = reinterpret_cast<HandleVehicleData*>(data)->sceneNP;
		// check if sceneNP is the hitObject or an ancestor thereof
		if ((sceneNP == hitObject) or sceneNP.is_ancestor_of(hitObject))
		{
			// the hit object is the scene: add an vehicle to the scene
			float meanScale = reinterpret_cast<HandleVehicleData*>(data)->meanScale;
			int vehicleFileIdx = reinterpret_cast<HandleVehicleData*>(data)->vehicleFileIdx;
			string moveType = reinterpret_cast<HandleVehicleData*>(data)->moveType;
			vector<NodePath>& vehicleNP = reinterpret_cast<HandleVehicleData*>(data)->vehicleNP;
			PT(OSSteerPlugIn)steerPlugIn = reinterpret_cast<HandleVehicleData*>(data)->steerPlugIn;
			vector<PT(OSSteerVehicle)>&steerVehicle = reinterpret_cast<HandleVehicleData*>(data)->steerVehicle;
			vector<vector<PT(AnimControl)> >&vehicleAnimCtls = reinterpret_cast<HandleVehicleData*>(data)->vehicleAnimCtls;
			// add vehicle
			LPoint3f pos = entry0->get_surface_point(NodePath());
			getVehicleModelAnims(meanScale, vehicleFileIdx, moveType, sceneNP,
					vehicleNP, steerPlugIn, steerVehicle,
					vehicleAnimCtls, pos);
			// show the added vehicles
			cout << "Vehicles added to plug-in so far:" << endl;
			for (int i = 0; i < steerPlugIn->get_num_steer_vehicles(); ++i)
			{
				cout << "\t- " << *((*steerPlugIn)[i]) << endl;
			}
		}
	}
}

// get a vehicle, model and animations
void getVehicleModelAnims(float meanScale, int vehicleFileIdx, const string& moveType,
		const NodePath& sceneNP, vector<NodePath>& vehicleNP, PT(OSSteerPlugIn)steerPlugIn,
vector<PT(OSSteerVehicle)>&steerVehicle, vector<vector<PT(AnimControl)> >& vehicleAnimCtls,
	const LPoint3f& pos)
{
	// get some models, with animations, to attach to vehicles
	// get the model
	vehicleNP.push_back(window->load_model(framework.get_models(), vehicleFile[vehicleFileIdx]));
	// set random scale (0.35 - 0.45)
	float scale = meanScale + 0.1 * ((float) rd() / (float) rd.max());
	vehicleNP.back().set_scale(scale);
	// associate an anim with a given anim control
	AnimControlCollection tmpAnims;
	NodePath vehicleAnimNP[2];
	// first anim -> modelAnimCtls[i][0]
	vehicleAnimNP[0] = window->load_model(vehicleNP.back(), vehicleAnimFiles[vehicleFileIdx][0]);
	auto_bind(vehicleNP.back().node(), tmpAnims);
	vehicleAnimCtls.push_back(vector<PT(AnimControl)>(2));
	vehicleAnimCtls.back()[0] = tmpAnims.get_anim(0);
	tmpAnims.clear_anims();
	vehicleAnimNP[0].detach_node();
	// second anim -> modelAnimCtls[i][1]
	vehicleAnimNP[1] = window->load_model(vehicleNP.back(), vehicleAnimFiles[vehicleFileIdx][1]);
	auto_bind(vehicleNP.back().node(), tmpAnims);
	vehicleAnimCtls.back()[1] = tmpAnims.get_anim(0);
	tmpAnims.clear_anims();
	vehicleAnimNP[1].detach_node();
	// reparent all node paths
	vehicleAnimNP[0].reparent_to(vehicleNP.back());
	vehicleAnimNP[1].reparent_to(vehicleNP.back());
	// set parameter for vehicle's move type (OPENSTEER or OPENSTEER_KINEMATIC)
	WPT(OSSteerManager) steerMgr = OSSteerManager::get_global_ptr();
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "mov_type",
	moveType);
	// create the steer vehicle (attached to the reference node)
	NodePath steerVehicleNP = steerMgr->create_steer_vehicle("vehicle" + str(vehicleNP.size() - 1));
	steerVehicle.push_back(DCAST(OSSteerVehicle, steerVehicleNP.node()));
	// set the position randomly
	LPoint3f randPos = pos;
	if (randPos == LPoint3f::zero())
	{
		randPos = getRandomPos(sceneNP);
	}
	steerVehicleNP.set_pos(randPos);
	// attach some geometry (a model) to steer vehicle
	vehicleNP.back().reparent_to(steerVehicleNP);
	// add the steer vehicle to the plug-in
	steerPlugIn->add_steer_vehicle(steerVehicleNP);
}

// read scene from a file
bool readFromBamFile(string fileName)
{
	return OSSteerManager::get_global_ptr()->read_from_bam_file(fileName);
}

// write scene to a file (and exit)
void writeToBamFileAndExit(const Event* e, void* data)
{
	string fileName = *reinterpret_cast<string*>(data);
	OSSteerManager::get_global_ptr()->write_to_bam_file(fileName);
	/// second option: remove custom update updateTask
	framework.get_task_mgr().remove(updateTask);
	// delete steer manager
	delete OSSteerManager::get_global_ptr();
	// close the window framework
	framework.close_framework();
	//
	exit(0);
}

// handle add/remove obstacles
void handleObstacles(const Event* e, void* data)
{
	bool addObstacle = reinterpret_cast<HandleObstacleData*>(data)->addObstacle;
	NodePath sceneNP = reinterpret_cast<HandleObstacleData*>(data)->sceneNP;
	PT(OSSteerPlugIn)steerPlugIn =
			reinterpret_cast<HandleObstacleData*>(data)->steerPlugIn;
	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		// get the hit object
		NodePath hitObject = entry0->get_into_node_path();
		cout << "hit " << hitObject << " object" << endl;

		// check if we want add obstacle and
		// if sceneNP is the hitObject or an ancestor thereof
		if (addObstacle
				and ((sceneNP == hitObject) or sceneNP.is_ancestor_of(hitObject)))
		{
			// the hit object is the scene: add an obstacle to the scene
			// get a model as obstacle
			NodePath obstacleNP = window->load_model(framework.get_models(),
					obstacleFile);
			obstacleNP.set_collide_mask(mask);
			// set random scale (0.03 - 0.04)
			float scale = 0.03 + 0.01 * ((float) rd() / (float) rd.max());
			obstacleNP.set_scale(scale);
			// set obstacle position
			LPoint3f pos = entry0->get_surface_point(sceneNP);
			obstacleNP.set_pos(sceneNP, pos);
			// try to add to plug-in
			if (steerPlugIn->add_obstacle(obstacleNP, "box") < 0)
			{
				// something went wrong remove from scene
				obstacleNP.remove_node();
				return;
			}
			cout << "added " << obstacleNP << " obstacle." << endl;
		}
		// check if we want remove obstacle
		else if (not addObstacle)
		{
			// cycle through the local obstacle list
			for (int index = 0; index < steerPlugIn->get_num_obstacles();
					++index)
			{
				// get the obstacle's NodePath
				int ref = steerPlugIn->get_obstacle(index);
				NodePath obstacleNP =
						OSSteerManager::get_global_ptr()->get_obstacle_by_ref(
								ref);
				// check if obstacleNP is the hitObject or an ancestor thereof
				if ((obstacleNP == hitObject)
						or obstacleNP.is_ancestor_of(hitObject))
				{
					// try to remove from plug-in
					if (not steerPlugIn->remove_obstacle(ref).is_empty())
					{
						// all ok remove from scene
						cout << "removed " << obstacleNP << " obstacle."
								<< endl;
						obstacleNP.remove_node();
						break;
					}
				}
			}
		}
	}
}
