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
string vehicleFile[5] =
{ "eve.egg", "ralph.egg", "sparrow.egg", "ball.egg", "red_car.egg" };
string vehicleAnimFiles[5][2] =
{
{ "eve-walk.egg", "eve-run.egg" },
{ "ralph-walk.egg", "ralph-run.egg" },
{ "sparrow-flying.egg", "sparrow-flying2.egg" },
{ "", "" },
{ "red_car-anim.egg", "red_car-anim2.egg" }};
const float animRateFactor[2] =
{ 1.20, 4.80 };
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
NodePath loadPlane(const string& name, float widthX, float widthY)
{
	CardMaker cm("plane");
	cm.set_frame(-widthX / 2.0, widthX / 2.0, -widthY / 2.0, widthY / 2.0);
	NodePath plane(cm.generate());
	plane.set_p(-90.0);
	plane.set_z(0.0);
	plane.set_color(0.15, 0.35, 0.35);
	plane.set_collide_mask(mask);
	plane.set_name(name);
	return plane;
}

// load terrain low poly stuff
NodePath loadTerrainLowPoly(const string& name, float widthScale,
		float heightScale, const string& texture)
{
	NodePath terrainNP = window->load_model(framework.get_models(),
			"terrain-low-poly.egg");
	terrainNP.set_name(name);
	terrainNP.set_scale(widthScale, widthScale, heightScale);
	PT(Texture)tex =
	TexturePool::load_texture(Filename(texture));
	terrainNP.set_texture(tex);
	return terrainNP;
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
NodePath loadTerrain(const string& name, float widthScale, float heightScale)
{
	GeoMipTerrain *terrain = new GeoMipTerrain("terrain");
	PNMImage heightField(Filename(dataDir + string("/heightfield.png")));
	terrain->set_heightfield(heightField);
	//sizing
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

// handle add vehicles
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

		HandleVehicleData* vehicleData =
				reinterpret_cast<HandleVehicleData*>(data);
		NodePath sceneNP = vehicleData->sceneNP;
		// check if sceneNP is the hitObject or an ancestor thereof
		if ((sceneNP == hitObject) or sceneNP.is_ancestor_of(hitObject))
		{
			// the hit object is the scene: add an vehicle to the scene
			float meanScale = vehicleData->meanScale;
			int vehicleFileIdx = vehicleData->vehicleFileIdx;
			string moveType = vehicleData->moveType;
			PT(OSSteerPlugIn)steerPlugIn = vehicleData->steerPlugIn;
			vector<PT(OSSteerVehicle)>&steerVehicles = vehicleData->steerVehicles;
			vector<vector<PT(AnimControl)> >&vehicleAnimCtls = vehicleData->vehicleAnimCtls;
			LVector3f deltaPos = vehicleData->deltaPos;
			// add vehicle
			LPoint3f pos = entry0->get_surface_point(NodePath()) + deltaPos;
			getVehicleModelAnims(meanScale, vehicleFileIdx, moveType, sceneNP,
					steerPlugIn, steerVehicles, vehicleAnimCtls, pos);
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
		const NodePath& sceneNP, PT(OSSteerPlugIn)steerPlugIn,
vector<PT(OSSteerVehicle)>&steerVehicles, vector<vector<PT(AnimControl)> >& vehicleAnimCtls,
	const LPoint3f& pos)
{
	// get some models, with animations, to attach to vehicles
	// get the model
	NodePath vehicleNP = window->load_model(framework.get_models(), vehicleFile[vehicleFileIdx]);
	// set random scale
	float scale = meanScale * (1 + 0.2 * (2 * (float) rd() / (float) rd.max() - 1));
	vehicleNP.set_scale(scale);
	// associate an anim with a given anim control
	AnimControlCollection tmpAnims;
	NodePath vehicleAnimNP[2];
	vehicleAnimCtls.push_back(vector<PT(AnimControl)>(2));
	if((!vehicleAnimFiles[vehicleFileIdx][0].empty()) &&
			(!vehicleAnimFiles[vehicleFileIdx][1].empty()))
	{
		// first anim -> modelAnimCtls[i][0]
		vehicleAnimNP[0] = window->load_model(vehicleNP, vehicleAnimFiles[vehicleFileIdx][0]);
		auto_bind(vehicleNP.node(), tmpAnims,
                PartGroup::HMF_ok_part_extra |
                PartGroup::HMF_ok_anim_extra |
                PartGroup::HMF_ok_wrong_root_name);
		vehicleAnimCtls.back()[0] = tmpAnims.get_anim(0);
		tmpAnims.clear_anims();
		vehicleAnimNP[0].detach_node();
		// second anim -> modelAnimCtls[i][1]
		vehicleAnimNP[1] = window->load_model(vehicleNP, vehicleAnimFiles[vehicleFileIdx][1]);
		auto_bind(vehicleNP.node(), tmpAnims,
                PartGroup::HMF_ok_part_extra |
                PartGroup::HMF_ok_anim_extra |
                PartGroup::HMF_ok_wrong_root_name);
		vehicleAnimCtls.back()[1] = tmpAnims.get_anim(0);
		tmpAnims.clear_anims();
		vehicleAnimNP[1].detach_node();
		// reparent all node paths
		vehicleAnimNP[0].reparent_to(vehicleNP);
		vehicleAnimNP[1].reparent_to(vehicleNP);
	}
	// set parameter for vehicle's move type (OPENSTEER or OPENSTEER_KINEMATIC)
	WPT(OSSteerManager) steerMgr = OSSteerManager::get_global_ptr();
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "mov_type",
	moveType);
	// create the steer vehicle (attached to the reference node)
	NodePath steerVehicleNP = steerMgr->create_steer_vehicle("vehicle" + str(steerVehicles.size()));
	steerVehicles.push_back(DCAST(OSSteerVehicle, steerVehicleNP.node()));
	// set the position randomly
	LPoint3f randPos = pos;
	if (randPos == LPoint3f::zero())
	{
		randPos = getRandomPos(sceneNP);
	}
	steerVehicleNP.set_pos(randPos);
	// attach some geometry (a model) to steer vehicle
	vehicleNP.reparent_to(steerVehicleNP);
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
	HandleObstacleData* obstacleData = reinterpret_cast<HandleObstacleData*>(data);
	bool addObstacle = obstacleData->addObstacle;
	NodePath sceneNP = obstacleData->sceneNP;
	LVecBase3f scale = obstacleData->scale;
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
			scale += scale * 0.2 * (2 * (float) rd() / (float) rd.max() - 1);
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
			cout << "added " << obstacleNP << " obstacle at " <<
					obstacleNP.get_pos() << endl;
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

// class Driver
Driver::Driver(PandaFramework* framework, const NodePath& ownerObjectNP,
		int taskSort)
{
	mWin = framework->get_window(0)->get_graphics_window();
	mOwnerObjectNP = ownerObjectNP;
	mTaskSort = taskSort;
	do_reset();
	do_initialize();
}

Driver::~Driver()
{
	do_finalize();
	do_reset();
	mOwnerObjectNP.clear();
	mWin = NULL;
}

void Driver::do_reset()
{
	//
	mEnabled = false;
	mForward = mBackward = mStrafeLeft = mStrafeRight = mUp = mDown =
			mHeadLeft = mHeadRight = mPitchUp = mPitchDown = false;
	//by default we consider mouse moved on every update, because
	//we want mouse poll by default; this can be changed by calling
	//the enabler (for example by an handler responding to mouse-move
	//event if it is possible. See: http://www.panda3d.org/forums/viewtopic.php?t=9326
	// http://www.panda3d.org/forums/viewtopic.php?t=6049)
	mMouseMove = true;
	mForwardKey = mBackwardKey = mStrafeLeftKey = mStrafeRightKey = mUpKey = mDownKey =
			mHeadLeftKey = mHeadRightKey = mPitchUpKey = mPitchDownKey =
					mMouseMoveKey = false;
	mSpeedKey = std::string("shift");
	mMouseEnabledH = mMouseEnabledP = mHeadLimitEnabled = mPitchLimitEnabled =
			false;
	mHLimit = mPLimit = 0.0;
	mSignOfTranslation = mSignOfMouse = 1;
	mFastFactor = 0.0;
	mActualSpeedXYZ = mMaxSpeedXYZ = mMaxSpeedSquaredXYZ = LVecBase3f::zero();
	mActualSpeedH = mActualSpeedP = mMaxSpeedHP = mMaxSpeedSquaredHP = 0.0;
	mAccelXYZ = LVecBase3f::zero();
	mAccelHP = 0.0;
	mFrictionXYZ = mFrictionHP = 0.0;
	mStopThreshold = 0.0;
	mSensX = mSensY = 0.0;
	mCentX = mCentY = 0.0;
	mUpdateData.clear();
	mUpdateTask.clear();
}

void Driver::do_initialize()
{
	//inverted setting (1/-1): not inverted -> 1, inverted -> -1
	mSignOfTranslation = 1;
	mSignOfMouse = 1;
	//head limit: enabled@[limit]; limit >= 0.0
	mHeadLimitEnabled = false;
	mHLimit = 0.0;
	//pitch limit: enabled@[limit]; limit >= 0.0
	mPitchLimitEnabled = false;
	mPLimit = 0.0;
	//mouse movement setting
	mMouseEnabledH = false;
	mMouseEnabledP = false;
	//key events setting
	//backward key
	mBackwardKey = true;
	//down key
	mDownKey = true;
	//forward key
	mForwardKey = true;
	//strafeLeft key
	mStrafeLeftKey = true;
	//strafeRight key
	mStrafeRightKey = true;
	//headLeft key
	mHeadLeftKey = true;
	//headRight key
	mHeadRightKey = true;
	//pitchUp key
	mPitchUpKey = true;
	//pitchDown key
	mPitchDownKey = true;
	//up key
	mUpKey = true;
	//mouseMove key: enabled/disabled
	mMouseMoveKey = false;
	//speedKey
	if (not (mSpeedKey == std::string("control")
			or mSpeedKey == std::string("alt")
			or mSpeedKey == std::string("shift")))
	{
		mSpeedKey = std::string("shift");
	}
	//
	//max linear speed (>=0)
	mMaxSpeedXYZ = LVecBase3f(5.0, 5.0, 5.0);
	mMaxSpeedSquaredXYZ = LVector3f(mMaxSpeedXYZ.get_x() * mMaxSpeedXYZ.get_x(),
			mMaxSpeedXYZ.get_y() * mMaxSpeedXYZ.get_y(),
			mMaxSpeedXYZ.get_z() * mMaxSpeedXYZ.get_z());
	//max angular speed (>=0)
	mMaxSpeedHP = 5.0;
	mMaxSpeedSquaredHP = mMaxSpeedHP * mMaxSpeedHP;
	//linear accel (>=0)
	mAccelXYZ = LVecBase3f(5.0, 5.0, 5.0);
	//angular accel (>=0)
	mAccelHP = 5.0;
	//reset actual speeds
	mActualSpeedXYZ = LVector3f::zero();
	mActualSpeedH = 0.0;
	mActualSpeedP = 0.0;
	//linear friction (>=0)
	mFrictionXYZ = 5.0;
	//angular friction (>=0)
	mFrictionHP = 5.0;
	//stop threshold ([0.0, 1.0])
	mStopThreshold = 0.01;
	//fast factor (>=0)
	mFastFactor = 5.0;
	//sens x (>=0)
	mSensX = 0.2;
	//sens_y (>=0)
	mSensY = 0.2;
	//create the task for updating Driver
	mUpdateData = new TaskInterface<Driver>::TaskData(this,
			&Driver::update);
	mUpdateTask = new GenericAsyncTask(string("Driver::update"),
			&TaskInterface<Driver>::taskFunction,
			reinterpret_cast<void*>(mUpdateData.p()));
	mUpdateTask->set_sort(mTaskSort);
	//
	mCentX = mWin->get_properties().get_x_size() / 2;
	mCentY = mWin->get_properties().get_y_size() / 2;
}

void Driver::do_finalize()
{
	//if enabled: disable
	if (mEnabled)
	{
		//actual disabling
		do_disable();
	}
	//
	return;
}

bool Driver::enable()
{
	//if enabled return
	RETURN_ON_COND(mEnabled, false)

	//actual enabling
	do_enable();
	//
	return true;
}

void Driver::do_enable()
{
	if (mMouseEnabledH or mMouseEnabledP or mMouseMoveKey)
	{
		//we want control through mouse movements
		//hide mouse cursor
		WindowProperties props;
		props.set_cursor_hidden(true);
		mWin->request_properties(props);
		//reset mouse to start position
		mWin->move_pointer(0, mCentX, mCentY);
	}
	//
	mEnabled = true;

	//Add mUpdateTask to the active queue.
	if (mUpdateTask)
	{
		AsyncTaskManager::get_global_ptr()->add(mUpdateTask);
	}
}

bool Driver::disable()
{
	//if not enabled return
	RETURN_ON_COND(not mEnabled, false)

	//actual disabling
	do_disable();
	//
	return true;
}

void Driver::do_disable()
{
	if (mMouseEnabledH or mMouseEnabledP or mMouseMoveKey)
	{
		//we have control through mouse movements
		//show mouse cursor
		WindowProperties props;
		props.set_cursor_hidden(false);
		mWin->request_properties(props);
	}
	//
	mEnabled = false;

	//Remove mUpdateTask from the active queue.
	if (mUpdateTask)
	{
		AsyncTaskManager::get_global_ptr()->remove(mUpdateTask);
	}
}

AsyncTask::DoneStatus Driver::update(GenericAsyncTask* task)
{
	float dt = ClockObject::get_global_clock()->get_dt();

	//handle mouse
	if (mMouseMove and (mMouseEnabledH or mMouseEnabledP))
	{
		MouseData md = mWin->get_pointer(0);
		float deltaX = md.get_x() - mCentX;
		float deltaY = md.get_y() - mCentY;

		if (mWin->move_pointer(0, mCentX, mCentY))
		{
			if (mMouseEnabledH and (deltaX != 0.0))
			{
				mOwnerObjectNP.set_h(
						mOwnerObjectNP.get_h() - deltaX * mSensX * mSignOfMouse);
			}
			if (mMouseEnabledP and (deltaY != 0.0))
			{
				mOwnerObjectNP.set_p(
						mOwnerObjectNP.get_p() - deltaY * mSensY * mSignOfMouse);
			}
		}
		//if mMouseMoveKey is true we are controlling mouse movements
		//so we need to reset mMouseMove to false
		if (mMouseMoveKey)
		{
			mMouseMove = false;
		}
	}
	//update position/orientation
	mOwnerObjectNP.set_y(mOwnerObjectNP,
			mActualSpeedXYZ.get_y() * dt * mSignOfTranslation);
	mOwnerObjectNP.set_x(mOwnerObjectNP,
			mActualSpeedXYZ.get_x() * dt * mSignOfTranslation);
	mOwnerObjectNP.set_z(mOwnerObjectNP, mActualSpeedXYZ.get_z() * dt);
	//head
	if (mHeadLimitEnabled)
	{
		float head = mOwnerObjectNP.get_h() + mActualSpeedH * dt * mSignOfMouse;
		if (head > mHLimit)
		{
			head = mHLimit;
		}
		else if (head < -mHLimit)
		{
			head = -mHLimit;
		}
		mOwnerObjectNP.set_h(head);
	}
	else
	{
		mOwnerObjectNP.set_h(
				mOwnerObjectNP.get_h() + mActualSpeedH * dt * mSignOfMouse);
	}
	//pitch
	if (mPitchLimitEnabled)
	{
		float pitch = mOwnerObjectNP.get_p() + mActualSpeedP * dt * mSignOfMouse;
		if (pitch > mPLimit)
		{
			pitch = mPLimit;
		}
		else if (pitch < -mPLimit)
		{
			pitch = -mPLimit;
		}
		mOwnerObjectNP.set_p(pitch);
	}
	else
	{
		mOwnerObjectNP.set_p(
				mOwnerObjectNP.get_p() + mActualSpeedP * dt * mSignOfMouse);
	}

	//update speeds
	//y axis
	if (mForward and (not mBackward))
	{
		if (mAccelXYZ.get_y() != 0.0)
		{
			//accelerate
			mActualSpeedXYZ.set_y(
					mActualSpeedXYZ.get_y() - mAccelXYZ.get_y() * dt);
			if (mActualSpeedXYZ.get_y() < -mMaxSpeedXYZ.get_y())
			{
				//limit speed
				mActualSpeedXYZ.set_y(-mMaxSpeedXYZ.get_y());
			}
		}
		else
		{
			//kinematic
			mActualSpeedXYZ.set_y(-mMaxSpeedXYZ.get_y());
		}
	}
	else if (mBackward and (not mForward))
	{
		if (mAccelXYZ.get_y() != 0.0)
		{
			//accelerate
			mActualSpeedXYZ.set_y(
					mActualSpeedXYZ.get_y() + mAccelXYZ.get_y() * dt);
			if (mActualSpeedXYZ.get_y() > mMaxSpeedXYZ.get_y())
			{
				//limit speed
				mActualSpeedXYZ.set_y(mMaxSpeedXYZ.get_y());
			}
		}
		else
		{
			//kinematic
			mActualSpeedXYZ.set_y(mMaxSpeedXYZ.get_y());
		}
	}
	else if (mActualSpeedXYZ.get_y() != 0.0)
	{
		if (mActualSpeedXYZ.get_y() * mActualSpeedXYZ.get_y()
				< mMaxSpeedSquaredXYZ.get_y() * mStopThreshold)
		{
			//stop
			mActualSpeedXYZ.set_y(0.0);
		}
		else
		{
			//decelerate
			mActualSpeedXYZ.set_y(
					mActualSpeedXYZ.get_y() * (1.0 - min(mFrictionXYZ * dt, 1.0f)));
		}
	}
	//x axis
	if (mStrafeLeft and (not mStrafeRight))
	{
		if (mAccelXYZ.get_x() != 0.0)
		{
			//accelerate
			mActualSpeedXYZ.set_x(
					mActualSpeedXYZ.get_x() + mAccelXYZ.get_x() * dt);
			if (mActualSpeedXYZ.get_x() > mMaxSpeedXYZ.get_x())
			{
				//limit speed
				mActualSpeedXYZ.set_x(mMaxSpeedXYZ.get_x());
			}
		}
		else
		{
			//kinematic
			mActualSpeedXYZ.set_x(mMaxSpeedXYZ.get_x());
		}
	}
	else if (mStrafeRight and (not mStrafeLeft))
	{
		if (mAccelXYZ.get_x() != 0.0)
		{
			//accelerate
			mActualSpeedXYZ.set_x(
					mActualSpeedXYZ.get_x() - mAccelXYZ.get_x() * dt);
			if (mActualSpeedXYZ.get_x() < -mMaxSpeedXYZ.get_x())
			{
				//limit speed
				mActualSpeedXYZ.set_x(-mMaxSpeedXYZ.get_x());
			}
		}
		else
		{
			//kinematic
			mActualSpeedXYZ.set_x(-mMaxSpeedXYZ.get_y());
		}
	}
	else if (mActualSpeedXYZ.get_x() != 0.0)
	{
		if (mActualSpeedXYZ.get_x() * mActualSpeedXYZ.get_x()
				< mMaxSpeedSquaredXYZ.get_x() * mStopThreshold)
		{
			//stop
			mActualSpeedXYZ.set_x(0.0);
		}
		else
		{
			//decelerate
			mActualSpeedXYZ.set_x(
					mActualSpeedXYZ.get_x() * (1.0 - min(mFrictionXYZ * dt, 1.0f)));
		}
	}
	//z axis
	if (mUp and (not mDown))
	{
		if (mAccelXYZ.get_z() != 0.0)
		{
			//accelerate
			mActualSpeedXYZ.set_z(
					mActualSpeedXYZ.get_z() + mAccelXYZ.get_z() * dt);
			if (mActualSpeedXYZ.get_z() > mMaxSpeedXYZ.get_z())
			{
				//limit speed
				mActualSpeedXYZ.set_z(mMaxSpeedXYZ.get_z());
			}
		}
		else
		{
			//kinematic
			mActualSpeedXYZ.set_z(mMaxSpeedXYZ.get_z());
		}
	}
	else if (mDown and (not mUp))
	{
		if (mAccelXYZ.get_z() != 0.0)
		{
			//accelerate
			mActualSpeedXYZ.set_z(
					mActualSpeedXYZ.get_z() - mAccelXYZ.get_z() * dt);
			if (mActualSpeedXYZ.get_z() < -mMaxSpeedXYZ.get_z())
			{
				//limit speed
				mActualSpeedXYZ.set_z(-mMaxSpeedXYZ.get_z());
			}
		}
		else
		{
			//kinematic
			mActualSpeedXYZ.set_z(-mMaxSpeedXYZ.get_z());
		}
	}
	else if (mActualSpeedXYZ.get_z() != 0.0)
	{
		if (mActualSpeedXYZ.get_z() * mActualSpeedXYZ.get_z()
				< mMaxSpeedSquaredXYZ.get_z() * mStopThreshold)
		{
			//stop
			mActualSpeedXYZ.set_z(0.0);
		}
		else
		{
			//decelerate
			mActualSpeedXYZ.set_z(
					mActualSpeedXYZ.get_z() * (1.0 - min(mFrictionXYZ * dt, 1.0f)));
		}
	}
	//rotation h
	if (mHeadLeft and (not mHeadRight))
	{
		if (mAccelHP != 0.0)
		{
			//accelerate
			mActualSpeedH += mAccelHP * dt;
			if (mActualSpeedH > mMaxSpeedHP)
			{
				//limit speed
				mActualSpeedH = mMaxSpeedHP;
			}
		}
		else
		{
			//kinematic
			mActualSpeedH = mMaxSpeedHP;
		}
	}
	else if (mHeadRight and (not mHeadLeft))
	{
		if (mAccelHP != 0.0)
		{
			//accelerate
			mActualSpeedH -= mAccelHP * dt;
			if (mActualSpeedH < -mMaxSpeedHP)
			{
				//limit speed
				mActualSpeedH = -mMaxSpeedHP;
			}
		}
		else
		{
			//kinematic
			mActualSpeedH = -mMaxSpeedHP;
		}
	}
	else if (mActualSpeedH != 0.0)
	{
		if (mActualSpeedH * mActualSpeedH < mMaxSpeedSquaredHP * mStopThreshold)
		{
			//stop
			mActualSpeedH = 0.0;
		}
		else
		{
			//decelerate
            mActualSpeedH = mActualSpeedH * (1.0 - min(mFrictionHP * dt, 1.0f));
		}
	}
	//rotation p
	if (mPitchUp and (not mPitchDown))
	{
		if (mAccelHP != 0.0)
		{
			//accelerate
			mActualSpeedP += mAccelHP * dt;
			if (mActualSpeedP > mMaxSpeedHP)
			{
				//limit speed
				mActualSpeedP = mMaxSpeedHP;
			}
		}
		else
		{
			//kinematic
			mActualSpeedP = mMaxSpeedHP;
		}
	}
	else if (mPitchDown and (not mPitchUp))
	{
		if (mAccelHP != 0.0)
		{
			//accelerate
			mActualSpeedP -= mAccelHP * dt;
			if (mActualSpeedP < -mMaxSpeedHP)
			{
				//limit speed
				mActualSpeedP = -mMaxSpeedHP;
			}
		}
		else
		{
			//kinematic
			mActualSpeedP = -mMaxSpeedHP;
		}
	}
	else if (mActualSpeedP != 0.0)
	{
		if (mActualSpeedP * mActualSpeedP < mMaxSpeedSquaredHP * mStopThreshold)
		{
			//stop
			mActualSpeedP = 0.0;
		}
		else
		{
			//decelerate
			mActualSpeedP = mActualSpeedP * (1.0 - min(mFrictionHP * dt, 1.0f));
		}
	}
	//
	return AsyncTask::DS_cont;
}
