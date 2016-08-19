/**
 * \file pedestrian_external.cpp
 *
 * \date 2016-08-19
 * \author consultit
 */

#include "common.h"

///specific data/functions declarations/definitions
NodePath sceneNP;
vector<vector<PT(AnimControl)> > vehicleAnimCtls;
PT(OSSteerPlugIn)steerPlugIn;
vector<PT(OSSteerVehicle)>steerVehicles;
NodePath playerNP;
//
void setParametersBeforeCreation();
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask*, void*);
NodePath getPlayerModelAnims(const string&, float, int, PT(OSSteerPlugIn),
		vector<PT(OSSteerVehicle)>&, vector<vector<PT(AnimControl)> >&,
		const LPoint3f& );
AsyncTask::DoneStatus updatePlayer(GenericAsyncTask*, void*);

int main(int argc, char *argv[])
{
	string msg("'pedestrian external'");
	startFramework(argc, argv, msg);

	/// here is room for your own code
	// print some help to screen
	PT(TextNode)text;
	text = new TextNode("Help");
	text->set_text(
            msg + "\n\n"
            "- press \"d\" to toggle debug drawing\n"
			"- press \"a\"/\"k\" to add 'opensteer'/'kinematic' vehicle\n"
            "- press \"s\"/\"shift-s\" to increase/decrease last inserted vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease last inserted vehicle's max force\n"
            "- press \"t\" to toggle last inserted vehicle's wander behavior\n"
			"- press \"o\"/\"shift-o\" to add/remove obstacle\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(-1.25, 0.0, 0.8);
	textNodePath.set_scale(0.035);

	// create a steer manager; set root and mask to manage 'kinematic' vehicles
	WPT(OSSteerManager)steerMgr = new OSSteerManager(window->get_render(), mask);

	// print creation parameters: defult values
	cout << endl << "Default creation parameters:";
	printCreationParameters();

	// load or restore all scene stuff: if passed an argument
	// try to read it from bam file
	if ((not (argc > 1)) or (not readFromBamFile(argv[1])))
	{
		// no argument or no valid bamFile
		// reparent the reference node to render
		steerMgr->get_reference_node_path().reparent_to(window->get_render());

		// get a sceneNP, naming it with "SceneNP" to ease restoring from bam
		// file
		sceneNP = loadTerrain("SceneNP");
		// and reparent to the reference node
		sceneNP.reparent_to(steerMgr->get_reference_node_path());

		// set sceneNP's collide mask
		sceneNP.set_collide_mask(mask);

		// set plug-in type and create it (attached to the reference node)
		steerMgr->set_parameter_value(OSSteerManager::STEERPLUGIN,
				"plugin_type", "pedestrian");
		NodePath plugInNP = steerMgr->create_steer_plug_in();
		steerPlugIn = DCAST(OSSteerPlugIn, plugInNP.node());

		// set player's creation parameters as string: type and externally updated
		steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE,
				"vehicle_type", "pedestrian");
		steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE,
				"external_update", "true");
		// add the player and set a reference to it
		playerNP = getPlayerModelAnims("PlayerNP", 0.5, 0, steerPlugIn,
				steerVehicles, vehicleAnimCtls,
				LPoint3f(79.474, 51.7236, 2.0207));

		// set remaining creation parameters as strings before
		// the other vehicles' creation
		cout << endl << "Current creation parameters:";
		setParametersBeforeCreation();

		// set the pathway
		ValueList<LPoint3f> pointList;
		pointList.add_value(LPoint3f(79.474, 51.7236, 2.0207));
		pointList.add_value(LPoint3f(108.071, 51.1972, 2.7246));
		pointList.add_value(LPoint3f(129.699, 30.1742, 0.720501));
		pointList.add_value(LPoint3f(141.597, 73.496, 2.14218));
		pointList.add_value(LPoint3f(105.917, 107.032, 3.06428));
		pointList.add_value(LPoint3f(61.2637, 109.622, 3.03588));
		// use single radius pathway
		ValueList<float> radiusList;
		radiusList.add_value(4);
		steerPlugIn->set_pathway(pointList, radiusList, true, true);
	}
	else
	{
		// valid bamFile
		// restore plug-in: through steer manager
		NodePath steerPlugInNP =
				OSSteerManager::get_global_ptr()->get_steer_plug_in(0);
		steerPlugIn = DCAST(OSSteerPlugIn, steerPlugInNP.node());
		// restore sceneNP: through panda3d
		sceneNP =
				OSSteerManager::get_global_ptr()->get_reference_node_path().find(
						"**/SceneNP");
		// reparent the reference node to render
		OSSteerManager::get_global_ptr()->get_reference_node_path().reparent_to(
				window->get_render());

		// restore the player's reference
		playerNP = OSSteerManager::get_global_ptr()->get_reference_node_path().find(
				"**/PlayerNP");

		// restore all steer vehicles (including the player)
		int NUMVEHICLES =
				OSSteerManager::get_global_ptr()->get_num_steer_vehicles();
		steerVehicles.resize(NUMVEHICLES);
		vehicleAnimCtls.resize(NUMVEHICLES);
		for (int i = 0; i < NUMVEHICLES; ++i)
		{
			// restore the steer vehicle: through steer manager
			NodePath steerVehicleNP =
					OSSteerManager::get_global_ptr()->get_steer_vehicle(i);
			steerVehicles[i] = DCAST(OSSteerVehicle, steerVehicleNP.node());
			// restore animations
			AnimControlCollection tmpAnims;
			auto_bind(steerVehicles[i], tmpAnims);
			vehicleAnimCtls[i] = vector<PT(AnimControl)>(2);
			for (int j = 0; j < tmpAnims.get_num_anims(); ++j)
			{
				vehicleAnimCtls[i][j] = tmpAnims.get_anim(j);
			}
		}

		// set creation parameters as strings before other plug-ins/vehicles creation
		cout << endl << "Current creation parameters:";
        steerMgr->set_parameter_value(OSSteerManager::STEERPLUGIN, "plugin_type",
                "pedestrian");
        steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE,
                "vehicle_type", "pedestrian");
		setParametersBeforeCreation();
	}

	/// first option: start the default update task for all plug-ins
///	steerMgr->start_default_update();

/// second option: start the custom update task for all plug-ins
	updateTask = new GenericAsyncTask("updatePlugIn", &updatePlugIn,
			(void*) steerPlugIn.p());
	framework.get_task_mgr().add(updateTask);

	// DEBUG DRAWING: make the debug reference node paths sibling of the reference node
	steerMgr->get_reference_node_path_debug().reparent_to(window->get_render());
	steerMgr->get_reference_node_path_debug_2d().reparent_to(
			window->get_aspect_2d());
	// enable debug drawing
	steerPlugIn->enable_debug_drawing(window->get_camera_group());

	/// set events' callbacks
	// toggle debug draw
	toggleDebugFlag = false;
	framework.define_key("d", "toggleDebugDraw", &toggleDebugDraw,
			(void*) steerPlugIn.p());

	// handle addition steer vehicles, models and animations
	HandleVehicleData vehicleData(0.7, 0, "opensteer", sceneNP,
						steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("a", "addVehicle", &handleVehicles,
			(void*) &vehicleData);
	HandleVehicleData vehicleDataKinematic(0.7, 1, "kinematic", sceneNP,
			steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("k", "addVehicle", &handleVehicles,
			(void*) &vehicleDataKinematic);

	// handle obstacle addition
	HandleObstacleData obstacleAddition(true, sceneNP, steerPlugIn,
			LVecBase3f(0.03, 0.03, 0.03));
	framework.define_key("o", "addObstacle", &handleObstacles,
			(void*) &obstacleAddition);
	// handle obstacle removal
	HandleObstacleData obstacleRemoval(false, sceneNP, steerPlugIn);
	framework.define_key("shift-o", "removeObstacle", &handleObstacles,
			(void*) &obstacleRemoval);

	// increase/decrease last inserted vehicle's max speed
	framework.define_key("s", "changeVehicleMaxSpeed", &changeVehicleMaxSpeed,
			(void*) &steerVehicles);
	framework.define_key("shift-s", "changeVehicleMaxSpeed",
			&changeVehicleMaxSpeed, (void*) &steerVehicles);
	// increase/decrease last inserted vehicle's max force
	framework.define_key("f", "changeVehicleMaxForce", &changeVehicleMaxForce,
			(void*) &steerVehicles);
	framework.define_key("shift-f", "changeVehicleMaxForce",
			&changeVehicleMaxForce, (void*) &steerVehicles);

	// handle OSSteerVehicle(s)' events
	framework.define_key("avoid_obstacle", "handleVehicleEvent",
			&handleVehicleEvent, nullptr);
	framework.define_key("avoid_close_neighbor", "handleVehicleEvent",
			&handleVehicleEvent, nullptr);

	// write to bam file on exit
	window->get_graphics_window()->set_close_request_event(
			"close_request_event");
	framework.define_key("close_request_event", "writeToBamFile",
			&writeToBamFileAndExit, (void*) &bamFileName);

	// player will be driven by WASD keys XXX


	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-128.0, 120.0, -40.0);
	trackball->set_hpr(0.0, 20.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}

// set parameters as strings before plug-ins/vehicles creation
void setParametersBeforeCreation()
{
	OSSteerManager* steerMgr = OSSteerManager::get_global_ptr();
	ValueList<string> valueList;
	// set vehicle's mass, speed
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "mass", "2.0");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "speed",
			"0.01");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value("avoid_obstacle@avoid_obstacle@1.0:avoid_close_neighbor@avoid_close_neighbor@");
	steerMgr->set_parameter_values(OSSteerManager::STEERVEHICLE,
			"thrown_events", valueList);
	//
	printCreationParameters();
}

// custom update task for plug-ins
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask* task, void* data)
{
	PT(OSSteerPlugIn)steerPlugIn = reinterpret_cast<OSSteerPlugIn*>(data);
	// call update for steerPlugIn
	double dt = ClockObject::get_global_clock()->get_dt();
	steerPlugIn->update(dt);
	// handle vehicle's animation
	for (int i = 0; i < (int)vehicleAnimCtls.size(); ++i)
	{
		// get current velocity size
		float currentVelSize = steerVehicles[i]->get_speed();
		if (currentVelSize > 0.0)
		{
			int animOnIdx, animOffIdx;
			currentVelSize < 4.0 ? animOnIdx = 0: animOnIdx = 1;
			animOffIdx = (animOnIdx + 1) % 2;
			// Off anim (0:walk, 1:run)
			if (vehicleAnimCtls[i][animOffIdx]->is_playing())
			{
				vehicleAnimCtls[i][animOffIdx]->stop();
			}
			// On amin (0:walk, 1:run)
			vehicleAnimCtls[i][animOnIdx]->set_play_rate(
					currentVelSize / animRateFactor[animOnIdx]);
			if (! vehicleAnimCtls[i][animOnIdx]->is_playing())
			{
				vehicleAnimCtls[i][animOnIdx]->loop(true);
			}
		}
		else
		{
			// stop any animation
			vehicleAnimCtls[i][0]->stop();
			vehicleAnimCtls[i][1]->stop();
		}
	}
	//
	return AsyncTask::DS_cont;
}

// get the player, model and animations
NodePath getPlayerModelAnims(const string& name, float scale,
		int vehicleFileIdx, PT(OSSteerPlugIn)steerPlugIn,
		vector<PT(OSSteerVehicle)>&steerVehicles,
		vector<vector<PT(AnimControl)> >& vehicleAnimCtls, const LPoint3f& pos)
{
	// get some models, with animations, to attach to vehicles
	// get the model
	NodePath vehicleNP = window->load_model(framework.get_models(), vehicleFile[vehicleFileIdx]);
	// set name
	vehicleNP.set_name(name);
	// set scale
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
	//
	WPT(OSSteerManager) steerMgr = OSSteerManager::get_global_ptr();
	// create the steer vehicle (attached to the reference node)
	// note: vehicle's move type is ignored
	NodePath steerVehicleNP = steerMgr->create_steer_vehicle("PlayerVehicle");
	steerVehicles.push_back(DCAST(OSSteerVehicle, steerVehicleNP.node()));
	// set the position
	steerVehicleNP.set_pos(pos);
	// attach some geometry (a model) to steer vehicle
	vehicleNP.reparent_to(steerVehicleNP);
	// add the steer vehicle to the plug-in
	steerPlugIn->add_steer_vehicle(steerVehicleNP);
	// return the steerVehicleNP
	return steerVehicleNP;
}


//XXX
class Driver
{
public:
	Driver(GraphicsWindow* win, const NodePath& ownerObject);
	virtual ~Driver();

	virtual void reset();
	virtual void onAddToObjectSetup();
	virtual void onRemoveFromObjectCleanup();
	virtual void onAddToSceneSetup();
	virtual void onRemoveFromSceneCleanup();

	virtual void update(void* data);

	bool enable();
	bool disable();
	bool isEnabled();

	void enableForward(bool enable);
	bool isForwardEnabled();
	void enableBackward(bool enable);
	bool isBackwardEnabled();
	void enableStrafeLeft(bool enable);
	bool isStrafeLeftEnabled();
	void enableStrafeRight(bool enable);
	bool isStrafeRightEnabled();
	void enableUp(bool enable);
	bool isUpEnabled();
	void enableDown(bool enable);
	bool isDownEnabled();
	void enableHeadLeft(bool enable);
	bool isHeadLeftEnabled();
	void enableHeadRight(bool enable);
	bool isHeadRightEnabled();
	void enablePitchUp(bool enable);
	bool isPitchUpEnabled();
	void enablePitchDown(bool enable);
	bool isPitchDownEnabled();
	void enableMouseMove(bool enable);
	bool isMouseMoveEnabled();

	//max values
	void setHeadLimit(bool enabled = false, float hLimit = 0.0);
	void setPitchLimit(bool enabled = false, float pLimit = 0.0);
	void setMaxLinearSpeed(const LVector3f& linearSpeed);
	void setMaxAngularSpeed(float angularSpeed);
	LVector3f getMaxSpeeds(float& angularSpeed);
	void setLinearAccel(const LVector3f& linearAccel);
	void setAngularAccel(float angularAccel);
	LVector3f getAccels(float& angularAccel);
	void setLinearFriction(float linearFriction);
	void setAngularFriction(float angularFriction);
	void getFrictions(float& linearFriction, float& angularFriction);
	void setSens(float sensX, float sensY);
	void getSens(float& sensX, float& sensY);
	void setFastFactor(float factor);
	float getFastFactor();
	//speed current values
	LVector3f getCurrentSpeeds(float& angularSpeedH, float& angularSpeedP);

private:
	///Main parameters.
	GraphicsWindow* mWin;
	NodePath mOwnerObjectNP;
	///Enabling flags.
	bool mStartEnabled, mEnabled;
	///Key controls and effective keys.
	bool mForward, mBackward, mStrafeLeft, mStrafeRight, mUp, mDown, mHeadLeft,
			mHeadRight, mPitchUp, mPitchDown, mMouseMove;
	bool mForwardKey, mBackwardKey, mStrafeLeftKey, mStrafeRightKey, mUpKey,
			mDownKey, mHeadLeftKey, mHeadRightKey, mPitchUpKey, mPitchDownKey,
			mMouseMoveKey;
	std::string mSpeedKey;
	///Key control values.
	bool mMouseEnabledH, mMouseEnabledP;
	bool mHeadLimitEnabled, mPitchLimitEnabled;
	float mHLimit, mPLimit;
	int mSignOfTranslation, mSignOfMouse;
	/// Sensitivity settings.
	float mFastFactor;
	LVecBase3f mActualSpeedXYZ, mMaxSpeedXYZ, mMaxSpeedSquaredXYZ;
	float mActualSpeedH, mActualSpeedP, mMaxSpeedHP, mMaxSpeedSquaredHP;
	LVecBase3f mAccelXYZ;
	float mAccelHP;
	float mFrictionXYZ;
	float mFrictionHP;
	float mStopThreshold;
	float mSensX, mSensY;
	int mCentX, mCentY;
	//member functions
	virtual bool initialize();
	void doEnable();
	void doDisable();
};

Driver::Driver(GraphicsWindow* win, const NodePath& ownerObjectNP)
{
	mWin = win;
	mOwnerObjectNP = ownerObjectNP;
	reset();
}

Driver::~Driver()
{
}

inline void Driver::reset()
{
	//
	mStartEnabled = mEnabled = false;
	mForward = mBackward = mStrafeLeft = mStrafeRight = mUp = mDown =
			mHeadLeft = mHeadRight = mPitchUp = mPitchDown = false;
	//by default we consider mouse moved on every update, because
	//we want mouse poll by default; this can be changed by calling
	//the enabler (for example by an handler responding to mouse-move
	//event if it is possible. See: http://www.panda3d.org/forums/viewtopic.php?t=9326
	// http://www.panda3d.org/forums/viewtopic.php?t=6049)
	mMouseMove = true;
	mForwardKey = mBackwardKey = mStrafeLeftKey = mStrafeRightKey = mUpKey, mDownKey =
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
}

inline bool Driver::isEnabled()
{
	return mEnabled;
}

inline void Driver::enableForward(bool enable)
{
	if (mForwardKey)
	{
		mForward = enable;
	}
}

inline bool Driver::isForwardEnabled()
{
	return mForward;
}

inline void Driver::enableBackward(bool enable)
{
	if (mBackwardKey)
	{
		mBackward = enable;
	}
}

inline bool Driver::isBackwardEnabled()
{
	return mBackward;
}

inline void Driver::enableStrafeLeft(bool enable)
{
	if (mStrafeLeftKey)
	{
		mStrafeLeft = enable;
	}
}

inline bool Driver::isStrafeLeftEnabled()
{

	return mStrafeLeft;
}

inline void Driver::enableStrafeRight(bool enable)
{

	if (mStrafeRightKey)
	{
		mStrafeRight = enable;
	}
}

inline bool Driver::isStrafeRightEnabled()
{

	return mStrafeRight;
}

inline void Driver::enableUp(bool enable)
{

	if (mUpKey)
	{
		mUp = enable;
	}
}

inline bool Driver::isUpEnabled()
{

	return mUp;
}

inline void Driver::enableDown(bool enable)
{

	if (mDownKey)
	{
		mDown = enable;
	}
}

inline bool Driver::isDownEnabled()
{

	return mDown;
}

inline void Driver::enableHeadLeft(bool enable)
{

	if (mHeadLeftKey)
	{
		mHeadLeft = enable;
	}
}

inline bool Driver::isHeadLeftEnabled()
{

	return mHeadLeft;
}

inline void Driver::enableHeadRight(bool enable)
{

	if (mHeadRightKey)
	{
		mHeadRight = enable;
	}
}

inline bool Driver::isHeadRightEnabled()
{

	return mHeadRight;
}

inline void Driver::enablePitchUp(bool enable)
{

	if (mPitchUpKey)
	{
		mPitchUp = enable;
	}
}

inline bool Driver::isPitchUpEnabled()
{

	return mPitchUp;
}

inline void Driver::enablePitchDown(bool enable)
{

	if (mPitchDownKey)
	{
		mPitchDown = enable;
	}
}

inline bool Driver::isPitchDownEnabled()
{

	return mPitchDown;
}

inline void Driver::enableMouseMove(bool enable)
{

	if (mMouseMoveKey)
	{
		mMouseMove = enable;
	}
}

inline bool Driver::isMouseMoveEnabled()
{

	return mMouseMove;
}

inline void Driver::setHeadLimit(bool enabled, float hLimit)
{

	mHeadLimitEnabled = enabled;
	hLimit >= 0.0 ? mHLimit = hLimit : mHLimit = -hLimit;
}

inline void Driver::setPitchLimit(bool enabled, float pLimit)
{

	mPitchLimitEnabled = enabled;
	pLimit >= 0.0 ? mPLimit = pLimit : mPLimit = -pLimit;
}

inline void Driver::setMaxLinearSpeed(const LVector3f& maxLinearSpeed)
{

	mMaxSpeedXYZ = maxLinearSpeed;
	mMaxSpeedSquaredXYZ = LVector3f(
			maxLinearSpeed.get_x() * maxLinearSpeed.get_x(),
			maxLinearSpeed.get_y() * maxLinearSpeed.get_y(),
			maxLinearSpeed.get_z() * maxLinearSpeed.get_z());
}

inline void Driver::setMaxAngularSpeed(float maxAngularSpeed)
{

	mMaxSpeedHP = maxAngularSpeed;
	mMaxSpeedSquaredHP = maxAngularSpeed * maxAngularSpeed;
}

inline LVector3f Driver::getMaxSpeeds(float& maxAngularSpeed)
{

	maxAngularSpeed = mMaxSpeedHP;
	return mMaxSpeedXYZ;
}

inline void Driver::setLinearAccel(const LVector3f& linearAccel)
{

	mAccelXYZ = linearAccel;
}

inline void Driver::setAngularAccel(float angularAccel)
{

	mAccelHP = angularAccel;
}

inline LVector3f Driver::getAccels(float& angularAccel)
{

	angularAccel = mAccelHP;
	return mAccelXYZ;
}

inline void Driver::setLinearFriction(float linearFriction)
{

	mFrictionXYZ = linearFriction;
}

inline void Driver::setAngularFriction(float angularFriction)
{

	mFrictionHP = angularFriction;
	if ((mFrictionHP < 0.0) or (mFrictionHP > 1.0))
	{
		mFrictionHP = 0.1;
	}
}

inline void Driver::getFrictions(float& linearFriction, float& angularFriction)
{

	linearFriction = mFrictionXYZ;
	angularFriction = mFrictionHP;
}

inline void Driver::setSens(float sensX, float sensY)
{

	mSensX = sensX;
	mSensY = sensY;
}

inline void Driver::getSens(float& sensX, float& sensY)
{

	sensX = mSensX;
	sensY = mSensY;
}

inline void Driver::setFastFactor(float factor)
{

	mFastFactor = factor;
}

inline float Driver::getFastFactor()
{

	return mFastFactor;
}

inline LVector3f Driver::getCurrentSpeeds(float& angularSpeedH,
		float& angularSpeedP)
{

	angularSpeedH = mActualSpeedH;
	angularSpeedP = mActualSpeedP;
	return mActualSpeedXYZ;
}

bool Driver::initialize()
{
	bool result = true;
	//get settings from template
	//enabling setting
	mStartEnabled = true;
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
	float value, absValue;
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
	mFrictionXYZ = 0.1;
	//angular friction (>=0)
	mFrictionHP = 0.1;
	//stop threshold [0.0, 1.0]
	mStopThreshold = 0.01;
	//fast factor (>=0)
	mFastFactor = 5.0;
	//sens x (>=0)
	mSensX = 0.2;
	//sens_y (>=0)
	mSensY = 0.2;
	//
	return result;
}

void Driver::onAddToObjectSetup()
{
	//
	mCentX = mWin->get_properties().get_x_size() / 2;
	mCentY = mWin->get_properties().get_y_size() / 2;
}

void Driver::onRemoveFromObjectCleanup()
{
	//see disable
	if (mEnabled and (mMouseEnabledH or mMouseEnabledP or mMouseMoveKey))
	{
		//we have control through mouse movements
		//show mouse cursor
		WindowProperties props;
		props.set_cursor_hidden(false);
		mWin->request_properties(props);
	}
	//
	reset();
}

void Driver::onAddToSceneSetup()
{
	//enable the component (if requested)
	if (mStartEnabled)
	{
		doEnable();
	}
}

void Driver::onRemoveFromSceneCleanup()
{
	//remove from control manager update
//	GameControlManager::GetSingletonPtr()->removeFromControlUpdate(this); XXX
}

bool Driver::enable()
{
	//if enabled return
	RETURN_ON_COND(mEnabled, false)

	//actual ebnabling
	doEnable();
	//
	return true;
}

void Driver::doEnable()
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

	//add to the control manager update
//	GameControlManager::GetSingletonPtr()->addToControlUpdate(this); XXX
}

bool Driver::disable()
{
	//if not enabled return
	RETURN_ON_COND(not mEnabled, false)

	//actual disabling
	doDisable();
	//
	return true;
}

void Driver::doDisable()
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
}

void Driver::update(void* data)
{

	float dt = *(reinterpret_cast<float*>(data));

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
	float kLinearReductFactor = mFrictionXYZ * dt;
	if (kLinearReductFactor > 1.0)
	{
		kLinearReductFactor = 1.0;
	}
	//y axis
	if (mForward and (not mBackward))
	{
		if (mAccelXYZ.get_y() != 0)
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
		if (mAccelXYZ.get_y() != 0)
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
					mActualSpeedXYZ.get_y() * (1 - kLinearReductFactor));
		}
	}
	//x axis
	if (mStrafeLeft and (not mStrafeRight))
	{
		if (mAccelXYZ.get_x() != 0)
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
		if (mAccelXYZ.get_x() != 0)
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
					mActualSpeedXYZ.get_x() * (1 - kLinearReductFactor));
		}
	}
	//z axis
	if (mUp and (not mDown))
	{
		if (mAccelXYZ.get_z() != 0)
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
		if (mAccelXYZ.get_z() != 0)
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
					mActualSpeedXYZ.get_z() * (1 - kLinearReductFactor));
		}
	}
	//rotation h
	if (mHeadLeft and (not mHeadRight))
	{
		if (mAccelHP != 0)
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
		if (mAccelHP != 0)
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
			float kAngularReductFactor = mFrictionHP * dt;
			if (kAngularReductFactor > 1.0)
			{
				kAngularReductFactor = 1.0;
			}
			mActualSpeedH = mActualSpeedH * (1 - kAngularReductFactor);
		}
	}
	//rotation p
	if (mPitchUp and (not mPitchDown))
	{
		if (mAccelHP != 0)
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
		if (mAccelHP != 0)
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
			float kAngularReductFactor = mFrictionHP * dt;
			if (kAngularReductFactor > 1.0)
			{
				kAngularReductFactor = 1.0;
			}
			mActualSpeedP = mActualSpeedP * (1 - kAngularReductFactor);
		}
	}
}
