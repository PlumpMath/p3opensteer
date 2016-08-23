/**
 * \file capture_the_flag.cpp
 *
 * \date 2016-05-26
 * \author consultit
 */

#include "common.h"

///specific data/functions declarations/definitions
NodePath sceneNP;
vector<vector<PT(AnimControl)> > vehicleAnimCtls;
PT(OSSteerPlugIn)steerPlugIn;
vector<PT(OSSteerVehicle)>steerVehicles;
//
NodePath flagNP;
AnimControlCollection flagAnims;

void setParametersBeforeCreation();
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask*, void*);
void addSeeker(const Event*, void*);
void addEnemy(const Event*, void*);
void setHomeBaseCenter(const Event*, void*);
NodePath getFlag(const string&);

int main(int argc, char *argv[])
{
	string msg("'capture_the_flag'");
	startFramework(argc, argv, msg);

	/// here is room for your own code
	// print some help to screen
	PT(TextNode)text;
	text = new TextNode("Help");
	text->set_text(
            msg + "\n\n"
            "- press \"d\" to toggle debug drawing\n"
            "- press \"a\"/\"e\" to add a seeker/enemy\n"
            "- press \"h\" to set home base center\n"
            "- press \"s\"/\"shift-s\" to increase/decrease last inserted vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease last inserted vehicle's max force\n"
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
		sceneNP = loadTerrainLowPoly("SceneNP", 128, 64);
		// and reparent to the reference node
		sceneNP.reparent_to(steerMgr->get_reference_node_path());

		// set sceneNP's collide mask
		sceneNP.set_collide_mask(mask);

		// set creation parameters as strings before plug-in/vehicles creation
		cout << endl << "Current creation parameters:";
		setParametersBeforeCreation();

		// create the plug-in (attached to the reference node)
		NodePath plugInNP = steerMgr->create_steer_plug_in();
		steerPlugIn = DCAST(OSSteerPlugIn, plugInNP.node());

		// set the capture the flag common settings
		steerPlugIn->set_home_base_radius(6.0);
		steerPlugIn->set_braking_rate(0.75);
		steerPlugIn->set_avoidance_predict_time_min(0.9);
		steerPlugIn->set_avoidance_predict_time_max(2.0);

		// load flag model naming it with "FlagNP" to ease restoring from bam
		// file
		flagNP = getFlag("FlagNP");
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

		// restore steer vehicles
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

		// restore flag and its animation
		flagNP =
				OSSteerManager::get_global_ptr()->get_reference_node_path().find(
						"**/FlagNP");
		auto_bind(flagNP.node(), flagAnims);
		flagAnims.get_anim(0)->loop(true);

		// set creation parameters as strings before other plug-ins/vehicles creation
		cout << endl << "Current creation parameters:";
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
	HandleVehicleData seekerData(1.2, 0, "kinematic", sceneNP,
						steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("a", "addSeeker", &addSeeker,
			(void*) &seekerData);
	HandleVehicleData enemyData(1.2, 1, "kinematic", sceneNP,
						steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("e", "addEnemy", &addEnemy,
			(void*) &enemyData);

	// set home base center
	framework.define_key("h", "setHomeBaseCenter", &setHomeBaseCenter,
			(void*) &flagNP);

	// handle obstacle addition
	HandleObstacleData obstacleAddition(true, sceneNP, steerPlugIn,
			LVecBase3f(0.05, 0.05, 0.08));
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
			&handleVehicleEvent, NULL);

	// write to bam file on exit
	window->get_graphics_window()->set_close_request_event(
			"close_request_event");
	framework.define_key("close_request_event", "writeToBamFile",
			&writeToBamFileAndExit, (void*) &bamFileName);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(0.0, 320.0, -10.0);
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
	// set plug-in type
	steerMgr->set_parameter_value(OSSteerManager::STEERPLUGIN, "plugin_type",
			"capture_the_flag");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value("avoid_obstacle@avoid_obstacle@1.0");
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

// creates a generic vehicle for ctf plug-in
static bool createCtfVehicle(const Event* e, void* data,
		OSSteerVehicle::OSSteerVehicleType vehicleType)
{
    float maxForce = 1.0;
    float maxSpeed = 5.0;
	// set vehicle's type == ctf_enemy
	string typeStr = "ctf_enemy";
    float speed = 1.0;
	if (vehicleType == OSSteerVehicle::CTF_SEEKER)
	{
		typeStr = "ctf_seeker";
        speed = 0.0;
	}
	OSSteerManager::get_global_ptr()->set_parameter_value(
			OSSteerManager::STEERVEHICLE, "vehicle_type", typeStr);

	unsigned int oldPlayerNum = steerVehicles.size();
	// handle vehicle
	handleVehicles(e, data);
	if (steerVehicles.size() > oldPlayerNum)
	{
		// set vehicle's parameters
		steerVehicles.back()->set_max_force(maxForce);
		steerVehicles.back()->set_max_speed(maxSpeed);
		steerVehicles.back()->set_speed(speed);
		steerVehicles.back()->set_up_axis_fixed(true);
		return true;
	}
	return false;
}

// adds a seeker
void addSeeker(const Event* e, void* data)
{
	if (not data)
	{
		return;
	}

	createCtfVehicle(e, data, OSSteerVehicle::CTF_SEEKER);
}

// adds an enemy
void addEnemy(const Event* e, void* data)
{
	if (not data)
	{
		return;
	}

	createCtfVehicle(e, data, OSSteerVehicle::CTF_ENEMY);
}

// set home base center
void setHomeBaseCenter(const Event*, void* data)
{
	if (steerPlugIn.is_null())
	{
		return;
	}

	NodePath* flag = reinterpret_cast<NodePath*>(data);
	// get the collision entry, if any
	PT(CollisionEntry)entry0 = getCollisionEntryFromCamera();
	if (entry0)
	{
		// get the hit object
		NodePath hitObject = entry0->get_into_node_path();
		cout << "hit " << hitObject << " object" << endl;

		// set home base center's position
		LPoint3f center = entry0->get_surface_point(window->get_render());
		steerPlugIn->set_home_base_center(center);
		flag->set_pos(center);
		cout << "set home base center at: " << center << endl;
	}
}

// load the flag
NodePath getFlag(const string& name)
{
	NodePath flag = window->load_model(framework.get_models(),
			"flag_oga.egg");
	flag.set_two_sided(true);
	flag.set_scale(1.5);
	flag.set_name(name);
	flag.reparent_to(
			OSSteerManager::get_global_ptr()->get_reference_node_path());
	NodePath flagWave = window->load_model(flag, "flag_oga-wave.egg");
	auto_bind(flag.node(), flagAnims);
	flagAnims.get_anim(0)->loop(true);
	return flag;
}
