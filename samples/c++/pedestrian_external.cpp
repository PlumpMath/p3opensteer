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
Driver* playerDriver;
LVector3f playerHeightRayCast;
int forwardMove = 1;
int forwardMoveStop = -1;
int leftMove = 2;
int leftMoveStop = -2;
int backwardMove = 3;
int backwardMoveStop = -3;
int rightMove = 4;
int rightMoveStop = -4;
//
void setParametersBeforeCreation();
void toggleWanderBehavior(const Event*, void*);
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask*, void*);
NodePath getPlayerModelAnims(const string&, float, int, PT(OSSteerPlugIn),
		vector<PT(OSSteerVehicle)>&, vector<vector<PT(AnimControl)> >&,
		const LPoint3f& );
void movePlayer(const Event*, void*);

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
			"- press \"up\"/\"left\"/\"down\"/\"right\" arrows to move the player\n"
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
		playerNP = getPlayerModelAnims("PlayerNP", 0.8, 0, steerPlugIn,
				steerVehicles, vehicleAnimCtls,
				LPoint3f(141.597, 73.496, 2.14218));
		// highlight the player
        playerNP.set_color(1.0, 1.0, 0.0, 0);

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
			&handleVehicleEvent, NULL);
	framework.define_key("avoid_close_neighbor", "handleVehicleEvent",
			&handleVehicleEvent, NULL);

	// write to bam file on exit
	window->get_graphics_window()->set_close_request_event(
			"close_request_event");
	framework.define_key("close_request_event", "writeToBamFile",
			&writeToBamFileAndExit, (void*) &bamFileName);

	// 'pedestrian' specific: toggle wander behavior
	framework.define_key("t", "toggleWanderBehavior", &toggleWanderBehavior,
			NULL);

	// get player dims for kinematic ray cast
	LVecBase3f modelDims;
	LVector3f modelDeltaCenter;
	steerMgr->get_bounding_dimensions(playerNP,	modelDims, modelDeltaCenter);
	playerHeightRayCast = LVector3f(0.0, 0.0, modelDims.get_z());

	// player will be driven by arrows keys
	playerDriver = new (nothrow) Driver(&framework, playerNP, 10);
    playerDriver->set_max_angular_speed(100.0);
    playerDriver->set_angular_accel(50.0);
    playerDriver->set_max_linear_speed(LVector3f(8.0, 8.0, 8.0));
    playerDriver->set_linear_accel(LVecBase3f(1.0, 1.0, 1.0));
    playerDriver->set_linear_friction(1.5);
	playerDriver->enable();
	framework.define_key("arrow_up", "forwardMove", &movePlayer, &forwardMove);
	framework.define_key("arrow_up-up", "forwardMoveStop", &movePlayer, &forwardMoveStop);
	framework.define_key("arrow_left", "leftMove", &movePlayer, &leftMove);
	framework.define_key("arrow_left-up", "leftMoveStop", &movePlayer, &leftMoveStop);
	framework.define_key("arrow_down", "backwardMove", &movePlayer, &backwardMove);
	framework.define_key("arrow_down-up", "backwardMoveStop", &movePlayer, &backwardMoveStop);
	framework.define_key("arrow_right", "rightMove", &movePlayer, &rightMove);
	framework.define_key("arrow_right-up", "rightMoveStop", &movePlayer, &rightMoveStop);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-128.0, 120.0, -40.0);
	trackball->set_hpr(0.0, 20.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	delete playerDriver;

	return (0);
}

// set parameters as strings before plug-ins/vehicles creation
void setParametersBeforeCreation()
{
	OSSteerManager* steerMgr = OSSteerManager::get_global_ptr();
	ValueList<string> valueList;
	// set vehicle's mass, speed
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "external_update",
			"false");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "mass", "2.0");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "speed",
			"0.01");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value("avoid_obstacle@avoid_obstacle@1.0:"
			"avoid_close_neighbor@avoid_close_neighbor@1.0:"
			"avoid_neighbor@avoid_neighbor@1.0");
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
	// make playerNP kinematic (ie stand on floor)
	if (DCAST(OSSteerVehicle, playerNP.node())->get_speed() > 0.0)
	{
		// get steer manager
		WPT(OSSteerManager)steerMgr = OSSteerManager::get_global_ptr();
		// correct panda's Z: set the collision ray origin wrt collision root
		LPoint3f pOrig = steerMgr->get_collision_root().get_relative_point(
				steerMgr->get_reference_node_path(), playerNP.get_pos()) + playerHeightRayCast * 2.0;
		// get the collision height wrt the reference node path
		Pair<bool,float> gotCollisionZ = steerMgr->get_collision_height(pOrig,
				steerMgr->get_reference_node_path());
		if (gotCollisionZ.get_first())
		{
			//updatedPos.z needs correction
			playerNP.set_z(gotCollisionZ.get_second());
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
	// set the name
	steerVehicleNP.set_name(name);
	// set scale
	steerVehicleNP.set_scale(scale);
	// set the position
	steerVehicleNP.set_pos(pos);
	// attach some geometry (a model) to steer vehicle
	vehicleNP.reparent_to(steerVehicleNP);
	// add the steer vehicle to the plug-in
	steerPlugIn->add_steer_vehicle(steerVehicleNP);
	// return the steerVehicleNP
	return steerVehicleNP;
}

// player's movement callback
void movePlayer(const Event*, void* data)
{
	if (not playerDriver)
	{
		return;
	}
	int action = *reinterpret_cast<int*>(data);
	bool enable;
	if (action > 0)
	{
		//start movement
		enable = true;
	}
	else
	{
		action = -action;
		//stop movement
		enable = false;
	}
	//
	if (action == forwardMove)
	{
		playerDriver->enable_forward(enable);
	}
	else if(action == leftMove)
	{
		playerDriver->enable_head_left(enable);
	}
	else if(action == backwardMove)
	{
		playerDriver->enable_backward(enable);
	}
	else if(action == rightMove)
	{
		playerDriver->enable_head_right(enable);
	}
}
