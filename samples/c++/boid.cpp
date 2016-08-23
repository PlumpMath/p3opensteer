/**
 * \file boid.cpp
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
void setParametersBeforeCreation();
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask*, void*);

int main(int argc, char *argv[])
{
	string msg("'boid'");
	startFramework(argc, argv, msg);

	/// here is room for your own code
	// print some help to screen
	PT(TextNode)text;
	text = new TextNode("Help");
	text->set_text(
            msg + "\n\n"
            "- press \"d\" to toggle debug drawing\n"
			"- press \"a\" to add 'opensteer' vehicle above the hit point\n"
            "- press \"s\"/\"shift-s\" to increase/decrease last inserted vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease last inserted vehicle's max force\n"
			"- press \"o\"/\"shift-o\" to add/remove obstacle\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(-1.25, 0.0, -0.5);
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

		// set creation parameters as strings before plug-in/vehicles creation
		cout << endl << "Current creation parameters:";
		setParametersBeforeCreation();

		// create the plug-in (attached to the reference node)
		NodePath plugInNP = steerMgr->create_steer_plug_in();
		steerPlugIn = DCAST(OSSteerPlugIn, plugInNP.node());

		// set world's center and radius
		steerPlugIn->set_world_center(LPoint3f(147.7, 145.6, 40.8));
		steerPlugIn->set_world_radius(25.0);
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
	HandleVehicleData vehicleData(0.007, 2, "opensteer", sceneNP,
						steerPlugIn, steerVehicles, vehicleAnimCtls,
						LVector3f(0.0, 0.0, 25.0));
	framework.define_key("a", "addVehicle", &handleVehicles,
			(void*) &vehicleData);

	// handle obstacle addition
	HandleObstacleData obstacleAddition(true, sceneNP, steerPlugIn,
			LVecBase3f(0.03, 0.03, 0.24));
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
	trackball->set_pos(-128.0, -20.0, -60.0);
	trackball->set_hpr(8.0, 10.0, 0.0);

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
			"boid");

	// set vehicle's type, max force, max speed, speed
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "vehicle_type",
			"boid");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "max_force",
			"5.0");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "max_speed",
			"10.0");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "speed", "3.0");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value("avoid_obstacle@avoid_obstacle@");
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
