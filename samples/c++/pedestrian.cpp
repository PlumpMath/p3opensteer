/**
 * \file main.cpp
 *
 * \date 2016-05-26
 * \author consultit
 */

#include "common.h"

///specific data/functions declarations/definitions
NodePath sceneNP, vehicleNP[2];
PT(AnimControl)vehicleAnimCtls[2][2];
PT(OSSteerPlugIn)steerPlugIn;
PT(OSSteerVehicle)steerVehicle[2];
int NUMVEHICLES = 2;
//
void setParametersBeforeCreation();
void toggleSteeringSpeed(const Event*, void*);
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask*, void*);

int main(int argc, char *argv[])
{
	string msg("'pedestrian'");
	startFramework(argc, argv, msg);

	/// here is room for your own code
	// print some help to screen
	PT(TextNode)text;
	text = new TextNode("Help");
	text->set_text(
			msg + "\n\n"
			"- press \"d\" to toggle debug drawing\n"
            "- press \"s\"/\"shift-s\" to increase/decrease vehicle's max speed\n"
            "- press \"f\"/\"shift-f\" to increase/decrease vehicle's max force\n"
			"- press \"s\" to toggle steering speed\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(-1.25, 0.0, 0.9);
	textNodePath.set_scale(0.035);

	// create a steer manager; set root and mask to manage 'kinematic' vehicles
	WPT(OSSteerManager)steerMgr = new OSSteerManager(window->get_render(), mask);

	// print creation parameters: defult values
	cout << endl << "Default creation parameters:";
	printCreationParameters();

	// set creation parameters as strings before plug-in/vehicles creation
	cout << endl << "Current creation parameters:";
	setParametersBeforeCreation();

	// load or restore all scene stuff: if passed an argument
	// try to read it from bam file
	if ((not (argc > 1)) or (not readFromBamFile(argv[1])))
	{
		// no argument or no valid bamFile
		// reparent the reference node to render
		steerMgr->get_reference_node_path().reparent_to(window->get_render());

		// get a sceneNP and reparent to the reference node
		sceneNP = loadTerrain();
		// set name: to ease restoring from bam file
		sceneNP.set_name("SceneNP");
		sceneNP.reparent_to(steerMgr->get_reference_node_path());

		// set sceneNP's collide mask
		sceneNP.set_collide_mask(mask);

		// create the default plug-in (attached to the reference node)
		NodePath plugInNP = steerMgr->create_steer_plug_in();
		steerPlugIn = DCAST(OSSteerPlugIn, plugInNP.node());

		// get steer vehicles, models and animations
		//1: get the model
		//2: create the steer vehicle (it is attached to the reference node)
		//3: set its position
		//4: attach the model to steer vehicle
		//5: add the steer vehicle to the plug-in
//		getVehiclesModelsAnims(NUMVEHICLES, sceneNP, vehicleNP, steerPlugIn, XXX
//		                           steerVehicle, vehicleAnimCtls);
	}
	else
	{
		// valid bamFile
		// restore plug-in: through steer manager
		NodePath steerPlugInNP = OSSteerManager::get_global_ptr()->get_steer_plug_in(0);
		steerPlugIn = DCAST(OSSteerPlugIn, steerPlugInNP.node());
		// restore sceneNP: through panda3d
		NodePath sceneNP = OSSteerManager::get_global_ptr()->
				get_reference_node_path().find("**/SceneNP");
		// reparent the reference node to render
		OSSteerManager::get_global_ptr()->get_reference_node_path().reparent_to(
		window->get_render());

		// restore steer vehicles
		for (int i = 0; i < NUMVEHICLES; ++i)
		{
			// restore the steer vehicle: through steer manager
			NodePath steerVehicleNP =
			OSSteerManager::get_global_ptr()->get_steer_vehicle(i);
			steerVehicle[i] = DCAST(OSSteerVehicle, steerVehicleNP.node());
			// restore animations
			AnimControlCollection tmpAnims;
			auto_bind(steerVehicle[i], tmpAnims);
			for (int j = 0; j < tmpAnims.get_num_anims(); ++j)
			{
				vehicleAnimCtls[i][j] = tmpAnims.get_anim(j);
			}
		}
	}

	// show the added vehicles
	cout << "Vehicles added to plug-in:" << endl;
	for (int i = 0; i < steerPlugIn->get_num_steer_vehicles(); ++i)
	{
		cout << "\t- " << *((*steerPlugIn)[i]) << endl;
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

	// increase/decrease vehicle's max speed
	framework.define_key("s", "changeVehicleMaxSpeed", &changeVehicleMaxSpeed,
			(void*) steerVehicle[0].p());
	framework.define_key("shift-s", "changeVehicleMaxSpeed", &changeVehicleMaxSpeed,
			(void*) steerVehicle[0].p());
	// increase/decrease vehicle's max force
	framework.define_key("f", "changeVehicleMaxForce", &changeVehicleMaxForce,
			(void*) steerVehicle[0].p());
	framework.define_key("shift-f", "changeVehicleMaxForce", &changeVehicleMaxForce,
			(void*) steerVehicle[0].p());

	// handle OSSteerVehicle(s)' events
	framework.define_key("move-event", "handleVehicleEvent",
			&handleVehicleEvent, nullptr);

	// write to bam file on exit
	window->get_graphics_window()->set_close_request_event(
			"close_request_event");
	framework.define_key("close_request_event", "writeToBamFile",
			&writeToBamFileAndExit, (void*) &bamFileName);

	// 'low speed turn' specific: toggle steering speed
	framework.define_key("t", "toggleSteeringSpeed", &toggleSteeringSpeed,
			nullptr);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(0.0, 50.0, 0.0);
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
			"low_speed_turn");

	// set vehicle type, mass, speed
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "vehicle_type",
			"low_speed_turn");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "mass", "2.0");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "speed", "0.01");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value("move@move-event@0.5");
	steerMgr->set_parameter_values(OSSteerManager::STEERVEHICLE,
			"thrown_events", valueList);
	//
	printCreationParameters();
}

// toggle steering speed
void toggleSteeringSpeed(const Event*, void*)
{
	if (steerVehicle[0]->get_steering_speed() < 4.9)
	{
		steerVehicle[0]->set_steering_speed(5.0);
	}
	else
	{
		steerVehicle[0]->set_steering_speed(1.0);
	}
	cout << *steerVehicle[0] << "'s steering speed is "
			<< steerVehicle[0]->get_steering_speed() << endl;
}

// custom update task for plug-ins
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask* task, void* data)
{
	PT(OSSteerPlugIn)steerPlugIn = reinterpret_cast<OSSteerPlugIn*>(data);
	// call update for steerPlugIn
	double dt = ClockObject::get_global_clock()->get_dt();
	steerPlugIn->update(dt);
	// handle vehicle's animation
	for (int i = 0; i < NUMVEHICLES; ++i)
	{
		// get current velocity size
		float currentVelSize = steerVehicle[i]->get_speed();
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
					currentVelSize / rateFactor[animOnIdx]);
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
