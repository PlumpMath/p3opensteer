/**
 * \file soccer.cpp
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
void addPlayerA(const Event*, void*);
void addPlayerB(const Event*, void*);
void addBall(const Event*, void*);

int main(int argc, char *argv[])
{
	string msg("'soccer'");
	startFramework(argc, argv, msg);

	/// here is room for your own code
	// print some help to screen
	PT(TextNode)text;
	text = new TextNode("Help");
	text->set_text(
            msg + "\n\n"
            "- press \"d\" to toggle debug drawing\n"
            "- press \"a\"/\"b\" to add a player to teamA/teamB\n"
            "- press \"p\" to add a ball\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(-1.25, 0.0, 0.90);
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
		sceneNP = loadPlane("SceneNP", 128, 128);
		PT(Texture) tex =
				TexturePool::load_texture(Filename(string("soccer-field.png")));
        sceneNP.set_texture(tex);
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

		// set playing field
		steerPlugIn->set_playing_field(LPoint3f(-45.5, -35.5, 0.1),
                LPoint3f(45.5, 35.5, 0.1), 0.279);
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
	HandleVehicleData playerAData(0.7, 0, "kinematic", sceneNP,
						steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("a", "addPlayerA", &addPlayerA,
			(void*) &playerAData);
	HandleVehicleData playerBData(0.7, 1, "kinematic", sceneNP,
						steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("b", "addPlayerB", &addPlayerB,
			(void*) &playerBData);
	HandleVehicleData ballData(0.7, 3, "kinematic", sceneNP,
			steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("p", "addBall", &addBall,
			(void*) &ballData);

	// handle OSSteerVehicle(s)' events
	framework.define_key("avoid_neighbor", "handleVehicleEvent",
			&handleVehicleEvent, nullptr);

	// write to bam file on exit
	window->get_graphics_window()->set_close_request_event(
			"close_request_event");
	framework.define_key("close_request_event", "writeToBamFile",
			&writeToBamFileAndExit, (void*) &bamFileName);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(0.0, 180.0, -15.0);
	trackball->set_hpr(0.0, 15.0, 0.0);

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
			"soccer");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value("avoid_neighbor@avoid_neighbor@");
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
		if ((!vehicleAnimCtls[i][0].is_null()) &&
				(!vehicleAnimCtls[i][1].is_null()))
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
	}
	//
	return AsyncTask::DS_cont;
}

// creates a generic vehicle for soccer plug-in
static bool createSoccerVehicle(const Event* e, void* data,
		OSSteerVehicle::OSSteerVehicleType vehicleType)
{
	// set vehicle's type == player
	string typeStr = "player";
    float maxForce = 3000.7;
    float maxSpeed = 10.0;
    float speed = 0.0;
	if (vehicleType == OSSteerVehicle::BALL)
	{
		typeStr = "ball";
        maxForce = 9.0;
        maxSpeed = 9.0;
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

// adds last created player to teamA
void addPlayerA(const Event* e, void* data)
{
	if (not data)
	{
		return;
	}

	if (createSoccerVehicle(e, data, OSSteerVehicle::PLAYER))
	{
		// add to teamA
		steerPlugIn->add_player_to_team(steerVehicles.back(),
				OSSteerPlugIn::TEAM_A);
	}
}

// adds last created player to teamA
void addPlayerB(const Event* e, void* data)
{
	if (not data)
	{
		return;
	}

	if (createSoccerVehicle(e, data, OSSteerVehicle::PLAYER))
	{
		// add to teamB
		steerPlugIn->add_player_to_team(steerVehicles.back(),
				OSSteerPlugIn::TEAM_B);
	}
}

// adds a ball
void addBall(const Event* e, void* data)
{
	if (not data)
	{
		return;
	}

	createSoccerVehicle(e, data, OSSteerVehicle::BALL);
}
