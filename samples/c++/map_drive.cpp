/**
 * \file map_drive.cpp
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
PT(TextureStage)rttTexStage;
//
void setParametersBeforeCreation();
AsyncTask::DoneStatus updatePlugIn(GenericAsyncTask*, void*);
void debugDrawToTexture(const Event*, void*);
void onTextureReady(const Event*, void*);
void togglePredictionType(const Event*, void*);

int main(int argc, char *argv[])
{
	string msg("'map drive'");
	startFramework(argc, argv, msg);

	/// here is room for your own code
	// print some help to screen
	PT(TextNode)text;
	text = new TextNode("Help");
	text->set_text(
            msg + "\n\n"
            "- press \"d\" to toggle debug drawing\n"
            "- press \"o\"/\"shift-o\" to add/remove obstacle\n"
            "- press \"t\" to (re)draw the map of the path\n"
            "- press \"a\" to add vehicle\n"
			"- press \"p\" to toggle map prediction type\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(0.25, 0.0, 0.8);
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
        sceneNP = loadTerrainLowPoly("SceneNP", 64, 24);
		// and reparent to the reference node
		sceneNP.reparent_to(steerMgr->get_reference_node_path());

		// set the texture stage used for debug draw texture
		rttTexStage = new TextureStage("rttTexStage");

		// set sceneNP's collide mask
		sceneNP.set_collide_mask(mask);

		// set creation parameters as strings before plug-in/vehicles creation
		cout << endl << "Current creation parameters:";
		setParametersBeforeCreation();

		// create the plug-in (attached to the reference node)
		NodePath plugInNP = steerMgr->create_steer_plug_in();
		steerPlugIn = DCAST(OSSteerPlugIn, plugInNP.node());

		// set the pathway
		ValueList<LPoint3f> pointList;
		ValueList<float> radiusList;
        pointList.add_value(LPoint3f(-41.80, 34.46, -0.17));
        radiusList.add_value(7.0);
        pointList.add_value(LPoint3f(-2.21, 49.15, -0.36));
        radiusList.add_value(8.0);
        pointList.add_value(LPoint3f(10.78, 16.65, 0.14));
        radiusList.add_value(9.0);
        pointList.add_value(LPoint3f(40.44, 17.58, -0.22));
        radiusList.add_value(9.0);
        pointList.add_value(LPoint3f(49.04, -22.15, -0.60));
        radiusList.add_value(8.0);
        pointList.add_value(LPoint3f(13.99, -52.70, 0.39));
        radiusList.add_value(8.0);
        pointList.add_value(LPoint3f(-3.46, -31.90, 0.71));
        radiusList.add_value(7.0);
        pointList.add_value(LPoint3f(-30.0, -39.97, -0.35));
        radiusList.add_value(6.0);
        pointList.add_value(LPoint3f(-47.12, -17.31, -0.43));
        radiusList.add_value(6.0);
        pointList.add_value(LPoint3f(-51.31, 9.08, -0.25));
        radiusList.add_value(7.0);
		steerPlugIn->set_pathway(pointList, radiusList, false, true);
		// make the map
		steerPlugIn->make_map(200);
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

		// restore the texture stage used for debug draw texture
		rttTexStage = sceneNP.find_all_texture_stages().find_texture_stage(
				"rttTexStage");
		if (not rttTexStage)
		{
			rttTexStage = new TextureStage("rttTexStage");
		}

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
	// print debug draw to texture
	framework.define_key("t", "debugDrawToTexture", &debugDrawToTexture,
			(void*) NULL);
	framework.define_key("debug_drawing_texture_ready", "onTextureReady",
			&onTextureReady, (void*) rttTexStage.p());

	/// set events' callbacks
	// toggle debug draw
	toggleDebugFlag = false;
	framework.define_key("d", "toggleDebugDraw", &toggleDebugDraw,
			(void*) steerPlugIn.p());

	// handle addition steer vehicles, models and animations
	HandleVehicleData vehicleData(0.4, 4, "kinematic", sceneNP,
						steerPlugIn, steerVehicles, vehicleAnimCtls);
	framework.define_key("a", "addVehicle", &handleVehicles,
			(void*) &vehicleData);

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
	framework.define_key("path_following", "handleVehicleEvent",
			&handleVehicleEvent, NULL);

	// write to bam file on exit
	window->get_graphics_window()->set_close_request_event(
			"close_request_event");
	framework.define_key("close_request_event", "writeToBamFile",
			&writeToBamFileAndExit, (void*) &bamFileName);

	// map drive specifics: toggle prediction type
	framework.define_key("p", "togglePredictionType", &togglePredictionType,
			NULL);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(0.0, 160.0, -5.0);
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
			"map_drive");

	// set vehicle's type, mass, speed
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "vehicle_type",
			"map_driver");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "max_speed",
			"20.0");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "max_force",
			"8.0");
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "up_axis_fixed",
			"true");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value(
			"avoid_obstacle@avoid_obstacle@1.0:path_following@path_following@1.0");
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

// debug draw to texture
void debugDrawToTexture(const Event* e, void* data)
{
	steerPlugIn->debug_drawing_to_texture(sceneNP,
			window->get_graphics_output());
}

// debug drawing texture is ready
void onTextureReady(const Event* e, void* data)
{
	PT(Texture)texture = DCAST(Texture,
			e->get_parameter(0).get_ptr());
	PT(TextureStage)rttTexStage = reinterpret_cast<TextureStage*>(data);
	//set up texture where to render
	sceneNP.clear_texture(rttTexStage);
	rttTexStage->set_mode(TextureStage::M_modulate);
	// take into account sceneNP dimensions
	sceneNP.set_tex_offset(rttTexStage, 0.5, 0.5);
	sceneNP.set_tex_scale(rttTexStage, 1.0 / 128.0, 1.0 / 128.0);
	sceneNP.set_tex_gen(rttTexStage, TexGenAttrib::M_world_position);
	sceneNP.set_texture(rttTexStage, texture, 10);
}

// toggle prediction type
void togglePredictionType(const Event*, void*)
{
	OSSteerPlugIn::OSMapPredictionType predictionType =
			steerPlugIn->get_map_prediction_type();
	if (predictionType == OSSteerPlugIn::CURVED_PREDICTION)
	{
		steerPlugIn->set_map_prediction_type(OSSteerPlugIn::LINEAR_PREDICTION);
		cout << "prediction type: linear" << endl;
	}
	else
	{
		steerPlugIn->set_map_prediction_type(OSSteerPlugIn::CURVED_PREDICTION);
		cout << "prediction type: curved" << endl;
	}
}
