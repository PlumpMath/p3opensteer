/**
 * \file main.cpp
 *
 * \date 2016-05-16
 * \author consultit
 */

#include "common.h"

///global data
PandaFramework framework;
WindowFramework *window;
//models and animations
GeoMipTerrain* terrain;

int main(int argc, char *argv[])
{
	// Load your application's configuration
	load_prc_file_data("", "model-path " + dataDir);
	load_prc_file_data("", "win-size 1024 768");
	load_prc_file_data("", "show-frame-rate-meter #t");
	load_prc_file_data("", "sync-video #t");
	// Setup your application
	framework.open_framework(argc, argv);
	framework.set_window_title("p3recastnavigation");
	window = framework.open_window();
	if (window != (WindowFramework *) NULL)
	{
		cout << "Opened the window successfully!\n";
		window->enable_keyboard();
		window->setup_trackball();
	}

	/// here is room for your own code

	/// typed object init; not needed if you build inside panda source tree
	OSSteerPlugIn::init_type();
	OSSteerVehicle::init_type();
	OSSteerManager::init_type();
	OSSteerPlugIn::register_with_read_factory();
	OSSteerVehicle::register_with_read_factory();
	///

	cout << "create a steer manager" << endl;
	WPT(OSSteerManager)steerMgr = new OSSteerManager(window->get_render());

	cout << "reparent the reference node to render" << endl;
	steerMgr->get_reference_node_path().reparent_to(window->get_render());

	cout << "get a terrain reparented to the reference node" << endl;
	terrain = loadTerrain();

	cout << "create a plug in (it is attached to the reference node)" << endl;
	NodePath plugInNP = steerMgr->create_steer_plug_in();
	PT(OSSteerPlugIn)plugIn = DCAST(OSSteerPlugIn, plugInNP.node());

//	cout << "mandatory: set sceneNP as owner of plugIn" << endl;
//	plugIn->set_owner_node_path(sceneNP);

//	cout << "setup the plugIn with sceneNP as its owner object" << endl;
//	plugIn->setup();

//	cout << "reparent sceneNP to the reference node" << endl;
//	sceneNP.reparent_to(steerMgr->get_reference_node_path());

//	cout << "get the agent model" << endl;
//	NodePath agentNP = window->load_model(framework.get_models(), "eve.egg");
//	agentNP.set_scale(0.40);

//	cout << "create the crowd agent (it is attached to the reference node) and set its position" << endl;
//	NodePath crowdAgentNP = steerMgr->create_crowd_agent("crowdAgent");
//	crowdAgent = DCAST(RNCrowdAgent, crowdAgentNP.node());
//	crowdAgentNP.set_pos(24.0, -20.4, -2.37);

//	cout << "attach the agent model to crowdAgent" << endl;
//	agentNP.reparent_to(crowdAgentNP);

//	cout << "attach the crowd agent to the nav mesh" << endl;
//	plugIn->add_crowd_agent(crowdAgentNP);

	cout << "start the default update task for all plug-ins" << endl;
	steerMgr->start_default_update();

	cout << "DEBUG DRAWING: make the debug reference node path sibling of the reference node" << endl;
	steerMgr->get_reference_node_path_debug().reparent_to(
			window->get_render());
	cout << "enable debug drawing" << endl;
	plugIn->enable_debug_drawing(window->get_camera_group());

	cout << "toggle debug draw" << endl;
	plugIn->toggle_debug_drawing(true);

//	cout << "set crowd agent move target on scene surface" << endl;
//	crowdAgent->set_move_target(LPoint3f(-20.5, 5.2, -2.36));

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-750.0, 400.0, -400.0);
	trackball->set_hpr(0.0, 30.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}
