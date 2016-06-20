/**
 * \file main.cpp
 *
 * \date 2016-05-16
 * \author consultit
 */

#include "common.h"

///global data definition
PandaFramework framework;
WindowFramework *window;

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

	cout << "create a steer manager; set root and mask to manage 'kinematic' vehicles" << endl;
	WPT(OSSteerManager)steerMgr = new OSSteerManager(window->get_render(), mask);

	cout << "reparent the reference node to render" << endl;
	steerMgr->get_reference_node_path().reparent_to(window->get_render());

	cout << "get a sceneNP and reparent to the reference node" << endl;
	NodePath sceneNP = loadPlane();
	sceneNP.reparent_to(steerMgr->get_reference_node_path());

	cout << "set sceneNP's collide mask" << endl;
	sceneNP.set_collide_mask(mask);

	cout << "create a plug in (it is attached to the reference node)" << endl;
	NodePath plugInNP = steerMgr->create_steer_plug_in();
	PT(OSSteerPlugIn)plugIn = DCAST(OSSteerPlugIn, plugInNP.node());

	cout << "get the model" << endl;
	NodePath modelNP = window->load_model(framework.get_models(), "eve.egg");
	modelNP.set_scale(10.0);

	cout << "create the steer vehicle (it is attached to the reference node) and set its position" << endl;
	NodePath vehicleNP = steerMgr->create_steer_vehicle("vehicle");
	PT(OSSteerVehicle)vehicle = DCAST(OSSteerVehicle, vehicleNP.node());
	vehicleNP.set_pos(750, 750.0, 0.0);

	cout << "attach the model to steer vehicle" << endl;
	modelNP.reparent_to(vehicleNP);

	cout << "attach the steer vehicle to the plug-in" << endl;
	plugIn->add_steer_vehicle(vehicleNP);

	cout << "start the default update task for all plug-ins" << endl;
	steerMgr->start_default_update();

	cout << "DEBUG DRAWING: make the debug reference node paths sibling of the reference node" << endl;
	steerMgr->get_reference_node_path_debug().reparent_to(
			window->get_render());
	steerMgr->get_reference_node_path_debug_2d().reparent_to(
			window->get_aspect_2d());
	cout << "enable debug drawing" << endl;
	plugIn->enable_debug_drawing(window->get_camera_group());

	cout << "toggle debug draw" << endl;
	plugIn->toggle_debug_drawing(true);

//	cout << "set crowd vehicle move target on scene surface" << endl;
//	vehicle->set_move_target(LPoint3f(-20.5, 5.2, -2.36));

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-750.0, 400.0, -400.0);
	trackball->set_hpr(0.0, 30.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}
