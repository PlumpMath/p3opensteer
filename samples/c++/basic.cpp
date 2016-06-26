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
	startFramework(argc, argv);

	/// here is room for your own code

	cout << "create a steer manager; set root and mask to manage 'kinematic' vehicles" << endl;
	WPT(OSSteerManager)steerMgr = new OSSteerManager(window->get_render(), mask);

	cout << "reparent the reference node to render" << endl;
	steerMgr->get_reference_node_path().reparent_to(window->get_render());

	cout << "get a sceneNP and reparent to the reference node" << endl;
	NodePath sceneNP = loadPlane();
	sceneNP.reparent_to(steerMgr->get_reference_node_path());

	cout << "set sceneNP's collide mask" << endl;
	sceneNP.set_collide_mask(mask);

	cout << "create the default plug-in (attached to the reference node): 'one turning'" << endl;
	NodePath plugInNP = steerMgr->create_steer_plug_in();
	PT(OSSteerPlugIn)plugIn = DCAST(OSSteerPlugIn, plugInNP.node());

	cout << "get the model" << endl;
	NodePath modelNP = window->load_model(framework.get_models(), "eve.egg");
	modelNP.set_scale(0.25);

	cout << "create the steer vehicle (it is attached to the reference node) and set its position" << endl;
	NodePath vehicleNP = steerMgr->create_steer_vehicle("vehicle");
	PT(OSSteerVehicle)vehicle = DCAST(OSSteerVehicle, vehicleNP.node());
	vehicleNP.set_pos(5.0, -8.0, 0.1);

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

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(0.0, 30.0, 0.0);
	trackball->set_hpr(0.0, 20.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}
