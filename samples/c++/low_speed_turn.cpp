/**
 * \file main.cpp
 *
 * \date 2016-05-26
 * \author consultit
 */

#include "common.h"

///global data/functions
PandaFramework framework;
WindowFramework *window;
PT(OSSteerPlugIn)plugIn;
//
void setParametersBeforeCreation();
void toggleDebugDraw(const Event*, void*);

int main(int argc, char *argv[])
{
	startFramework(argc, argv);

	/// here is room for your own code
	// print some help to screen
	PT(TextNode) text;
	text = new TextNode("Help");
	text->set_text(
			"- press \"d\" to toggle debug drawing\n"
			"- press \"s\" to toggle setup cleanup\n"
			"- press \"p\" to place agents randomly\n"
			"- press \"t\", \"y\" to set agents' targets under mouse cursor\n"
			"- press \"o\" to add obstacle under mouse cursor\n"
			"- press \"shift-o\" to remove obstacle under mouse cursor\n");
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

	// reparent the reference node to render
	steerMgr->get_reference_node_path().reparent_to(window->get_render());

	// get a sceneNP and reparent to the reference node
	NodePath sceneNP = loadPlane();
	sceneNP.reparent_to(steerMgr->get_reference_node_path());

	// set sceneNP's collide mask
	sceneNP.set_collide_mask(mask);

	// create the default plug-in (attached to the reference node): 'one turning'
	NodePath plugInNP = steerMgr->create_steer_plug_in();
	plugIn = DCAST(OSSteerPlugIn, plugInNP.node());

	// get the model
	NodePath modelNP = window->load_model(framework.get_models(), "eve.egg");
	modelNP.set_scale(0.25);

	// create the steer vehicle (it is attached to the reference node) and set its position
	NodePath vehicleNP = steerMgr->create_steer_vehicle("vehicle");
	PT(OSSteerVehicle)vehicle = DCAST(OSSteerVehicle, vehicleNP.node());
	vehicleNP.set_pos(5.0, -8.0, 0.1);

	// attach the model to steer vehicle
	modelNP.reparent_to(vehicleNP);

	// attach the steer vehicle to the plug-in
	plugIn->add_steer_vehicle(vehicleNP);

	// start the default update task for all plug-ins
	steerMgr->start_default_update();

	// DEBUG DRAWING: make the debug reference node paths sibling of the reference node
	steerMgr->get_reference_node_path_debug().reparent_to(window->get_render());
	steerMgr->get_reference_node_path_debug_2d().reparent_to(
			window->get_aspect_2d());
	// enable debug drawing
	plugIn->enable_debug_drawing(window->get_camera_group());

	/// set events' callbacks
	// toggle debug draw
	bool toggleDebugFlag = false;
	framework.define_key("d", "toggleDebugDraw", &toggleDebugDraw,
			(void*) &toggleDebugFlag);

	// handle OSSteerVehicle(s)' events
	framework.define_key("move-event", "handleVehicleEvent",
			&handleVehicleEvent, nullptr);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(0.0, 30.0, 0.0);
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

	// set vehicle type
	steerMgr->set_parameter_value(OSSteerManager::STEERVEHICLE, "vehicle_type",
			"low_speed_turn");

	// set vehicle throwing events
	valueList.clear();
	valueList.add_value("move@move-event@0.5");
	steerMgr->set_parameter_values(OSSteerManager::STEERVEHICLE,
			"thrown_events", valueList);
	//
	printCreationParameters();
}

// toggle debug draw
void toggleDebugDraw(const Event* e, void* data)
{
	if(! plugIn)
	{
		return;
	}
	bool* toggleDebugFlag = reinterpret_cast<bool*>(data);
	*toggleDebugFlag = not *toggleDebugFlag;
	plugIn->toggle_debug_drawing(*toggleDebugFlag);
}
