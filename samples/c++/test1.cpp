/**
 * \file main.cpp
 *
 * \date 2016-05-13
 * \author consultit
 */

#include <pandaFramework.h>
#include <auto_bind.h>
#include <load_prc_file.h>
#include <osSteerManager.h>
#include <osSteerPlugIn.h>
#include <osSteerVehicle.h>
#include <collisionRay.h>
#include <mouseWatcher.h>
#include <random>
#include <bamFile.h>

#include "main.h"

///functions' declarations

///global data
PandaFramework framework;
WindowFramework *window;
CollideMask mask = BitMask32(0x10);

//models and animations

//bame file
string bamFileName("steer_plugin.boo");
//support
random_device rd;

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
//	OSSteerPlugIn::init_type();
//	OSSteerVehicle::init_type();
//	OSSteerManager::init_type();
//	OSSteerPlugIn::register_with_read_factory();
//	OSSteerVehicle::register_with_read_factory();
	///
	// print some help to screen
	PT(TextNode) text;
	text = new TextNode("Help");
	text->set_text(
			"- press \"d\" to toggle debug drawing\n"
			"- press \"s\" to toggle setup cleanup\n"
			"- press \"p\" to place agents randomly\n"
			"- press \"t\" to set agents' target under mouse cursor\n"
			"- press \"o\" to add obstacle under mouse cursor\n"
			"- press \"shift-o\" to remove obstacle under mouse cursor\n");
	NodePath textNodePath = window->get_aspect_2d().attach_new_node(text);
	textNodePath.set_pos(-1.25, 0.0, 0.9);
	textNodePath.set_scale(0.035);

	// place camera trackball (local coordinate)
	PT(Trackball)trackball = DCAST(Trackball, window->get_mouse().find("**/+Trackball").node());
	trackball->set_pos(-10.0, 90.0, -2.0);
	trackball->set_hpr(0.0, 15.0, 0.0);

	// do the main loop, equals to call app.run() in python
	framework.main_loop();

	return (0);
}

///functions' definitions
