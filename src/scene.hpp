#pragma once


#include "cgp/cgp.hpp"
#include "environment.hpp"

#include "simulation/simulation.hpp"

using cgp::mesh_drawable;


struct gui_parameters {
	bool display_frame = true;
	bool add_balloon = true;
	bool show_house = true;
	bool show_balloon = true;
};

// The structure of the custom scene
struct scene_structure : scene_inputs_generic {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;
	window_structure window;

	mesh_drawable global_frame;          // The standard global frame
	environment_structure environment;   // Standard environment controler
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                  // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	cgp::timer_event_periodic timer;


	std::vector<particle_structure> particles;
	house_structure house_ ;
	// std::vector<wire_structure> wires;
	cgp::mesh_drawable balloon;
	cgp::curve_drawable cube_wireframe;

	cgp::mesh_drawable house_mesh;
	cgp::mesh_drawable floor;

	// wire related structures
	std::vector<wire_structure> wires;                     // The values of the position, velocity, forces, etc, stored as a 2D grid
	std::vector<wire_structure_drawable> wires_drawable;   // Helper structure to display the wire as a mesh
	simulation_parameters parameters;          // Stores the parameters of the simulation (stiffness, mass, damping, time step, etc)
	std::vector<constraint_structure> constraints;           // Handle the parameters of the constraints (fixed vertices, floor and sphere)

	// Helper variables
	bool simulation_running = true;   // Boolean indicating if the simulation should be computed
	cgp::opengl_texture_image_structure wire_texture;             // Storage of the texture ID used for the wire

	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();    // Standard initialization to be called before the animation loop
	void display_frame(); // The frame display to be called within the animation loop
	void display_gui();   // The display of the GUI, also called within the animation loop

void initialize_wire(int N_sample, particle_structure &particle);


	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

	void emit_particle();
	// void simulation_step(float dt);
	
	void balloon_display();
	void house_display();
};





