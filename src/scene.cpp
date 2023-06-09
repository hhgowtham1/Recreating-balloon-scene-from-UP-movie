#include "scene.hpp"


using namespace cgp;




void scene_structure::initialize()
{
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_z();
	
	// camera_control.look_at({ 3.0f, 2.0f, 2.0f }, {0,0,0}, {0,0,1});
	// global_frame.initialize_data_on_gpu(mesh_primitive_frame());
	
	// camera_control.camera_model.manipulator_scale_distance_to_center(25.f+20.f);

	timer.event_period = 1.0f;

	// Edges of the containing cube
	//  Note: this data structure is set for display purpose - don't use it to compute some information on the cube - it would be un-necessarily complex
	numarray<vec3> cube_wireframe_data = { {-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
		{-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
		{-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1} };
	cube_wireframe.initialize_data_on_gpu(cube_wireframe_data);
	cube_wireframe.model.scaling = 20.f;
	mesh h_mesh = mesh_load_file_obj("../assets/up_house_s.obj");
	size_t  N = h_mesh.position.size();
	cgp::vec3 center = cgp::vec3(0.,0.,0.);//h_mesh.position.colwise().mean();s
	for (size_t k = 0; k < N; ++k)
	{
		center+= h_mesh.position[k];
	}
	center = center/N ; 	
	for (size_t k = 0; k < N; ++k)
	{
		h_mesh.position[k]= h_mesh.position[k]-vec3{center.x,0.f,center.z} ;
	}
	camera_control.look_at(-6*vec3(center.x,center.y,center.z),center);
	camera_control.camera_model.center_of_rotation = vec3{-8.f,5.f, 80.f + 80.f + 35.f};
	// camera_control.camera_model.manipulator_translate_front(5);
	house_mesh.initialize_data_on_gpu(h_mesh);
	house_mesh.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/woodfloor.jpg");
	house_mesh.model.rotation= rotation_transform::from_axis_angle({ 1,0,0 }, 3.14f/2);
	
	floor.initialize_data_on_gpu(mesh_primitive_quadrangle({ -1.5f,-1.5f,0 }, { -1.5f,1.5f,0 }, { 1.5f,1.5f,0 }, { 1.5f,-1.5f,0 }));
	floor.model.scaling = 150.f;
	floor.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/wood.jpg");
	mesh balloon_mesh= mesh_load_file_obj("../assets/Balloon.obj");
	N = balloon_mesh.position.size();
	center = cgp::vec3(0.,0.,0.);
	for (size_t k = 0; k < N; ++k)
	{
		center+= balloon_mesh.position[k];
	}
	center = center/N ; 	
	for (size_t k = 0; k < N; ++k)
	{
		balloon_mesh.position[k]= balloon_mesh.position[k]-center;
	}
	balloon.initialize_data_on_gpu(balloon_mesh);
	house_.m = 500.f;
	house_.p = {0,0,0};
	house_.v = {0,0,0.f};
}




// Compute a new wire in its initial position (can be called multiple times)
void scene_structure::initialize_wire(int N_sample,particle_structure &particle)//, particle_structure particle)
{
	std::cout<<"."<<std::flush;
	wire_structure wire;
	wire_structure_drawable wire_drawable;
	constraint_structure constraint;
	wire.initialize(N_sample,particle.p);
	wire_drawable.initialize(particle.p);
	wire_drawable.drawable.color = {0.5f,0.5f,0.5f};

	// wire.
	constraint.fixed_sample.clear();
	constraint.add_fixed_position(0, particle);
	cgp::vec3 center = house_.p+cgp::vec3(0.f,0.f,80.f);
	constraint.add_fixed_position(N_sample - 1, center);
	wires.push_back(wire);
	wires_drawable.push_back(wire_drawable);
	constraints.push_back(constraint);
}


void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	global_frame.model.scaling = 80.f;
	if (gui.display_frame)
		draw(global_frame, environment);

	timer.update();

	draw(floor, environment);
	emit_particle();


	// Call the simulation of the particle system
	float const dt = 0.01f * timer.scale;
	if(particles.size()!=0)
	gui.add_balloon =  simulate1(house_, particles ,dt, wires, constraints);

	
	// Display the result
	if(gui.show_balloon)
	{
	balloon_display();
	}
	house_display();
	if (gui.display_frame)
	{
		draw(global_frame, environment);
	}
	
}

void scene_structure::house_display()
{
	
	house_structure const& particle = house_;		
	house_mesh.model.translation = particle.p;
	for (int k=0 ;k< wires.size(); k++)
	{
	constraint_structure &constraint = constraints.at(k);
	constraint.remove_fixed_position(wires.at(k).N_samples()-1);
	constraint.update_fixed_position(wires.at(k).N_samples()-1, house_.p + vec3{-8.f,5.f, 80.f + 80.f + 35.f } );//vec3{30,-36.f,90.f}
	}

	if(gui.show_house)
	{
	draw(house_mesh, environment);
	}
}
void scene_structure::balloon_display()
{
	// Display the particles as balloons
	size_t const N = particles.size();
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure const& particle = particles[k];
		balloon.material.color = particle.c;
		balloon.model.translation = particle.p;
		balloon.model.scaling = 1.f/2.5f;

		draw(balloon, environment);
	}

	for(size_t k=0; k<N; ++k)
	{
		wire_structure &wire = wires.at(k);
		particle_structure &particle = particles.at(k);
		wire_structure_drawable &wire_drawable = wires_drawable.at(k);
		constraint_structure &constraint = constraints.at(k);
		constraint.remove_fixed_position(0);
		constraint.update_fixed_position(0, particle.p);
		int const N_step = 5;
		for (int k_step = 0; /*simulation_running == true &&*/ k_step < N_step; ++k_step)
		{
			// Update the forces on each particle
			simulation_compute_force(wire, parameters);

			// One step of numerical integration
			simulation_numerical_integration(wire, parameters, parameters.dt);
			// Apply the positional (and velocity) constraints
			simulation_apply_constraints(wire, constraint);

			// Check if the simulation has not diverged - otherwise stop it
			bool const simulation_diverged = simulation_detect_divergence(wire);
			if (simulation_diverged) {
				std::cout << "\n *** Simulation has diverged ***" << std::endl;
				std::cout << " > The simulation is stoped" << std::endl;
				// simulation_running = false;
			}
		}

		wire_drawable.update(wire); // update the positions on the GPU

		// Display the wire
		draw(wire_drawable, environment);
	}
		// Display the box in which the particles should stay
	
}

void scene_structure::emit_particle()
{
	// Emit particle with random velocity
	//  Assume first that all particles have the same radius and mass
	static numarray<vec3> const color_lut = { {1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1} };
	if (timer.event && gui.add_balloon) 
	{
		float const theta = rand_interval(0, 2 * Pi);
		vec3 const v = vec3(1.0f * std::cos(theta), 1.0f * std::sin(theta), 20.0f);

		particle_structure particle;
		particle.p = house_.p+vec3{-8.f,5.f, 80.f + 80.f + 35.f};
		particle.r = 3.f;
		particle.c = color_lut[int(rand_interval() * color_lut.size())];
		particle.v = v;
		particle.m = 0.1f; //
		particle.vol = (4.f/3.f) *  Pi * std::pow(particle.r,3); 
		particle.l = 20.f+rand_interval(20.f, 80.f);
		if(particles.size()<10)
		particle.l-= 20.f;
		particles.push_back(particle);
		initialize_wire((particle.l)/10, particle);
		if(particles.size()==1)
		timer.scale =0.1f;

		if(particles.size()<550&&particles.size()>2)
		timer.event_period= 0.02f;
		if(particles.size()==560) timer.event_period = 1.f;

		
	}
}


void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.5f, "%.2f s");
	ImGui::SliderFloat("Time to add new balloon", &timer.event_period, 0.2f, 2.0f, "%0.1f s");
	ImGui::Checkbox("Add balloon", &gui.add_balloon);
	ImGui::Checkbox("Show balloon", &gui.show_balloon);
	// ImGui::Checkbox("Show house", &gui.show_house);
}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);
}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}

