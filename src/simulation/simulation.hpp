#pragma once

#include "cgp/cgp.hpp"
#include "../environment.hpp"

struct particle_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed

    cgp::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
    float vol;   // Volume
    float l;     // wire length
    // wire_structure wire;
};

struct house_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed
    // cgp::vec3 f; // Forces acting on the house(chimney)
    // cgp::vec3 c; // Color
    // float r;     // Radius
    float m;     // mass

    // wire_structure wire;
};





// void draw_wireframe(wire_structure_drawable const& wire_drawable, environment_generic_structure const& environment);

// Parameter attached to a fixed vertex (ku,kv) coordinates + 3D position
struct position_contraint {
	int ku;
	cgp::vec3 position;
};


struct constraint_structure
{
	float ground_z = 0.0f;                                   // Height of the flood
	// sphere_parameter sphere = { {0.1f, 0.5f, 0.0f}, 0.15f }; // Colliding sphere
	
	std::map<size_t, position_contraint> fixed_sample; // Storage of all fixed position of the wire

	// Add a new fixed position
	// void add_fixed_position(int ku, wire_structure const& wire);
	void add_fixed_position(int ku, particle_structure const& balloon);
    void add_fixed_position(int ku, cgp::vec3 const& v);
	// Remove a fixed position
	void remove_fixed_position(int ku);
    void update_fixed_position(int ku, particle_structure const & balloon);
    void update_fixed_position(int ku, cgp::vec3 const& v);
};

struct wire_structure
{    
    // Buffers are stored as 2D grid that can be accessed as grid(ku,kv)
    cgp::numarray<cgp::vec3> position;  
    cgp::numarray<cgp::vec3> velocity;  
    cgp::numarray<cgp::vec3> force;


    
    void initialize(int N_sample_edge, cgp::vec3 initial_pos);  // Initialize a square flat wire
    int N_samples() const;      // Number of vertex along one dimension of the grid
};

void simulate(std::vector<particle_structure>& particles, float dt);
bool simulate1(house_structure & house, std::vector<particle_structure>& particles, float dt, std::vector<wire_structure>& wires, std::vector<constraint_structure> &constraints);


void collision_sphere_plane(house_structure& particle, cgp::vec3 const& n_plane, cgp::vec3 const& p0_plane);
void collision_sphere_sphere(particle_structure& particle_1, particle_structure& particle_2);
void collision_balloon_balloon(particle_structure& particle_1, particle_structure& particle_2);

struct wire_structure_drawable
{
    cgp::curve_drawable drawable;

    void initialize(int N_sample_edge, cgp::vec3 initial_pos);
    void update(wire_structure const& wire);
};

void draw(wire_structure_drawable const& wire_drawable, environment_generic_structure const& environment);



struct simulation_parameters
{
    float dt = 0.005f;        // time step for the numerical integration
    float mass_total = 0.1f; // total mass of the wire
    float K = 5.0f;         // stiffness parameter
    float mu = 15.0f;        // damping parameter

    //  Wind magnitude and direction
    struct {
        float magnitude = 1.0f;
        cgp::vec3 direction = { 0,-1,0 };
    } wind;
};


// Fill the forces in the wire given the position and velocity
void simulation_compute_force(wire_structure& wire, simulation_parameters const& parameters);

// Perform 1 step of a semi-implicit integration with time step dt
void simulation_numerical_integration(wire_structure& wire, simulation_parameters const& parameters, float dt);

// Apply the constraints (fixed position, obstacles) on the wire position and velocity
void simulation_apply_constraints(wire_structure& wire, constraint_structure const& constraint, particle_structure& particle);

// Helper function that tries to detect if the simulation diverged 
bool simulation_detect_divergence(wire_structure const& wire);