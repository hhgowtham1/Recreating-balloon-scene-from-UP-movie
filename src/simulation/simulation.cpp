	#include "simulation.hpp"
	using namespace cgp;



	// static size_t hash_pair(int ku, int kv)//needs to be edited and should be made to take the position vec3
	// {
	// 	return size_t(ku) + 1000 * size_t(kv);
	// }

	// void constraint_structure::add_fixed_position(int ku, wire_structure const& wire)
	void constraint_structure::add_fixed_position(int ku, particle_structure const& balloon)
	{
	cgp::vec3 temp = balloon.p - vec3(12.f,0.f,0.f);
	fixed_sample[ku] = { ku, temp };
	}
	void constraint_structure::add_fixed_position(int ku, cgp::vec3 const& v)
	{
	fixed_sample[ku] = { ku, v };
	}
	void constraint_structure::update_fixed_position(int ku, particle_structure const & balloon)
	{
	remove_fixed_position(ku);// fixed_sample[ku]
	// cgp::vec3 temp = balloon.p - vec3(12.f,0.f,0.f);
	// fixed_sample[ku] = { ku, temp };
	add_fixed_position(ku,balloon);
	}
	void constraint_structure::update_fixed_position(int ku, cgp::vec3 const& v)
	{
	remove_fixed_position(ku);
	// cgp::vec3 temp = balloon.p - vec3(0,0,3);
	// fixed_sample[ku] = { ku, v };
	add_fixed_position(ku,v);
	}
	void constraint_structure::remove_fixed_position(int ku)
	{
	fixed_sample.erase(ku);
	}



	static vec3 spring_force(const vec3& p_i, const vec3& p_j, float L0, float K)
	{
	vec3 const p = p_i - p_j;
	float const L = norm(p);
	vec3 const u = p / L;

	vec3 const F = -K * (L - L0) * u;
	return F;
	}




	void simulation_compute_force(wire_structure& wire, simulation_parameters const& parameters)
	{
	cgp::numarray<vec3>& force = wire.force;  // Storage for the forces exerted on each vertex

	cgp::numarray<vec3> const& position = wire.position;  // Storage for the positions of the vertices
	cgp::numarray<vec3> const& velocity = wire.velocity;  // Storage for the velocity of the vertices


	size_t const N_total = wire.position.size();       // total number of vertices
	size_t const N = wire.N_samples();                 // number of vertices in one dimension of the grid
	// std::cout<<"N= "<<N<<std::endl;
	// Retrieve simulation parameter
	//  The default value of the simulation parameters are defined in simulation.hpp
	float const K = parameters.K;              // spring stifness
	float const m = parameters.mass_total / N_total; // mass of a particle
	float const mu = parameters.mu;            // damping/friction coefficient
	float const	L0 = 1.0f / (N - 1.0f);        // rest length between two direct neighboring particle

	const vec3 g = { 0,0,-9.81f };

	// Use #prgam omp parallel for - for parallel loops
	#pragma omp parallel for
	for (int k = 0; k < N_total; ++k)
	{
	vec3& f = force.at(k);

	// gravity
	f = m * g; 

	// damping
	f += -mu * m * velocity.at(k);

	//wind
	// float const coeff = dot(parameters.wind.direction, n);
	// f += parameters.wind.magnitude * coeff * n * L0 * L0;
	}

	// Spring
	#pragma omp parallel for
	for (int ku = 1; ku < N-1; ++ku){
	int const offset = ku;// + N * kv;

	vec3& f = force.at(offset);
	vec3 const& p = position.at(offset);
	vec3 const& pnb = position.at(offset-1);
	vec3 const& pna = position.at(offset+1);
	f+= spring_force(p,pnb,L0,K); 
	f+= spring_force(p,pna,L0,K);    
	}
	{
	vec3& f = force.at(0);
	vec3 const& p = position.at(0);
	vec3 const& pnb = position.at(1);
	f+= spring_force(p,pnb,L0,K);
	}
	{
	vec3& f = force.at(N-1);
	vec3 const& p = position.at(N-1);	
	vec3 const& pnb = position.at(N-2);
	f+= spring_force(p,pnb,L0,K);
	}

	}



	void simulation_numerical_integration(wire_structure& wire, simulation_parameters const& parameters, float dt)
	{
	int const N = wire.N_samples();
	int const N_total = wire.position.size();
	float const m = parameters.mass_total/ static_cast<float>(N_total);

	#pragma omp parallel for
	for (int k = 0; k < N_total; ++k)
	{
	vec3 const& f = wire.force.at(k);
	vec3& p = wire.position.at(k);
	vec3& v = wire.velocity.at(k);


	// Standard semi-implicit numerical integration
	v = v + dt * f / m;
	p = p + dt * v;
	}

	}

	void simulation_apply_constraints(wire_structure& wire, constraint_structure const& constraint, particle_structure& particle)
	{
	// Fixed positions of the wire
	for (auto const& it : constraint.fixed_sample) {
	position_contraint c = it.second;
	wire.position.at(c.ku) = c.position; // set the position to the fixed one
	}

	// #ifdef SOLUTION
	const int N = wire.position.size();
	const float epsilon = 1e-2f;
	#pragma omp parallel for
	for (int k = 0; k < N; ++k)
	{
	vec3& p = wire.position.at(k);
	vec3& v = wire.velocity.at(k);

	// Ground constraint
	{
		if (p.z <= constraint.ground_z + epsilon)
		{
			p.z = constraint.ground_z + epsilon;
			v.z = 0.0f;
		}
	}
	// Length constraint
	{
		// float dist = norm(wire.position.at(0) - wire.position.at(wire.N_samples()-1));
		
		// if(dist > particle.l )//&& particle.p.z > 80)
		// {
		// 	particle.v.z = 0.f;
		// 	// particle.m+= 0.1f;
		// 	// std::cout<<"coming here"<<std::endl;
		// }
	}

	}
	}

	// #ifdef SOLUTION
	void collision_sphere_plane(house_structure& particle, const vec3& n, const vec3& p0)
	{
	vec3& p = particle.p;
	vec3& v = particle.v;
	// float const r = particle.r;

	float const epsilon = 1e-5f;
	float const alpha_n = 0.95f;  // attenuation normal
	float const alpha_t = 0.90f;  // attenuation tangential

	float const s = dot(p - p0, n) ;
	if (s < -epsilon)
	{
	vec3 const vn = dot(v, n) * n;
	vec3 const vt = v - vn;

	p = p - (s * n);
	v = -alpha_n * vn + alpha_t * vt;
	}

	}
	/*
	void collision_balloon_balloon(particle_structure& balloon_1, particle_structure& balloon_2)
	{
	vec3& p1 = balloon_1.p;
	vec3& p2 = balloon_2.p;
	vec3& v1 = balloon_1.v;
	vec3& v2 = balloon_2.v;
	float const r1 = balloon_1.r;
	float const r2 = balloon_2.r;
	float const m1 = balloon_1.m;
	float const m2 = balloon_2.m;
	float const k = 50.0f; // Spring constant
	float const b = 10.0f; // Damping coefficient

	float const epsilon = 1e-5f;
	float const alpha = 0.95f;

	vec3 const p12 = p1 - p2;
	float const d12 = norm(p12);

	if (d12 < r1 + r2)
	{
	vec3 const u12 = p12 / d12;
	float const collision_depth = r1 + r2 - d12;

	// Calculate spring force and apply it to the balloons
	vec3 const spring_force = -k * collision_depth * u12 - b * dot(v1 - v2, u12) * u12;
	v1 += (spring_force / m1);
	v2 -= (spring_force / m2);

	// Move the balloons apart slightly to prevent interpenetration
	p1 += (collision_depth / 2.0f + epsilon) * u12;
	p2 -= (collision_depth / 2.0f + epsilon) * u12;

	// Apply collision response to the balloons
	if (norm(v1 - v2) > 0.2f) {
		float const j = dot(v1 - v2, u12);
		v1 = v1 - alpha * j * u12;
		v2 = v2 + alpha * j * u12;
	}
	else // Contact
	{
		v1 = v1 / 1.2f;
		v2 = v2 / 1.2f;
	}
	}
	}
	*/
	void collision_balloon_balloon(particle_structure& balloon_1, particle_structure& balloon_2)
	{
	float COR=0.3f;
	vec3& p1 = balloon_1.p;
	vec3& p2 = balloon_2.p;
	vec3& v1 = balloon_1.v;
	vec3& v2 = balloon_2.v;
	float const r1 = balloon_1.r;
	float const r2 = balloon_2.r;

	float const epsilon = 1e-5f;

	vec3 const p12 = p1 - p2;
	float const d12 = norm(p12);

	if (d12 < r1 + r2)
	{
	vec3 const u12 = p12 / d12;
	float const collision_depth = r1 + r2 - d12;

	p1 += (collision_depth / 2.0f + epsilon) * u12;
	p2 -= (collision_depth / 2.0f + epsilon) * u12;

	vec3 const v12 = v1 - v2;
	float const vn = dot(v12, u12);

	if (vn < 0.0f) {
		float const j = -(1.0f + COR) * vn / (1.0f / balloon_1.m + 1.0f / balloon_2.m);
		vec3 const impulse = j * u12;
		v1 += impulse / balloon_1.m;
		v2 -= impulse / balloon_2.m;
	}
	else // Contact
	{
		v1 = v1 / 1.2f;
		v2 = v2 / 1.2f;
	}

	}
	}


	void collision_sphere_sphere(particle_structure& particle_1, particle_structure& particle_2)
	{
	vec3& p1 = particle_1.p;
	vec3& p2 = particle_2.p;
	vec3& v1 = particle_1.v;
	vec3& v2 = particle_2.v;
	float const r1 = particle_1.r;
	float const r2 = particle_2.r;

	float const epsilon = 1e-5f;
	float const alpha = 0.95f;

	vec3 const p12 = p1 - p2;
	float const d12 = norm(p12);

	if (d12 < r1 + r2)
	{
	vec3 const u12 = p12 / d12;
	float const collision_depth = r1 + r2 - d12;

	p1 += (collision_depth / 2.0f + epsilon) * u12;
	p2 -= (collision_depth / 2.0f + epsilon) * u12;

	if (norm(v1 - v2) > 0.2f) {
		float const j = dot(v1 - v2, u12);
		v1 = v1 - alpha * j * u12;
		v2 = v2 + alpha * j * u12;
	}
	else // Contact
	{
		v1 = v1 / 1.2f;
		v2 = v2 / 1.2f;
	}

	}
	}
	// #endif

	// #ifdef SOLUTION
	/*
	void simulate(std::vector<particle_structure>& balloons, float dt_true)
	{

	vec3 const g = { 0,0,-9.81f };
	size_t const N_substep = 10;
	float const dt = dt_true / N_substep;
	for (size_t k_substep = 0; k_substep < N_substep; ++k_substep)
	{
	size_t const N = balloons.size();
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure& balloon = balloons[k];

		vec3 const f = balloon.m * g;
		// Gravity force
		vec3 const f_gravity = balloon.m * g;

		// Upward force proportional to displacement from initial position
		float const k_up = 10.0f; // Upward force constant
		vec3 const f_up = k_up * (balloon.p - vec3(0.f,0.f,0.f));

		// Total force
		vec3 const f_total = f_gravity + f_up;
		std::cout<<"f_total: "<<f_total<< " f_up "<<f_up<<" f_gravity: "<<f_gravity<<std::endl;
		// Update velocity and position
		balloon.v = (1 - 0.9f * dt) * balloon.v + dt * f_total;
		balloon.p = balloon.p + dt * balloon.v;

	}

	// Collisions between balloons
	for (size_t k1 = 0; k1 < N; ++k1)
		for (size_t k2 = k1 + 1; k2 < N; ++k2)
			collision_balloon_balloon(balloons[k1], balloons[k2]);


	// Collisions with cube
	// const std::vector<vec3> face_normal = { {0, 1,0}, { 1,0,0}, {0,0, 1}, {0,-1,0}, {-1,0,0}, {0,0,-1} };
	// const std::vector<vec3> face_position = { {0,-1,0}, {-1,0,0}, {0,0,-1}, {0, 1,0}, { 1,0,0}, {0,0, 1} };
	// const size_t N_face = face_normal.size();
	// for (size_t k = 0; k < N; ++k)
	// 	for (size_t k_face = 0; k_face < N_face; ++k_face)
	// 		collision_sphere_plane(particles[k], face_normal[k_face], face_position[k_face]);
	}

	}
	*/
	// void simulate(std::vector<particle_structure>& balloons, float dt_true)
	// {
	//     vec3 const g = { 0,0,-9.81f };
	// 	float const rho_air = 1.225f; // density of air in kg/m^3
	// 	float const rho_helium = 0.1785f; // density of helium in kg/m^3

	//     size_t const N_substep = 10;
	//     float const dt = dt_true / N_substep;

	//     for (size_t k_substep = 0; k_substep < N_substep; ++k_substep)
	//     {
	//         size_t const N = balloons.size();

	//         for (size_t k = 0; k < N; ++k)
	//         {
	//             particle_structure& balloon = balloons[k];

	//             // Gravity
	// 			vec3 const f_gravity = balloon.m * g;

	// 			// Buoyancy
	// 			vec3 const buoyancy = -g * (rho_air - rho_helium) * balloon.m;

	// 			// Total force	
	// 			vec3 const f_total = f_gravity + buoyancy;// Update velocity and position
	//             balloon.v = (1 - 0.9f * dt) * balloon.v + dt * f_total;
	//             balloon.p = balloon.p + dt * balloon.v;
	//         }


	//     }
	// }

	bool simulate1(house_structure & house, std::vector<particle_structure>& balloons, float dt_true, std::vector<wire_structure>& wires, std::vector<constraint_structure> &constraints)
	{
	bool toreturn = true;

	// std::cout<<house.p<<" "<<std::flush;
	vec3 const g = { 0,0,-9.81f };
	float const gravity = -9.81f;
	size_t const N_substep = 10;
	float const dt = dt_true / N_substep;
	const float epsilon = 1e-2f;
	float const rho_air = 1.225f; // density of air in kg/m^3
	float const rho_helium = 0.1785f; // density of helium in kg/m^3

	for (size_t k_substep = 0; k_substep < N_substep; ++k_substep)
	{
	size_t const N = balloons.size();
		float const k = 5.f;

		vec3 f = vec3(0.f,0.f,0.f);
		f=  house.m * g;
		vec3 f_cumulative = vec3(0.f,0.f,0.f);
		vec3 v_cumulative = vec3(0.f,0.f,0.f);

		vec3 f_buoyant = vec3(0.f,0.f,0.f);
		f_buoyant =  (rho_air-rho_helium) * -g * balloons[0].vol/100.f;

		vec3 f_gravity = vec3(0.f,0.f,0.f);
		f_gravity.z = (balloons[0].m +rho_helium * balloons[0].vol/100.f)*gravity;

		vec3 Total_tension = -static_cast<float>(N)*(f_buoyant+f_gravity);
	for (size_t i = 0; i < N; ++i)
	{
		particle_structure& balloon = balloons[i];
		wire_structure& wire = wires[i];

	// 		vec3 const wire_dir = normalize(wire.position.at(wire.N_samples()-1) - balloon.p);
	// float const wire_len = norm((wire.position.at(wire.N_samples()-1)) - balloon.p);
	// vec3 const f_tension = k * (wire_len - balloon.l) * wire_dir;
	// f += parameters.wind.magnitude * coeff * n * L0 * L0;
		vec3 f_total= f_buoyant+f_gravity;
		if(norm(balloon.p-vec3{12.f,0.f,0.f} -(house.p + vec3{-8.f,5.f, 80.f + 80.f + 35.f }) ) >balloon.l && norm(Total_tension)< norm(f))
		{
			f_total += Total_tension/static_cast<float>(N) ;
			vec3 temp = balloon.p-vec3{12.f,0.f,0.f} -(house.p + vec3{-8.f,5.f, 80.f + 80.f + 35.f }) ;
			if(temp.x>temp.z || temp.y> temp.z||balloon.p.z<230.f)
			balloon.v-= 2.f*normalize(temp); 

		}
		
		// else 
		//+f_tension; 
		// f_cumulative += f_total;
		// std::cout<<f_total<<" "<<f_tension<<std::endl;
		balloon.v = (1 - 0.9f * dt) * balloon.v + dt * f_total;
		// balloon.v *= 0.1f*vec3(0,-1,0);// parameters.wind.magnitude *parameters.wind.direction;
		balloon.p = balloon.p + dt * balloon.v;
		// if(balloon.v.x>balloon.v.z ||balloon.v.y>balloon.z || balloon)
		// v_cumulative += balloon.v; 
	}

		// std::cout<<"vel : "<<v_cumulative<<" force:  "<<f<<" "<<f_cumulative<<std::endl;
		
		if(norm(Total_tension)> norm(f) && 1)
		{
			// std::cout<<"norms :"<<norm(Total_tension)<<"  "<<norm(f)<<std::flush;
		// house.v = v_cumulative/N;
		std::cout<<"'"<<std::flush;
		// }
		// else
		// {
		f-= Total_tension;
		house.v = (1 - 0.9f * dt) * house.v + dt * f;
		if(house.v.z>0.5f*balloons[N-1].v.z)
		{ toreturn = false;
		if(house.v.z>balloons[N-1].v.z)
			house.v.z = balloons[N-1].v.z; 
		}
		// house.v = house.v *0.0f;
		// if(house.v.z!= 0.0f)
		std::cout<<house.v<<std::endl;
		}
		house.p = house.p + dt * house.v;
		for (size_t i = 0; i < N; ++i)
		{
		constraint_structure &constraint = constraints.at(i);
		particle_structure &particle = balloons.at(i);
		constraint.update_fixed_position(wires.at(i).N_samples()-1, house.p + vec3{-8.f,5.f, 80.f + 80.f + 35.f } );//vec3{30,-36.f,90.f}
		constraint.update_fixed_position(0, particle.p);
		}
	// Collisions between balloons
	for (size_t k1 = 0; k1 < N; ++k1)
		for (size_t k2 = k1 + 1; k2 < N; ++k2)
			collision_balloon_balloon(balloons[k1], balloons[k2]);

	// Ground constraint
	{
		if (house.p.z <= 0.f + epsilon)
		{
			// particle.p.z = 20.f + epsilon;
			house.v.z = 0.0f;
			house.p.z = 0.f + epsilon;
		}
	}
	}
	return toreturn;
	}


	void wire_structure::initialize(int N_samples_edge_arg, cgp::vec3 initial_pos)
	{
	assert_cgp(N_samples_edge_arg <10 , "N_samples_edge=" + str(N_samples_edge_arg) + " should be < 10");

	position.clear();
	velocity.clear();
	force.clear();

	velocity.resize(N_samples_edge_arg);
	force.resize(N_samples_edge_arg);

	numarray<vec3> wire_mesh_data = {initial_pos+vec3{0,0,0},initial_pos+vec3{0,0,1}, initial_pos+vec3{0,0,2}, initial_pos+vec3{0,0,3}, initial_pos+vec3{0,0,4}, initial_pos+vec3{0,0,5}, initial_pos+vec3{0,0,6}, initial_pos+vec3{0,0,7}, initial_pos+vec3{0,0,8}, initial_pos+vec3{0,0,9}, initial_pos+vec3{0,0,10}};

	for(int i=0; i< N_samples_edge_arg; i++ )
	{
	// std::cout<<" printing "<<wire_mesh_data.at(i)<<std::endl;
	position.push_back(wire_mesh_data[i]);
	}
	}


	int wire_structure::N_samples() const
	{
	return position.size();
	}

	void wire_structure_drawable::initialize(int N_samples_edge, cgp::vec3 initial_pos)
	{
	numarray<vec3>  const wire_mesh = {initial_pos+vec3{0,0,0},initial_pos+vec3{0,0,1}, initial_pos+vec3{0,0,2}, initial_pos+vec3{0,0,3}, initial_pos+vec3{0,0,4}, initial_pos+vec3{0,0,5}, initial_pos+vec3{0,0,6}, initial_pos+vec3{0,0,7}, initial_pos+vec3{0,0,8}, initial_pos+vec3{0,0,9}, initial_pos+vec3{0,0,10}};
	drawable.clear();
	drawable.initialize_data_on_gpu(wire_mesh);
	// drawable.material.phong.specular = 0.0f;
	opengl_check;
	}


	void wire_structure_drawable::update(wire_structure const& wire)
	{    
	drawable.vbo_position.update(wire.position.data);
	}

	void draw(wire_structure_drawable const& wire_drawable, environment_generic_structure const& environment)
	{
	draw(wire_drawable.drawable, environment);
	}
	// void draw_wireframe(wire_structure_drawable const& wire_drawable, environment_generic_structure const& environment)
	// {
	//     draw_wireframe(wire_drawable.drawable, environment);
	// }

	bool simulation_detect_divergence(wire_structure const& wire)
	{
	bool simulation_diverged = false;
	const size_t N = wire.position.size();
	for (size_t k = 0; simulation_diverged == false && k < N; ++k)
	{
	const float f = norm(wire.force.at(k));
	const vec3& p = wire.position.at(k);

	if (std::isnan(f)) // detect NaN in force
	{
		std::cout << "\n **** NaN detected in forces" << std::endl;
		simulation_diverged = true;
	}

	if (f > 400.0f) // detect strong force magnitude
	{
		std::cout << "\n **** Warning : Strong force magnitude detected " << f << " at vertex " << k << " ****" << std::endl;
		simulation_diverged = true;
	}

	if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) // detect NaN in position
	{
		std::cout << "\n **** NaN detected in positions" << std::endl;
		simulation_diverged = true;
	}
	}

	return simulation_diverged;
	}