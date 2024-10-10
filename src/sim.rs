use std::collections::HashMap;

use libm::atan2;
use nalgebra::{Isometry, Isometry3, Vector3};
use pyo3::pyfunction;
use rapier3d_f64::{control::*, prelude::*};

const CAR_GROUP: Group = Group::GROUP_1;

#[pyfunction]
pub fn simulate(initial_state: Vec<f64>, input: Vec<f64>, params: HashMap<String, f64>) -> Vec<f64> {
    let dt = params.get("dt").unwrap(); // Delta time in seconds per timestep
    let l = params.get("l").unwrap(); // Length unit

    let gravity = vector![0.0, -9.81, 0.0];
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let integration_parameters = IntegrationParameters {
        dt: *dt,
        min_ccd_dt: *dt/100.0,
        length_unit: *l,
        ..IntegrationParameters::default()
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    create_floor(&mut bodies, &mut colliders);

    let (mut car, car_handle) = create_car(initial_state, params, &mut bodies, &mut colliders);
    let wheels = car.wheels_mut();
    let engine_force = input[0];
    let steering_angle = input[1];

    println!("Engine force = {}", engine_force);

    wheels[0].engine_force = engine_force;
    wheels[0].steering = steering_angle;
    wheels[1].engine_force = engine_force;
    wheels[1].steering = steering_angle;

    physics_pipeline.step(
        &gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut ccd_solver,
        Some(&mut query_pipeline),
        &physics_hooks,
        &event_handler,
    );
    
    // Update the car again
    car.update_vehicle(
        integration_parameters.dt,
        &mut bodies,
        &colliders,
        &query_pipeline,
        QueryFilter::exclude_dynamic().exclude_rigid_body(car_handle),
    );
    let car_body = &bodies[car_handle];

    // Retrieve the position of the car body
    let translation = car_body.translation();
    let x = translation.x;
    let z = translation.z;

    // Calculate the components of the forwards vector.
    let forwards = car_body.position() * Vector3::ith(car.index_forward_axis, 1.0);
    let forwards_horiozntal = UnitVector::new_normalize(
        vector![forwards.x, 0.0, forwards.z]
    );
    let nx = forwards_horiozntal.x;
    let nz = forwards_horiozntal.z;

    // Calculate the forward and angular velocity of the vehicle
    let linvel = car_body.linvel();
    let v = f64::sqrt(linvel.x.powi(2) + linvel.z.powi(2));
    let w = car_body.angvel().y;

    return vec![x, z, nx, nz, v, w];
}

fn create_floor(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
    let ground_size = 5000.0;
    let ground_height = 1.0;
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size)
    .friction(0.5);

    colliders.insert_with_parent(collider, floor_handle, bodies);
}

fn create_car(initial_state: Vec<f64>, params: HashMap<String, f64>, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> (DynamicRayCastVehicleController, RigidBodyHandle) {
    // Unpack the state vector
    let x0 = initial_state[0];
    let z0 = initial_state[1];
    let nx0 = initial_state[2];
    let nz0 = initial_state[3];
    let v0 = initial_state[4];
    let w0 = initial_state[5];
    let phi0 = atan2(nz0, nx0);

    // Unpack the parameters
    let l = params.get("l").unwrap(); // Length unit
    let m = params.get("m").unwrap();
    let hw = params.get("hw").unwrap()*l;
    let hh = params.get("hh").unwrap()*l;
    let zeta = params.get("zeta").unwrap();

    // Create the chassis rigid body
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![x0, 2.0*hh + hh/4.0, z0])
        .rotation(vector![0.0, phi0, 0.0])
        .linvel(vector![v0*nx0, 0.0, v0*nz0])
        .angvel(vector![0.0, w0, 0.0])
        .linear_damping(*zeta);
    let car_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).mass(*m);

    colliders.insert_with_parent(collider, car_handle, bodies);

    // Add the wheels
    let mut car = DynamicRayCastVehicleController::new(car_handle);
    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };
    let wheel_positions = [
        point![hw * 1.5, -hh, hw],
        point![hw * 1.5, -hh, -hw],
        point![-hw * 1.5, -hh, hw],
        point![-hw * 1.5, -hh, -hw],
    ];

    for pos in wheel_positions {
        car.add_wheel(pos, -Vector::y(), Vector::z(), hh, hh / 4.0, &tuning);
    }

    return (car, car_handle);
}

struct JointCar {
    state: Vec<f64>,
    chassis_handle: RigidBodyHandle,
    wheel_handles: [RigidBodyHandle; 4]
}

impl JointCar {
    fn new(initial_state: Vec<f64>, params: HashMap<String, f64>, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> Self {
        // Unpack the state vector
        let x0 = initial_state[0];
        let z0 = initial_state[1];
        let nx0 = initial_state[2];
        let nz0 = initial_state[3];
        let v0 = initial_state[4];
        let w0 = initial_state[5];
        let phi0 = atan2(nz0, nx0);

        // Unpack the parameters
        let l = params.get("l").unwrap(); // Length unit
        let m = params.get("m").unwrap();
        let hw = params.get("hw").unwrap()*l;
        let hh = params.get("hh").unwrap()*l;

        // Create the chassis rigid body
        let chassis_translation = vector![x0, 2.0*hh + hh/4.0, z0];
        let chassis_rotation = vector![0.0, phi0, 0.0];
        let chassis_position = Isometry3::new(chassis_translation, chassis_rotation);
        let chassis_body_builder = RigidBodyBuilder::dynamic()
            .position(chassis_position)
            .linvel(vector![v0*nx0, 0.0, v0*nz0])
            .angvel(vector![0.0, w0, 0.0]);
        let chassis_handle = bodies.insert(chassis_body_builder);
        let chassis_collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw)
            .mass(*m)
            .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP));

        colliders.insert_with_parent(chassis_collider, chassis_handle, bodies);

        let wheel_offsets = [
            vector![hw * 1.5, -hh, hw],
            vector![hw * 1.5, -hh, -hw],
            vector![-hw * 1.5, -hh, hw],
            vector![-hw * 1.5, -hh, -hw],
        ];
        let wheel_radius = hh/4.0;
        let mut wheel_handles = vec![];
        
        for (id, offset) in wheel_offsets.into_iter().enumerate() {
            let wheel_translation = offset + chassis_translation;
            let wheel_body = RigidBodyBuilder::dynamic()
                .translation(wheel_translation)
                .rotation(vector![std::f64::consts::FRAC_PI_2, phi0, 0.0]);
            let wheel_collider = ColliderBuilder::cylinder(0.05, wheel_radius)
                .friction(1.0)
                .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP));
            let wheel_handle = bodies.insert(wheel_body);

            colliders.insert_with_parent(wheel_collider, wheel_handle, bodies);
            wheel_handles.push(wheel_handle);
        }

        JointCar {
            state: initial_state,
            chassis_handle,
            wheel_handles: wheel_handles.try_into().unwrap()
        }
    }
}