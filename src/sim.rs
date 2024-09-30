use libm::atan2;
use nalgebra::Vector3;
use pyo3::pyfunction;
use rapier3d_f64::{control::*, prelude::*};

#[pyfunction]
pub fn simulate(X0: Vec<f64>, u: Vec<f64>) -> Vec<f64> {
    let gravity = vector![0.0, -9.81, 0.0];
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let integration_parameters = IntegrationParameters::default();
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

    let (mut car, car_handle) = create_car(X0, &mut bodies, &mut colliders);
    let wheels = car.wheels_mut();
    let engine_force = u[0];
    let steering_angle = u[1];

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
    
    // Update the car
    car.update_vehicle(
        integration_parameters.dt,
        &mut bodies,
        &colliders,
        &query_pipeline,
        QueryFilter::exclude_dynamic().exclude_rigid_body(car_handle),
    );
    let car_body = &bodies[car_handle];

    let translation = car_body.translation();
    let x = translation.x;
    let z = translation.z;

    // Retrieve the forward and angular velocity of the vehicle
    let v = car.current_vehicle_speed;
    let w = car_body.angvel().y;

    // Calculate the components of the forwards vector.
    let forwards = car_body.position() * Vector3::ith(car.index_forward_axis, 1.0);
    let forwards_horiozntal = UnitVector::new_normalize(
        vector![forwards.x, 0.0, forwards.z]
    );
    let nx = forwards_horiozntal.x;
    let nz = forwards_horiozntal.z;

    return vec![x, z, nx, nz, v, w];
}

fn create_floor(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
    let ground_size = 500.0;
    let ground_height = 0.1;
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);

    colliders.insert_with_parent(collider, floor_handle, bodies);
}

fn create_car(X0: Vec<f64>, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> (DynamicRayCastVehicleController, RigidBodyHandle) {
    // Unpack the state vector
    let x0 = X0[0];
    let z0 = X0[1];
    let nx0 = X0[2];
    let nz0 = X0[3];
    let v0 = X0[4];
    let w0 = X0[5];
    let phi = atan2(nz0, nx0);

    // Create the chassis rigid body
    let hw = 0.3;
    let hh = 0.15;
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![x0, 2.0*hh + hh/4.0, z0])
        .rotation(vector![0.0, phi, 0.0])
        .linvel(vector![v0*nx0, 0.0, v0*nz0])
        .angvel(vector![0.0, w0, 0.0]);
    let car_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);

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