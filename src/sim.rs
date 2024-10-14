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
        .translation(vector![x0, hh + hh/4.0, z0])
        .rotation(vector![0.0, phi0, 0.0])
        .linvel(vector![v0*nx0, 0.0, v0*nz0])
        .angvel(vector![0.0, w0, 0.0]);
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

struct Car {
    state: Vec<f64>,
    chassis_handle: RigidBodyHandle,
    wheels: Vec<RigidBodyHandle>,
    axles: Vec<RigidBodyHandle>,
    axle_joints: Vec<ImpulseJointHandle>,
    wheel_joints: Vec<ImpulseJointHandle>
}

impl Car {
    fn new(
        initial_state: Vec<f64>, 
        params: HashMap<String, f64>, 
        bodies: &mut RigidBodySet, 
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet
    ) -> Self {
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
        let mut wheels = vec![];
        let mut axles = vec![];
        let mut axle_joints = vec![];
        let mut wheel_joints = vec![];
        
        for (id, offset) in wheel_offsets.into_iter().enumerate() {
            let is_front = id < 2;

            // Create the wheel
            let wheel_translation = chassis_position * offset;
            let wheel_body = RigidBodyBuilder::dynamic()
                .translation(wheel_translation)
                .rotation(vector![std::f64::consts::FRAC_PI_2, phi0, 0.0]);
            let wheel_collider = ColliderBuilder::cylinder(0.05, wheel_radius)
                .friction(1.0)
                .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP))
                .mass(0.05);
            let wheel_handle = bodies.insert(wheel_body);

            colliders.insert_with_parent(wheel_collider, wheel_handle, bodies);
            wheels.push(wheel_handle);
            
            // Create the "axle"
            let axle_body = RigidBodyBuilder::dynamic()
                .translation(wheel_translation)
                .additional_mass(0.05);

            let axle_handle = bodies.insert(axle_body);

            axles.push(axle_handle);

            // Joint between the chassis and the axle
            let mut locked_axes = JointAxesMask::LIN_AXES
                | JointAxesMask::ANG_X
                | JointAxesMask::ANG_Z;

            if !is_front {
                locked_axes = locked_axes | JointAxesMask::ANG_Y
            }

            let mut axle_joint = GenericJointBuilder::new(locked_axes)
                .motor_position(JointAxis::LinY, 0.0, 1.0e4, 1.0e3)
                .local_anchor1(point![offset.x, -hh, offset.z]);

            if is_front {
                axle_joint = axle_joint.limits(JointAxis::AngY, [-0.7, 0.7]);
            }

            let steering_joint_handle = impulse_joints.insert(
                chassis_handle, 
                axle_handle,
                axle_joint,
                true
            );

            axle_joints.push(steering_joint_handle);

            // Joint between the axle and the wheel
            let wheel_joint = RevoluteJointBuilder::new(Vector3::z_axis());
            let wheel_joint_handle = impulse_joints.insert(axle_handle, wheel_handle, wheel_joint, true);

            wheel_joints.push(wheel_joint_handle);
        }

        Car {
            state: initial_state,
            chassis_handle,
            wheels,
            axles,
            axle_joints,
            wheel_joints
        }
    }

    fn apply_inputs(&mut self, input: Vec<f64>, joints: &mut ImpulseJointSet) {
        let steering_angle = input[0];
        let rpm = input[1];

        for i in 0..2 {
            let axle_joint = joints.get_mut(self.axle_joints[i]).unwrap();

            axle_joint.data
                .set_motor_position(
                    JointAxis::AngY, 
                    steering_angle, 
                    1.0e4, 
                    1.0e3
                );
        }

        for i in 0..2 {
            let wheel_joint = joints.get_mut(self.wheel_joints[i]).unwrap();

            wheel_joint.data
                .set_motor_velocity(
                    JointAxis::AngY, 
                    rpm,
                    1.0e2
                );
        }
    }
}