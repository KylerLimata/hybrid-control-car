use std::{borrow::Borrow, collections::HashMap};

use libm::atan2;
use nalgebra::{Isometry, Isometry3, Vector3};
use pyo3::{pyclass, pyfunction, pymethods};
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
    .friction(1.0);

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

#[pyclass]
#[derive(Clone)]
pub struct SimulationConfig {
    #[pyo3(get, set)]
    pub dt: f64,
    #[pyo3(get, set)]
    pub units_per_meter: i64,
    #[pyo3(get, set)]
    pub chassis_mass: f64,
    #[pyo3(get, set)]
    pub chassis_width: f64,
    #[pyo3(get, set)]
    pub chassis_height: f64,
    #[pyo3(get, set)]
    pub wheel_mass: f64,
    #[pyo3(get, set)]
    pub axle_mass: f64,
    #[pyo3(get, set)]
    pub max_steering_angle: f64,
}

#[pymethods]
impl SimulationConfig {
    #[new]
    fn new() -> Self {
        SimulationConfig {
            dt: 0.01,
            units_per_meter: 1,
            chassis_mass: 1.0,
            chassis_width: 0.1,
            chassis_height: 0.1,
            wheel_mass: 0.01,
            axle_mass: 0.01,
            max_steering_angle: std::f64::consts::PI,
        }
    }
}

#[pyclass]
pub struct SimulationEnvironment {
    config: SimulationConfig,
    car: Option<Car>,
    physics_pipeline: PhysicsPipeline,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    gravity: Vector3<f64>,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase, 
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
}

#[pymethods]
impl SimulationEnvironment {
    #[new]
    fn new(config: SimulationConfig) -> Self {
        let integration_parameters = IntegrationParameters {
            dt: config.dt,
            min_ccd_dt: config.dt/100.0,
            length_unit: config.units_per_meter as f64,
            ..IntegrationParameters::default()
        };
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        create_floor(&mut bodies, &mut colliders);

        SimulationEnvironment {
            config,
            car: None,
            physics_pipeline: PhysicsPipeline::new(),
            bodies: bodies,
            colliders: colliders,
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
        }
    }

    fn step(&mut self, initial_state: Vec<f64>, input: Vec<f64>) -> Vec<f64> {
        // Ensure passed initial state and input are of the right size
        if initial_state.len() != 6 {
            panic!("State vec should be of size 6.")
        }
        if input.len() != 2 {
            panic!("Input vec should be of size 2.")
        }

        let mut should_replace = false;

        if let Some(car) = &self.car {
            for i in 0..6 {
                if (initial_state[i] - car.state[i]).abs() > 1e-3 {
                    should_replace = true;
                }
            }
        }

        if should_replace || self.car.is_none() {
            if let Some(car) = &mut self.car.replace(Car::new(
                initial_state, 
                &self.config, 
                &mut self.bodies, 
                &mut self.colliders, 
                &mut self.impulse_joints
            )) {
                car.remove(
                    &mut self.bodies, 
                    &mut self.colliders, 
                    &mut self.islands, 
                    &mut self.impulse_joints, 
                    &mut self.multibody_joints
                );
            }
        }

        if let Some(car) = &mut self.car {
            car.apply_inputs(input, &mut self.impulse_joints);
        }

        let physics_hooks = ();
        let event_handler = ();

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        if let Some(car) = &mut self.car {
            car.update_state(&mut self.bodies);

            let chassis_body = self.bodies.get(car.chassis_handle).unwrap();
            let hh = self.config.chassis_height/2.0;

            assert!(chassis_body.translation().y > 0.0);

            return car.state.clone();
        }

        panic!("Somehow, the car vanished")
    }
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
    fn new(initial_state: Vec<f64>, config: &SimulationConfig, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, impulse_joints: &mut ImpulseJointSet) -> Self {
        // Unpack the state vector
        let x0 = initial_state[0];
        let z0 = initial_state[1];
        let nx0 = initial_state[2];
        let nz0 = initial_state[3];
        let v0 = initial_state[4];
        let w0 = initial_state[5];
        let phi0 = atan2(nz0, nx0);

        // Unpack the parameters
        // let l = config.units_per_meter as f64;
        let hw = config.chassis_width/2.0;
        let hh = config.chassis_height/2.0;

        // Calculate the initial position of the vehicle (translation + rotation)
        let car_translation = vector![x0, 0.0, z0];
        let car_axisangle = vector![0.0, phi0, 0.0];
        let car_isometry = Isometry3::new(
            car_translation,
            car_axisangle
        );

        // Create the chassis rigid body
        let chassis_offset = vector![0.0, hh + hh/4.0, 0.0];
        let chassis_translation = chassis_offset + car_translation;
        let chassis_isometry = Isometry3::new(
            chassis_translation,
            car_axisangle
        );
        let chassis_body_builder = RigidBodyBuilder::dynamic()
            .position(chassis_isometry)
            .linvel(vector![v0*nx0, 0.0, v0*nz0])
            .angvel(vector![0.0, w0, 0.0])
            .enabled_rotations(
                false, 
                true, 
                false
            )
            .can_sleep(false);
        let chassis_handle = bodies.insert(chassis_body_builder);
        let chassis_collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw)
            .mass(config.chassis_mass)
            .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP));

        colliders.insert_with_parent(chassis_collider, chassis_handle, bodies);
        assert!(chassis_translation.y == hh + hh/4.0);

        let wheel_radius = hh/4.0;
        let wheel_offsets = [
            vector![hw * 1.5, wheel_radius, hw],
            vector![hw * 1.5, wheel_radius, -hw],
            vector![-hw * 1.5, wheel_radius, hw],
            vector![-hw * 1.5, wheel_radius, -hw],
        ];
        let mut wheels = vec![];
        let mut axles = vec![];
        let mut axle_joints = vec![];
        let mut wheel_joints = vec![];
        
        for (id, offset) in wheel_offsets.into_iter().enumerate() {
            let is_front = id < 2;

            // Create the wheel
            let wheel_translation = (car_isometry * offset) + car_translation;
            
            let wheel_body = RigidBodyBuilder::dynamic()
                .translation(wheel_translation)
                .can_sleep(false);
            let wheel_collider = ColliderBuilder::ball(wheel_radius)
                .friction(1.0)
                .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP))
                .mass(config.wheel_mass);
            let wheel_handle = bodies.insert(wheel_body);

            colliders.insert_with_parent(wheel_collider, wheel_handle, bodies);
            wheels.push(wheel_handle);
            
            // Create the "axle"
            let axle_body = RigidBodyBuilder::dynamic()
                .translation(wheel_translation)
                .additional_mass(config.axle_mass);
            let axle_handle = bodies.insert(axle_body);

            axles.push(axle_handle);

            // Joint between the chassis and the axle
            let mut locked_axes = JointAxesMask::LIN_AXES
                | JointAxesMask::ANG_X
                | JointAxesMask::ANG_Z;

            if !is_front {
                locked_axes = locked_axes | JointAxesMask::ANG_Y
            }

            let axle_joint_attachment = offset - chassis_offset;
            let mut axle_joint = GenericJointBuilder::new(locked_axes)
                .motor_position(JointAxis::LinY, 0.0, 1.0e4, 1.0e3)
                .local_anchor1(axle_joint_attachment.into());

            if is_front {
                axle_joint = axle_joint.limits(
                    JointAxis::AngY, 
                    [-config.max_steering_angle, config.max_steering_angle]
                );
            }

            let axle_joint_handle = impulse_joints.insert(
                chassis_handle, 
                axle_handle,
                axle_joint,
                true
            );

            axle_joints.push(axle_joint_handle);

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

    fn apply_inputs(&mut self, input: Vec<f64>, impulse_joints: &mut ImpulseJointSet) {
        let rpm = input[0];
        let steering_angle = input[1];
        

        for i in 0..2 {
            let axle_joint = impulse_joints.get_mut(self.axle_joints[i]).unwrap();

            axle_joint.data
                .set_motor_position(
                    JointAxis::AngY,
                    steering_angle, 
                    1.0e4,
                    1.0e3
                );
        }

        for i in 0..2 {
            let wheel_joint = impulse_joints.get_mut(self.wheel_joints[i]).unwrap();

            wheel_joint.data
                .set_motor_velocity(
                    JointAxis::AngZ, 
                    rpm,
                    1.0e2
                );
        }
    }

    fn update_state(&mut self, bodies: &mut RigidBodySet) {
        let chassis = &bodies[self.chassis_handle];

        // Retrieve the position of the car body
        let translation = chassis.translation();
        let x = translation.x;
        let z = translation.z;

        // Calculate the components of the forwards vector.
        let forwards = chassis.position() * Vector3::x_axis();
        let nx = forwards.x;
        let nz = forwards.z;

        // Calculate the forward and angular velocity of the vehicle
        let linvel = chassis.linvel();
        let v = f64::sqrt(linvel.x.powi(2) + linvel.z.powi(2));
        let w = chassis.angvel().y;

        self.state = vec![x, z, nx, nz, v, w];
    }
    
    fn remove(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, islands: &mut IslandManager, impulse_joints: &mut ImpulseJointSet, multibody_joints: &mut MultibodyJointSet) {
        for i in 0..4 {            
            bodies.remove(
                self.wheels[i], 
                islands, 
                colliders, 
                impulse_joints, 
                multibody_joints, 
                true
            );
            bodies.remove(
                self.axles[i], 
                islands, 
                colliders, 
                impulse_joints, 
                multibody_joints, 
                true
            );
        }

        bodies.remove(
            self.chassis_handle, 
            islands, 
            colliders, 
            impulse_joints, 
            multibody_joints, 
            true
        );
    }
}