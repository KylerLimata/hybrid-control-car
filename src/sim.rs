use libm::atan2;
use nalgebra::{Isometry3, Vector3};
use pyo3::{pyclass, pymethods};
use rapier3d_f64::prelude::*;

const CAR_GROUP: Group = Group::GROUP_1;

fn create_floor(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
    let ground_size = 5000.0;
    let ground_height = 10.0;
    let ground_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let ground_handle = bodies.insert(ground_body);
    let ground_collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size)
    .friction(1.0);

    colliders.insert_with_parent(ground_collider, ground_handle, bodies);
}

#[pyclass]
#[derive(Clone)]
pub struct SimulationConfig {
    #[pyo3(get, set)]
    pub dt: f64,
    #[pyo3(get, set)]
    pub units_per_meter: i64,
    #[pyo3(get, set)]
    pub chassis_width: f64,
    #[pyo3(get, set)]
    pub chassis_length: f64,
    #[pyo3(get, set)]
    pub chassis_height: f64,
    #[pyo3(get, set)]
    pub wheel_radius: f64,
    #[pyo3(get, set)]
    pub chassis_density: f64,
    #[pyo3(get, set)]
    pub wheel_density: f64,
    #[pyo3(get, set)]
    pub axle_density: f64,
    #[pyo3(get, set)]
    pub suspension_height: f64,
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
            chassis_width: 1.0,
            chassis_length: 2.0,
            chassis_height: 1.0,
            wheel_radius: 0.5,
            chassis_density: 100.0,
            wheel_density: 100.0,
            axle_density: 100.0,
            suspension_height: 0.2,
            max_steering_angle: std::f64::consts::PI,
        }
    }
}

#[pyclass]
pub struct SimulationEnvironment {
    config: SimulationConfig,
    car: Car,
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
        let mut impulse_joints = ImpulseJointSet::new();
        let car = Car::new(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], &config, &mut bodies, &mut colliders, &mut impulse_joints);

        create_floor(&mut bodies, &mut colliders);

        let mut env = SimulationEnvironment {
            config,
            car: car,
            physics_pipeline: PhysicsPipeline::new(),
            bodies: bodies,
            colliders: colliders,
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: impulse_joints,
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
        };

        env.reset_car(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

        return env;
    }

    fn step(&mut self, initial_state: Vec<f64>, input: Vec<f64>) -> Vec<f64> {
        // Ensure passed initial state and input are of the right size
        if initial_state.len() != 7 {
            panic!("State vec should be of size 7.")
        }
        if input.len() != 2 {
            panic!("Input vec should be of size 2.")
        }

        let mut should_replace = false;

        for i in 0..6 {
            if (initial_state[i] - &self.car.state[i]).abs() > 1e-3 {
                should_replace = true;
            }
        }

        if should_replace {
            self.reset_car(initial_state);
        }

        self.car.apply_inputs(input.clone(), &mut self.bodies, &mut self.impulse_joints);
        self.step_pipeline();
        self.car.update_state(&self.config, &mut self.bodies);

        for i in 0..4 {
            let wheel = &self.bodies[self.car.wheels[i]];
            let y = wheel.translation().y;
            let hh = self.config.chassis_height/2.0;
            let radius = hh/4.0;

            if y < radius {
                panic!("Wheel id {} has fallen through the floor! wheel.y = {}", i, y)
            }
        }

        return self.car.state.clone();
    }

    fn reset_car(&mut self, initial_state: Vec<f64>) {
        self.car.remove(
            &mut self.bodies, 
            &mut self.colliders, 
            &mut self.islands, 
            &mut self.impulse_joints, 
            &mut self.multibody_joints
        );

        self.car = Car::new(
            initial_state,
            &self.config,
            &mut self.bodies, 
            &mut self.colliders, 
            &mut self.impulse_joints
        );
        let mut sleeping = false;

        while !sleeping {
            self.step_pipeline();

            let chassis = self.bodies.get_mut(self.car.chassis_handle).unwrap();
            sleeping = chassis.is_sleeping();
        }

        self.car.unlock(&mut self.bodies);
    }

    fn step_pipeline(&mut self) {
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
        let hl = config.chassis_length/2.0;
        let hw = config.chassis_width/2.0;
        let hh = config.chassis_height/2.0;
        let r = config.wheel_radius;
        let placement_offset = r + config.suspension_height;

        // Calculate the initial position of the vehicle (translation + rotation)
        let car_translation = vector![x0, placement_offset, z0];
        let car_axisangle = vector![0.0, phi0, 0.0];
        let car_isometry = Isometry3::new(
            car_translation,
            car_axisangle
        );

        // Create the chassis rigid body
        let chassis_offset = vector![0.0, hh + r, 0.0];
        let chassis_translation = chassis_offset + car_translation;
        let chassis_isometry = Isometry3::new(
            chassis_translation,
            car_axisangle
        );
        let chassis_body_builder = RigidBodyBuilder::dynamic()
            .position(chassis_isometry)
            .linvel(vector![v0*nx0, 0.0, v0*nz0])
            .angvel(vector![0.0, w0, 0.0])
            .enabled_rotations(false, false, false)
            .enabled_translations(false, true, false);
        let chassis_handle = bodies.insert(chassis_body_builder);
        let chassis_collider = ColliderBuilder::cuboid(hl, hh, hw)
            .density(config.chassis_density)
            .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP));

        colliders.insert_with_parent(chassis_collider, chassis_handle, bodies);
        assert!(chassis_translation.y == hh + r + placement_offset);

        let wheel_offsets = [
            vector![hl * 0.75, r, hw],
            vector![hl * 0.75, r, -hw],
            vector![-hl * 0.75, r, hw],
            vector![-hl * 0.75, r, -hw],
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
                .translation(wheel_translation);
            let wheel_collider = ColliderBuilder::ball(r)
                .friction(1.0)
                .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP))
                .density(config.wheel_density);
            let wheel_handle = bodies.insert(wheel_body);

            colliders.insert_with_parent(wheel_collider, wheel_handle, bodies);
            wheels.push(wheel_handle);

            assert!(wheel_translation.y == r + placement_offset);
            
            // Create the "axle"
            let axle_mass_props = MassProperties::from_ball(config.axle_density, r);
            let axle_body = RigidBodyBuilder::dynamic()
                .translation(wheel_translation)
                .additional_mass_properties(axle_mass_props);
            let axle_handle = bodies.insert(axle_body);

            axles.push(axle_handle);

            // Joint between the chassis and the axle
            let mut locked_axes = JointAxesMask::LIN_X
                | JointAxesMask::LIN_Z
                | JointAxesMask::ANG_X
                | JointAxesMask::ANG_Z;

            if !is_front {
                locked_axes = locked_axes | JointAxesMask::ANG_Y
            }

            let mut axle_joint = GenericJointBuilder::new(locked_axes)
                .limits(JointAxis::LinY, [0.0, config.suspension_height])
                .motor_position(JointAxis::LinY, 0.0, 1.0e4, 1.0e3)
                .local_anchor1(point![offset.x, -(hh + r), offset.z]);

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

    fn apply_inputs(&mut self, input: Vec<f64>, bodies: &mut RigidBodySet, impulse_joints: &mut ImpulseJointSet) {
        let rpm = input[0];
        let steering_angle = input[1];
        let chassis = &mut bodies[self.chassis_handle];

        chassis.wake_up(true);

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
                    1.0e3
                );
        }
    }

    fn update_state(&mut self, config: &SimulationConfig, bodies: &mut RigidBodySet) {
        let chassis = &bodies[self.chassis_handle];
        let wheel = &bodies[self.wheels[0]];

        // Retrieve the position of the car body
        let translation = chassis.translation();
        let x = translation.x;
        let y = wheel.translation().y - config.wheel_radius;
        let z = translation.z;

        // Calculate the components of the forwards vector.
        let forwards = chassis.position() * Vector3::x_axis();
        let nx = forwards.x;
        let nz = forwards.z;

        // Calculate the forward and angular velocity of the vehicle
        let linvel = chassis.linvel();
        let v = f64::sqrt(linvel.x.powi(2) + linvel.z.powi(2));
        let w = chassis.angvel().y;

        self.state = vec![x, y, z, nx, nz, v, w];
    }
    
    fn unlock(&mut self, bodies: &mut RigidBodySet) {
        let chassis = &mut bodies[self.chassis_handle];
        let locked_axes = LockedAxes::ROTATION_LOCKED_X | LockedAxes::ROTATION_LOCKED_Z;

        chassis.set_locked_axes(locked_axes, false);
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