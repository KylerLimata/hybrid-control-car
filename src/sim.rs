use libm::atan2;
use nalgebra::{Isometry3, Vector3};
use pyo3::{pyclass, pymethods};
use rapier3d_f64::prelude::*;

const CAR_GROUP: Group = Group::GROUP_1;

fn create_floor(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
    let ground_size = 5000.0;
    let ground_height = 10.0;
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size)
    .friction(1.0);

    colliders.insert_with_parent(collider, floor_handle, bodies);
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
            chassis_mass: 1.0,
            chassis_width: 0.1,
            chassis_height: 0.1,
            wheel_mass: 0.01,
            axle_mass: 0.01,
            suspension_height: 0.2,
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
        if initial_state.len() != 7 {
            panic!("State vec should be of size 7.")
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
            let placement_offset = 0.0;

            if let Some(car) = &mut self.car.replace(Car::new(
                initial_state,
                placement_offset,
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

            /*
            if let Some(car) = &mut self.car {
                let chassis = self.bodies.get_mut(car.chassis_handle).unwrap();
                let locked_axes = LockedAxes::TRANSLATION_LOCKED_X
                    | LockedAxes::TRANSLATION_LOCKED_Z
                    | LockedAxes::ROTATION_LOCKED;

                chassis.set_locked_axes(locked_axes, true);

                /*
                while chassis.translation().y - (hh + hh/4.0) > 0.0 {
                    self.step_pipeline();
                }
                */
            }

            // The car is placed above the floor to avoid clipping through it,
            // so we need to use Rapier to place the car on the floor.
            for _ in 0..100 {
                self.step_pipeline();
            }

            if let Some(car) = &mut self.car {
                let chassis = self.bodies.get_mut(car.chassis_handle).unwrap();

                chassis.set_locked_axes(LockedAxes::empty(), true);
            }
            */
        }

        if let Some(car) = &mut self.car {
            car.apply_inputs(input.clone(), &mut self.bodies, &mut self.impulse_joints);
        }

        self.step_pipeline();

        if let Some(car) = &mut self.car {
            car.update_state(&self.config, &mut self.bodies);

            return car.state.clone();
        }

        panic!("Somehow, the car vanished")
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
    fn new(initial_state: Vec<f64>, placement_offset: f64, config: &SimulationConfig, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, impulse_joints: &mut ImpulseJointSet) -> Self {
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
        let wheel_radius = hh/4.0;

        // Calculate the initial position of the vehicle (translation + rotation)
        let car_translation = vector![x0, placement_offset, z0];
        let car_axisangle = vector![0.0, phi0, 0.0];
        let car_isometry = Isometry3::new(
            car_translation,
            car_axisangle
        );

        // Create the chassis rigid body
        let chassis_offset = vector![0.0, hh + wheel_radius, 0.0];
        let chassis_translation = chassis_offset + car_translation;
        let chassis_isometry = Isometry3::new(
            chassis_translation,
            car_axisangle
        );
        let chassis_body_builder = RigidBodyBuilder::dynamic()
            .position(chassis_isometry)
            .linvel(vector![v0*nx0, 0.0, v0*nz0])
            .angvel(vector![0.0, w0, 0.0])
            .can_sleep(false);
        let chassis_handle = bodies.insert(chassis_body_builder);
        let chassis_collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw)
            .mass(config.chassis_mass)
            .collision_groups(InteractionGroups::new(CAR_GROUP, !CAR_GROUP));

        colliders.insert_with_parent(chassis_collider, chassis_handle, bodies);
        assert!(chassis_translation.y == hh + hh/4.0 + placement_offset);

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

            assert!(wheel_translation.y == wheel_radius + placement_offset);
            
            // Create the "axle"
            let axle_body = RigidBodyBuilder::dynamic()
                .translation(wheel_translation)
                .additional_mass(config.axle_mass);
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
                .local_anchor1(point![offset.x, -(hh + wheel_radius), offset.z]);

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
        let hh = config.chassis_height/2.0;

        // Retrieve the position of the car body
        let translation = chassis.translation();
        let x = translation.x;
        let y = translation.y - (hh + hh/4.0);
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