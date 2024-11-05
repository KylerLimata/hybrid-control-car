use nalgebra::{UnitVector3, Vector3};
use rapier3d_f64::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d_f64::prelude::*;
use pyo3::prelude::*;
use sim::{SimulationConfig, SimulationEnvironment};

mod sim;

#[pyclass]
struct CarSimulation {
    physics_pipeline: PhysicsPipeline,
    gravity: Vector3<f64>,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: BroadPhaseMultiSap,
    narrow_phase: NarrowPhase, 
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    car: DynamicRayCastVehicleController,
    state: Vec<f64>
}

#[pymethods]
impl CarSimulation {
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
        let initial_state = vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let car = init_car(initial_state.clone(), &mut bodies, &mut colliders);

        let sim = CarSimulation {
            physics_pipeline: PhysicsPipeline::new(),
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters: integration_parameters,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: bodies,
            colliders: colliders,
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            car: car,
            state: initial_state
        };

        return sim;
    }

    fn create_floor(&mut self) {
        let ground_size = 500.0;
        let ground_height = 0.1;

        let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
        let floor_handle = self.bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
        self.colliders.insert_with_parent(collider, floor_handle, &mut self.bodies);
    }

    fn step(&mut self, initial_state: Vec<f64>, input: Vec<f64>) -> Vec<f64> {
        let mut should_replace = false;

        for i in 0..6 {
            if (initial_state[i] - &self.state[i]).abs() > 1e-3 {
                should_replace = true;
            }
        }

        if should_replace {
            let handle = self.car.chassis;

            self.bodies.remove(
                handle, 
                &mut self.islands, 
                &mut self.colliders, 
                &mut self.impulse_joints, 
                &mut self.multibody_joints, 
                true
            );

            self.car = init_car(
                initial_state, 
                &mut self.bodies, 
                &mut self.colliders
            )
        }


        self.apply_inputs(input);
        
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

        self.update_state();

        return self.state.clone();
    }

    fn apply_inputs(&mut self, input: Vec<f64>) {
        // Unpack input
        let engine_force = input[0];
        let steering_angle = input[1];

        // Apply inputs
        let wheels = self.car.wheels_mut();

        wheels[0].engine_force = engine_force;
        wheels[0].steering = steering_angle;
        wheels[1].engine_force = engine_force;
        wheels[1].steering = steering_angle;
    }

    fn update_state(&mut self) {
        let car_handle = self.car.chassis;

        // Update the car
        self.car.update_vehicle(
            self.integration_parameters.dt,
            &mut self.bodies,
            &self.colliders,
            &self.query_pipeline,
            QueryFilter::exclude_dynamic().exclude_rigid_body(car_handle),
        );

        let car_body = &self.bodies[car_handle];
    
        // Retrieve the horizontal position of the vehicle
        let translation = car_body.translation();
        let x = translation.x;
        let z = translation.z;

        // Calculate the components of the forwards vector.
        let forwards = car_body.rotation() * Vector3::ith(self.car.index_forward_axis, 1.0);
        let forwards_horizontal = UnitVector3::new_normalize(vector![
            forwards.x,
            0.0,
            forwards.z
        ]);
        let n_x = forwards_horizontal.x;
        let n_z = forwards_horizontal.z;

        // Retrieve the forward and angular velocity of the vehicle
        let linvel = car_body.linvel();
        let v = f64::sqrt(linvel.x.powf(2.0) + linvel.z.powf(2.0));
        let w = car_body.angvel().y;

        // Create the state vector
        self.state = vec![x, z, n_x, n_z, v, w];
    }
}

fn init_car(initial_state: Vec<f64>, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> DynamicRayCastVehicleController {
    // Unpack the state vector
    let x0 = initial_state[0];
    let z0 = initial_state[1];
    let nx0 = initial_state[2];
    let nz0 = initial_state[3];
    let v0 = initial_state[4];
    let w0 = initial_state[5];
    let phi0 = libm::atan2(nz0, nx0);

    // Create the chassis body
    let hw = 0.3;
    let hh = 0.15;
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![x0, 2.0*hh + hh/4.0, z0])
        .rotation(vector![0.0, phi0, 0.0])
        .linvel(vector![v0*nx0, 0.0, v0*nz0])
        .angvel(vector![0.0, w0, 0.0]);
    let vehicle_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);

    colliders.insert_with_parent(collider, vehicle_handle, bodies);

    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };
    let mut car = DynamicRayCastVehicleController::new(vehicle_handle);
    let wheel_positions = [
        point![hw * 1.5, -hh, hw],
        point![hw * 1.5, -hh, -hw],
        point![-hw * 1.5, -hh, hw],
        point![-hw * 1.5, -hh, -hw],
    ];

    for pos in wheel_positions {
        car.add_wheel(pos, -Vector::y(), Vector::z(), hh, hh / 4.0, &tuning);
    }

    return car;
}

#[pymodule]
fn hybrid_control_car(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<CarSimulation>()?;
    module.add_class::<SimulationConfig>()?;
    module.add_class::<SimulationEnvironment>()?;
    Ok(())
}