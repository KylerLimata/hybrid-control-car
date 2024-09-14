use nalgebra::Vector3;
use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;
use pyo3::prelude::*;

#[pyclass]
struct CarSimulation {
    physics_pipeline: PhysicsPipeline,
    gravity: Vector3<f32>,
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
    car: Option<DynamicRayCastVehicleController>
}

#[pymethods]
impl CarSimulation {
    #[new]
    fn new() -> Self {
        let mut sim = CarSimulation {
            physics_pipeline: PhysicsPipeline::new(),
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            car: None
        };

        sim.reset_car(0.0);

        return sim;
    }

    fn reset_car(&mut self, rotation: f32) {
        let bodies = &mut self.bodies;

        // Remove the current car from the sim
        if let Some(car) = &self.car  {
            let chassis_handle = car.chassis;

            bodies.remove(
                chassis_handle, 
                &mut self.islands, 
                &mut self.colliders, 
                &mut self.impulse_joints, 
                &mut self.multibody_joints, 
                true
            );
        }

        let hw = 0.3;
        let hh = 0.15;
        let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 0.0, 0.0])
        .rotation(vector![0.0, rotation, 0.0]);
        let vehicle_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);
        self.colliders.insert_with_parent(collider, vehicle_handle, bodies);

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

        self.car = Some(car)
    }

    fn create_floor(&mut self) {
        let ground_size = 500.0;
        let ground_height = 0.1;

        let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
        let floor_handle = self.bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
        self.colliders.insert_with_parent(collider, floor_handle, &mut self.bodies);
    }

    fn step(&mut self, engine_force: f32, steering_angle: f32) -> PyResult<(Vec<f32>, bool, bool, bool)> {
        // Update the steering and throttle
        let car = self.car.as_mut().unwrap();
        let car_handle = car.chassis;
        let wheels = car.wheels_mut();

        wheels[0].engine_force = engine_force;
        wheels[0].steering = steering_angle;
        wheels[1].engine_force = engine_force;
        wheels[1].steering = steering_angle;

        car.update_vehicle(
            self.integration_parameters.dt,
            &mut self.bodies,
            &self.colliders,
            &self.query_pipeline,
            QueryFilter::exclude_dynamic().exclude_rigid_body(car_handle),
        );

        // Step the physics pipeline
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
            &(),
            &(),
        );

        // Retrieve the horizontal position and velocity of the vehicle
        let car_body = &self.bodies[car_handle];
        let translation = car_body.translation();
        let velocity = car_body.linvel();
        let state = vec![translation.x, translation.z, velocity.x, velocity.z];

        // Test whether the vehicle is colliding
        let collider_handle = car_body.colliders()[0];
        let collider = self.colliders.get(collider_handle).unwrap();
        let colliding = collider.active_events().contains(ActiveEvents::COLLISION_EVENTS);

        Ok((state, colliding, false, false))
    }
}

#[pymodule]
fn hybrid_control_car(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<CarSimulation>()?;
    Ok(())
}