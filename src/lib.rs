use nalgebra::Vector3;
use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;
use pyo3::prelude::*;

#[pyclass]
struct CarSimulation {
    physics_pipeline: PhysicsPipeline,
    gravity: Vector3<f32>,
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: BroadPhaseMultiSap,
    narrow_phase: NarrowPhase, 
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    car: RigidBodyHandle
}

#[pymethods]
impl CarSimulation {
    #[new]
    fn new() -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();

        let car = init_car(&mut bodies, &mut colliders);

        CarSimulation {
            physics_pipeline: PhysicsPipeline::new(),
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters: IntegrationParameters::default(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: bodies,
            colliders: colliders,
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            car: car
        }
    }


    fn step(&mut self) -> PyResult<()> {
        println!("Stepping!");
        Ok(())
    }
}

fn init_car(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> RigidBodyHandle {
    let hw = 0.3;
    let hh = 0.15;
    let rigid_body = RigidBodyBuilder::dynamic().translation(vector![0.0, 1.0, 0.0]);
    let vehicle_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);
    colliders.insert_with_parent(collider, vehicle_handle, bodies);

    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };
    let mut vehicle = DynamicRayCastVehicleController::new(vehicle_handle);
    let wheel_positions = [
        point![hw * 1.5, -hh, hw],
        point![hw * 1.5, -hh, -hw],
        point![-hw * 1.5, -hh, hw],
        point![-hw * 1.5, -hh, -hw],
    ];

    for pos in wheel_positions {
        vehicle.add_wheel(pos, -Vector::y(), Vector::z(), hh, hh / 4.0, &tuning);
    }

    return vehicle_handle;
}

#[pymodule]
fn hybrid_control_car(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<CarSimulation>()?;
    Ok(())
}