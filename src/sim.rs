use libm::atan2;

use rapier3d_f64::prelude::*;

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
    create_car(X0, &mut bodies, &mut colliders);

    return vec![];
}

fn create_floor(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
    let ground_size = 500.0;
    let ground_height = 0.1;
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);

    colliders.insert_with_parent(collider, floor_handle, bodies);
}

fn create_car(X0: Vec<f64>, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
    // Unpack the state vector
    let x0 = X0[0];
    let z0 = X0[1];
    let nx0 = X0[2];
    let nz0 = X0[3];
    let v0 = X0[4];
    let phi = atan2(nz0, nx0);

    let hw = 0.3;
    let hh = 0.15;
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![x0, 2.0*hh + hh/4.0, z0])
        .rotation(vector![0.0, phi, 0.0])
        .linvel(vector![0.0, 0.0, 0.0]);
    let vehicle_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);
}