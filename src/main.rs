use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

fn main() {
    println!("Hello, world!");
}

fn init_car(bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
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
}