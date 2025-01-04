use std::env;
use std::f64::consts::{PI, E};

fn gravitational_acceleration(altitude: f64) -> f64 {
    let earth_radius = 6_371_000.0; 
    let g_surface = 9.81; 

    g_surface * (earth_radius / (earth_radius + altitude)).powi(2)
}

fn air_density(altitude: f64, temperature: f64) -> f64 {
    let sea_level_density = 1.225; 
    let scale_height = 8_500.0; 
    let temp_factor = temperature / 288.15; 

    sea_level_density * (-altitude / scale_height).exp() * temp_factor
}

/// Simulates the motion of a pendulum with nonlinear effects, damping, and driving forces.
/// # Parameters
/// - `initial_angle`: Initial angular displacement in radians.
/// - `length`: Length of the pendulum (m).
/// - `mass`: Mass of the pendulum bob (kg).
/// - `altitude`: Altitude of the pendulum (m).
/// - `air_resistance`: Initial air resistance coefficient (kg·m²/s).
/// - `nonlinear_spring`: Nonlinear spring constant (N·m/rad³).
/// - `driving_force`: Driving force amplitude (N).
/// - `driving_frequency`: Driving force angular frequency (rad/s).
/// - `temperature`: Ambient temperature (K).
/// - `time_step`: Time step for the simulation (s).
/// - `duration`: Total duration of the simulation (s).
/// # Returns
/// - A vector of (time, angle) tuples representing the pendulum's motion over time.
fn simulate_pendulum(
    initial_angle: f64,
    length: f64,
    mass: f64,
    altitude: f64,
    air_resistance: f64,
    nonlinear_spring: f64,
    driving_force: f64,
    driving_frequency: f64,
    temperature: f64,
    time_step: f64,
    duration: f64,
) -> Vec<(f64, f64)> {
    let mut time = 0.0;
    let mut angle = initial_angle;
    let mut angular_velocity:f64  = 0.0;

    let mut results = Vec::new();

    while time <= duration {
        results.push((time, angle));

        let g = gravitational_acceleration(altitude);
        let rho = air_density(altitude, temperature);
        let damping = air_resistance * rho * angular_velocity.abs();

        let spring_force = -nonlinear_spring * angle.powi(3);

        let driving = driving_force * (driving_frequency * time).cos();

        let angular_acceleration =
            -g / length * angle.sin() - (damping / mass) * angular_velocity + spring_force / mass + driving / (mass * length);

        angular_velocity += angular_acceleration * time_step;
        angle += angular_velocity * time_step;

        time += time_step;
    }

    results
}

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() != 12 {
        println!("Usage: {} <initial_angle> <length> <mass> <altitude> <air_resistance> <nonlinear_spring> <driving_force> <driving_frequency> <temperature> <time_step> <duration>", args[0]);
        return;
    }

    let initial_angle: f64 = args[1].parse().expect("Invalid initial angle");
    let length: f64 = args[2].parse().expect("Invalid length");
    let mass: f64 = args[3].parse().expect("Invalid mass");
    let altitude: f64 = args[4].parse().expect("Invalid altitude");
    let air_resistance: f64 = args[5].parse().expect("Invalid air resistance");
    let nonlinear_spring: f64 = args[6].parse().expect("Invalid nonlinear spring constant");
    let driving_force: f64 = args[7].parse().expect("Invalid driving force");
    let driving_frequency: f64 = args[8].parse().expect("Invalid driving frequency");
    let temperature: f64 = args[9].parse().expect("Invalid temperature");
    let time_step: f64 = args[10].parse().expect("Invalid time step");
    let duration: f64 = args[11].parse().expect("Invalid duration");

    let motion = simulate_pendulum(
        initial_angle,
        length,
        mass,
        altitude,
        air_resistance,
        nonlinear_spring,
        driving_force,
        driving_frequency,
        temperature,
        time_step,
        duration,
    );

    println!("Time (s) | Angle (radians)");
    for (time, angle) in motion {
        println!("{:.2} | {:.4}", time, angle);
    }
}
