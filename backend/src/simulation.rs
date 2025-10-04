use geo::{EuclideanDistance, Point, Polygon};
use rand::Rng;
use serde::Serialize;
use std::f64::consts::PI;

use crate::{Shark, random_point_in_water};

#[derive(Debug, Serialize)]
pub struct Simulation {
    pub sharks: Vec<Shark>,
}

impl Simulation {
    pub fn new<R: Rng>(
        amount_of_sharks: usize,
        rng: &mut R,
        land_shape_file: &[Polygon<f64>],
    ) -> Self {
        let mut sharks = Vec::<Shark>::with_capacity(amount_of_sharks);
        for _ in 0..amount_of_sharks {
            let rand_point = random_point_in_water(rng, land_shape_file);
            let random_orientation: f64 = rng.random_range(0.0..(2.0 * PI));
            let random_speed: f64 = rng.random_range(0.5..1.5); // Example: speed between 0.5 and 1.5 units
            let shark = Shark {
                position: rand_point,
                rotation_rad: random_orientation,
                speed: random_speed,
            };
            sharks.push(shark);
        }
        Self { sharks }
    }
}

impl Simulation {
    pub fn step(
        &mut self,
        dt: f64,                  // Time step
        perception_radius: f64,   // Distance to consider nearby sharks
        separation_distance: f64, // Minimum distance to avoid other sharks
        cohesion_strength: f64,   // Weight for cohesion rule
        separation_strength: f64, // Weight for separation rule
        alignment_strength: f64,  // Weight for alignment rule
                                  // max_turn_angle: f64,      // Maximum turn in radians per step
                                  // land_shape_file: &[Polygon<f64>],
                                  // rng: &mut R,
    ) {
        let mut new_sharks = Vec::with_capacity(self.sharks.len());

        for i in 0..self.sharks.len() {
            let shark = &self.sharks[i];
            let mut velocity = Point::new(
                shark.speed * shark.rotation_rad.cos(),
                shark.speed * shark.rotation_rad.sin(),
            );

            // Find nearby sharks within perception radius
            let mut nearby_sharks = Vec::new();
            for (j, other) in self.sharks.iter().enumerate() {
                if i != j && shark.position.euclidean_distance(&other.position) < perception_radius
                {
                    nearby_sharks.push(other);
                }
            }

            // Cohesion: Move toward average position of nearby sharks
            let mut cohesion = Point::new(0.0, 0.0);
            if !nearby_sharks.is_empty() {
                let mut avg_position = Point::new(0.0, 0.0);
                for neighbor in &nearby_sharks {
                    avg_position = Point::new(
                        avg_position.x() + neighbor.position.x(),
                        avg_position.y() + neighbor.position.y(),
                    );
                }
                avg_position = Point::new(
                    avg_position.x() / nearby_sharks.len() as f64,
                    avg_position.y() / nearby_sharks.len() as f64,
                );
                cohesion = Point::new(
                    (avg_position.x() - shark.position.x()) * cohesion_strength,
                    (avg_position.y() - shark.position.y()) * cohesion_strength,
                );
            }

            // Separation: Steer away from sharks that are too close
            let mut separation = Point::new(0.0, 0.0);
            for neighbor in &nearby_sharks {
                let distance = shark.position.euclidean_distance(&neighbor.position);
                if distance < separation_distance && distance > 0.0 {
                    let away = Point::new(
                        shark.position.x() - neighbor.position.x(),
                        shark.position.y() - neighbor.position.y(),
                    );
                    let scale = separation_strength / distance.max(0.1); // Avoid division by zero
                    separation = Point::new(
                        separation.x() + away.x() * scale,
                        separation.y() + away.y() * scale,
                    );
                }
            }

            // Alignment: Match average orientation of nearby sharks
            let mut alignment_vec = Point::new(0.0, 0.0);
            if !nearby_sharks.is_empty() {
                for neighbor in &nearby_sharks {
                    let neigh_vel = Point::new(
                        neighbor.speed * neighbor.rotation_rad.cos(),
                        neighbor.speed * neighbor.rotation_rad.sin(),
                    );
                    alignment_vec = Point::new(
                        alignment_vec.x() + neigh_vel.x(),
                        alignment_vec.y() + neigh_vel.y(),
                    );
                }
                alignment_vec = Point::new(
                    alignment_vec.x() / nearby_sharks.len() as f64,
                    alignment_vec.y() / nearby_sharks.len() as f64,
                );
                alignment_vec = Point::new(
                    (alignment_vec.x() - velocity.x()) * alignment_strength,
                    (alignment_vec.y() - velocity.y()) * alignment_strength,
                );
            }

            // Combine forces to update velocity
            velocity = Point::new(
                velocity.x() + cohesion.x() + separation.x() + alignment_vec.x(),
                velocity.y() + cohesion.y() + separation.y() + alignment_vec.y(),
            );

            // Update orientation (rotation_rad)
            let desired_angle = velocity.y().atan2(velocity.x());
            let angle_diff = (desired_angle - shark.rotation_rad)
                .sin()
                .atan2((desired_angle - shark.rotation_rad).cos());
            let new_angle = shark.rotation_rad + angle_diff;
            // Optionally, limit the turn if max_turn_angle is used:
            // let new_angle = shark.rotation_rad + angle_diff.min(max_turn_angle).max(-max_turn_angle);

            // Update position
            let new_speed = (velocity.x().powi(2) + velocity.y().powi(2)).sqrt();
            let new_position = Point::new(
                shark.position.x() + velocity.x() * dt,
                shark.position.y() + velocity.y() * dt,
            );

            // // Ensure the new position is in water
            // while !is_point_in_water(&new_position, land_shape_file) {
            //     // If on land, try a random nearby point
            //     new_position = random_point_in_water(rng, land_shape_file);
            // }

            // Update shark
            let new_shark = Shark {
                position: new_position,
                rotation_rad: new_angle,
                speed: new_speed.min(2.0).max(0.5), // Clamp speed to reasonable range
            };
            new_sharks.push(new_shark);
        }

        // dbg!(&self);

        self.sharks = new_sharks;
    }
}
