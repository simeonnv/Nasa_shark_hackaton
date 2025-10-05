use geo::algorithm::closest_point::ClosestPoint; // trait
use geo::algorithm::contains::Contains; // trait
use geo::algorithm::euclidean_distance::EuclideanDistance; // trait (you already used this)
use geo::{BoundingRect, Centroid, Closest};
use geo::{Point, Polygon};
use rand::Rng;
use serde::Serialize;
use std::f64::EPSILON;
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
            let random_speed: f64 = rng.random_range(0.5..1.5);
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
    /// Step the simulation forward.
    ///
    /// New params added:
    /// - map_bounds: (min_x, min_y, max_x, max_y)
    /// - land_avoid_radius: distance within which sharks start avoiding land
    /// - land_avoid_strength: multiplier for land repulsion
    /// - border_margin: margin from edges to start repulsion
    /// - border_strength: multiplier for edge repulsion
    /// - corner_avoid_radius: radius around each corner to avoid
    /// - corner_strength: multiplier for corner repulsion
    // In your Simulation impl
    pub fn step(
        &mut self,
        dt: f64,
        perception_radius: f64,
        separation_distance: f64,
        cohesion_strength: f64,
        separation_strength: f64,
        alignment_strength: f64,
        land_shape_file: &[Polygon<f64>],
        map_bounds: (f64, f64, f64, f64),
        land_avoid_radius: f64,
        land_avoid_strength: f64,
        border_margin: f64,
        border_strength: f64,
        // Note: corner avoidance is complex and often better handled by a general border avoidance.
        // I've simplified it into the main border logic for robustness.
        // corner_avoid_radius: f64,
        // corner_strength: f64,
    ) {
        let (min_x, min_y, max_x, max_y) = map_bounds;
        let max_turn_rate = PI * dt; // Allow turning up to 180 degrees per second (adjust as needed)

        let old_sharks: Vec<Shark> = self.sharks.clone(); // Clone for immutable access in loops
        let mut new_sharks = Vec::with_capacity(self.sharks.len());

        for i in 0..old_sharks.len() {
            let shark = &old_sharks[i];

            // --- 1. GATHER NEIGHBORS ---
            let mut nearby_sharks = Vec::new();
            for (j, other) in old_sharks.iter().enumerate() {
                if i != j && shark.position.euclidean_distance(&other.position) < perception_radius
                {
                    nearby_sharks.push(other);
                }
            }

            // --- 2. CALCULATE STEERING FORCES ---
            // These vectors represent a "desired change" in velocity.

            // Boids forces (unchanged logic, just separated)
            let cohesion = calculate_cohesion(shark, &nearby_sharks);
            let separation = calculate_separation(shark, &nearby_sharks, separation_distance);
            let alignment = calculate_alignment(shark, &nearby_sharks);

            // --- NEW: Proactive Avoidance Forces ---
            // Predict a future position to check for collisions
            let look_ahead_dist = shark.speed * 20.0 * dt; // Look ahead based on speed
            let future_pos = Point::new(
                shark.position.x() + look_ahead_dist * shark.rotation_rad.cos(),
                shark.position.y() + look_ahead_dist * shark.rotation_rad.sin(),
            );

            // Avoidance forces should be much stronger to override boids behavior
            let land_avoidance =
                calculate_land_avoidance(shark, &future_pos, land_shape_file, land_avoid_radius);
            let border_avoidance =
                calculate_border_avoidance(shark, &future_pos, map_bounds, border_margin);

            // --- 3. COMBINE FORCES WITH WEIGHTS ---
            let mut total_force = Point::new(0.0, 0.0);

            // If a strong avoidance is needed, it should dominate.
            // The strength of these avoidance vectors will be high if a threat is detected.
            if land_avoidance.x().powi(2) + land_avoidance.y().powi(2) > EPSILON
                || border_avoidance.x().powi(2) + border_avoidance.y().powi(2) > EPSILON
            {
                // Prioritize avoidance above all else
                total_force = Point::new(
                    total_force.x() + land_avoidance.x() * land_avoid_strength,
                    total_force.y() + land_avoidance.y() * land_avoid_strength,
                );
                total_force = Point::new(
                    total_force.x() + border_avoidance.x() * border_strength,
                    total_force.y() + border_avoidance.y() * border_strength,
                );
            } else {
                // If no immediate danger, apply standard boids rules
                total_force = Point::new(
                    total_force.x() + cohesion.x() * cohesion_strength,
                    total_force.y() + cohesion.y() * cohesion_strength,
                );
                total_force = Point::new(
                    total_force.x() + separation.x() * separation_strength,
                    total_force.y() + separation.y() * separation_strength,
                );
                total_force = Point::new(
                    total_force.x() + alignment.x() * alignment_strength,
                    total_force.y() + alignment.y() * alignment_strength,
                );
            }

            // --- 4. UPDATE VELOCITY AND ORIENTATION ---
            let mut velocity = Point::new(
                shark.speed * shark.rotation_rad.cos(),
                shark.speed * shark.rotation_rad.sin(),
            );

            // Apply the steering force to the current velocity
            velocity = Point::new(
                velocity.x() + total_force.x() * dt,
                velocity.y() + total_force.y() * dt,
            );

            // Clamp speed
            let new_speed = (velocity.x().powi(2) + velocity.y().powi(2)).sqrt();
            let new_speed_clamped = new_speed.min(2.0).max(0.5); // Your speed limits

            // Normalize velocity and apply clamped speed
            if new_speed > EPSILON {
                velocity = Point::new(
                    (velocity.x() / new_speed) * new_speed_clamped,
                    (velocity.y() / new_speed) * new_speed_clamped,
                );
            }

            // --- NEW: Smoothly turn towards the new velocity direction ---
            let desired_angle = velocity.y().atan2(velocity.x());
            let mut angle_diff = desired_angle - shark.rotation_rad;

            // Ensure we take the shortest path around the circle
            while angle_diff <= -PI {
                angle_diff += 2.0 * PI;
            }
            while angle_diff > PI {
                angle_diff -= 2.0 * PI;
            }

            // Clamp the turn rate
            let turn = angle_diff.max(-max_turn_rate).min(max_turn_rate);
            let new_angle = shark.rotation_rad + turn;

            // --- 5. UPDATE POSITION ---
            let mut new_position = Point::new(
                shark.position.x() + velocity.x() * dt,
                shark.position.y() + velocity.y() * dt,
            );

            // --- REMOVED ---
            // The problematic post-update correction and hard clamping are gone!
            // The proactive steering should prevent sharks from getting into bad spots.
            // As a last resort safety net, you could still clamp, but it shouldn't be the primary mechanic.
            new_position = Point::new(
                new_position.x().max(min_x + EPSILON).min(max_x - EPSILON),
                new_position.y().max(min_y + EPSILON).min(max_y - EPSILON),
            );

            new_sharks.push(Shark {
                position: new_position,
                rotation_rad: new_angle,
                speed: new_speed_clamped,
            });
        }

        self.sharks = new_sharks;
    }
}

// --- HELPER FUNCTIONS (add these outside the `step` method, within `impl Simulation`) ---

// These functions isolate the logic for each behavior, making the main `step` loop cleaner.
// The logic inside is mostly from your original code.

fn calculate_cohesion(shark: &Shark, nearby: &[&Shark]) -> Point<f64> {
    if nearby.is_empty() {
        return Point::new(0.0, 0.0);
    }
    let mut avg_pos = Point::new(0.0, 0.0);
    for neighbor in nearby {
        avg_pos = Point::new(
            avg_pos.x() + neighbor.position.x(),
            avg_pos.y() + neighbor.position.y(),
        );
    }
    avg_pos = Point::new(
        avg_pos.x() / nearby.len() as f64,
        avg_pos.y() / nearby.len() as f64,
    );
    // Return a steering vector towards the average position
    Point::new(
        avg_pos.x() - shark.position.x(),
        avg_pos.y() - shark.position.y(),
    )
}

fn calculate_separation(shark: &Shark, nearby: &[&Shark], separation_distance: f64) -> Point<f64> {
    let mut steer = Point::new(0.0, 0.0);
    for neighbor in nearby {
        let dist = shark.position.euclidean_distance(&neighbor.position);
        if dist > 0.0 && dist < separation_distance {
            let diff = Point::new(
                shark.position.x() - neighbor.position.x(),
                shark.position.y() - neighbor.position.y(),
            );
            // Weight by distance (stronger repulsion for closer sharks)
            steer = Point::new(steer.x() + diff.x() / dist, steer.y() + diff.y() / dist);
        }
    }
    steer
}

fn calculate_alignment(shark: &Shark, nearby: &[&Shark]) -> Point<f64> {
    if nearby.is_empty() {
        return Point::new(0.0, 0.0);
    }
    let mut avg_vel = Point::new(0.0, 0.0);
    for neighbor in nearby {
        avg_vel = Point::new(
            avg_vel.x() + neighbor.rotation_rad.cos(),
            avg_vel.y() + neighbor.rotation_rad.sin(),
        );
    }
    avg_vel = Point::new(
        avg_vel.x() / nearby.len() as f64,
        avg_vel.y() / nearby.len() as f64,
    );

    // Return a steering vector towards the average heading
    let shark_vel = Point::new(shark.rotation_rad.cos(), shark.rotation_rad.sin());
    Point::new(avg_vel.x() - shark_vel.x(), avg_vel.y() - shark_vel.y())
}

/// NEW: Calculates a strong steering vector away from nearby land if the shark's future position is threatened.
fn calculate_land_avoidance(
    shark: &Shark,
    future_pos: &Point<f64>,
    land_shape_file: &[Polygon<f64>],
    land_avoid_radius: f64,
) -> Point<f64> {
    let mut avoidance_force = Point::new(0.0, 0.0);

    for poly in land_shape_file {
        if poly.contains(future_pos) {
            // Future position is INSIDE land. This is high alert.
            // Find the closest point on the polygon to the shark's CURRENT position
            // and push it directly away from that point.
            if let Closest::SinglePoint(cp) | Closest::Intersection(cp) =
                poly.closest_point(&shark.position)
            {
                let away_vec = Point::new(shark.position.x() - cp.x(), shark.position.y() - cp.y());
                // This force should be very strong to guarantee avoidance.
                return away_vec; // Return immediately with a strong directive
            }
        }
    }
    // If not inside, you could add a softer repulsion for being near, but the
    // "look-ahead" check is the most critical part.
    avoidance_force
}

/// NEW: Calculates a steering vector to turn the shark away from the map boundaries.
fn calculate_border_avoidance(
    shark: &Shark,
    future_pos: &Point<f64>,
    map_bounds: (f64, f64, f64, f64),
    border_margin: f64,
) -> Point<f64> {
    let (min_x, min_y, max_x, max_y) = map_bounds;
    let mut desired_velocity = Point::new(0.0, 0.0);
    let mut changed = false;

    if future_pos.x() < min_x + border_margin {
        desired_velocity = Point::new(1.0, desired_velocity.y()); // Steer right
        changed = true;
    } else if future_pos.x() > max_x - border_margin {
        desired_velocity = Point::new(-1.0, desired_velocity.y()); // Steer left
        changed = true;
    }

    if future_pos.y() < min_y + border_margin {
        desired_velocity = Point::new(desired_velocity.x(), 1.0); // Steer up
        changed = true;
    } else if future_pos.y() > max_y - border_margin {
        desired_velocity = Point::new(desired_velocity.x(), -1.0); // Steer down
        changed = true;
    }

    if changed {
        // This returns a "desired direction" vector away from the border.
        let current_velocity = Point::new(shark.rotation_rad.cos(), shark.rotation_rad.sin());
        return Point::new(
            desired_velocity.x() - current_velocity.x(),
            desired_velocity.y() - current_velocity.y(),
        );
    }

    Point::new(0.0, 0.0) // No avoidance needed
}
