use crate::{Shark, random_point_in_water};
use geo::algorithm::closest_point::ClosestPoint; // trait
use geo::algorithm::contains::Contains; // trait
use geo::algorithm::euclidean_distance::EuclideanDistance; // trait
use geo::{BoundingRect, Centroid, Closest, Intersects, Rect};
use geo::{Point, Polygon};
use rand::Rng;
use serde::Serialize;
use std::f64::EPSILON;
use std::f64::consts::PI;

#[derive(Debug, Serialize)]
pub struct Simulation {
    pub sharks: Vec<Shark>,
    land_bounds: Vec<Rect<f64>>,
    // 1. ADDED: Vector of points the sharks are interested in
    pub goals: Vec<Point<f64>>,
}

impl Simulation {
    pub fn new<R: Rng>(
        amount_of_sharks: usize,
        rng: &mut R,
        land_shape_file: &[Polygon<f64>],
        // 2. ADDED: Goals parameter
        goals: Vec<Point<f64>>,
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

        let land_bounds = land_shape_file
            .iter()
            .filter_map(|poly| poly.bounding_rect())
            .collect();

        Self {
            sharks,
            land_bounds,
            // 3. Initialized the new field
            goals,
        }
    }
}

impl Simulation {
    #[allow(clippy::too_many_arguments)] // Allowing many arguments for the simulation parameters
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
        // 4. ADDED: Goal-seeking parameters
        goal_seeking_radius: f64,
        goal_seeking_strength: f64,
    ) {
        let (min_x, min_y, max_x, max_y) = map_bounds;
        let max_turn_rate = PI * dt;

        let old_sharks: Vec<Shark> = self.sharks.clone();
        let mut new_sharks = Vec::with_capacity(self.sharks.len());

        for i in 0..old_sharks.len() {
            let shark = &old_sharks[i];

            let mut nearby_sharks = Vec::new();
            for (j, other) in old_sharks.iter().enumerate() {
                if i != j && shark.position.euclidean_distance(&other.position) < perception_radius
                {
                    nearby_sharks.push(other);
                }
            }

            let cohesion = calculate_cohesion(shark, &nearby_sharks);
            let separation = calculate_separation(shark, &nearby_sharks, separation_distance);
            let alignment = calculate_alignment(shark, &nearby_sharks);
            // 5. ADDED: Goal-seeking force calculation
            let goal_seeking = calculate_goal_seeking(shark, &self.goals, goal_seeking_radius);

            let look_ahead_dist = shark.speed * 20.0 * dt; // Look ahead based on speed
            let future_pos = Point::new(
                shark.position.x() + look_ahead_dist * shark.rotation_rad.cos(),
                shark.position.y() + look_ahead_dist * shark.rotation_rad.sin(),
            );

            let land_avoidance = calculate_land_avoidance(
                shark,
                &future_pos,
                land_shape_file,
                &self.land_bounds,
                land_avoid_radius,
            );
            let border_avoidance =
                calculate_border_avoidance(shark, &future_pos, map_bounds, border_margin);

            let mut total_force = Point::new(0.0, 0.0);

            if land_avoidance.x().powi(2) + land_avoidance.y().powi(2) > EPSILON
                || border_avoidance.x().powi(2) + border_avoidance.y().powi(2) > EPSILON
            {
                total_force = Point::new(
                    total_force.x() + land_avoidance.x() * land_avoid_strength,
                    total_force.y() + land_avoidance.y() * land_avoid_strength,
                );
                total_force = Point::new(
                    total_force.x() + border_avoidance.x() * border_strength,
                    total_force.y() + border_avoidance.y() * border_strength,
                );
            } else {
                // Flocking forces
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
                // 6. ADDED: Goal-seeking force integration
                total_force = Point::new(
                    total_force.x() + goal_seeking.x() * goal_seeking_strength,
                    total_force.y() + goal_seeking.y() * goal_seeking_strength,
                );
            }

            let mut velocity = Point::new(
                shark.speed * shark.rotation_rad.cos(),
                shark.speed * shark.rotation_rad.sin(),
            );

            velocity = Point::new(
                velocity.x() + total_force.x() * dt,
                velocity.y() + total_force.y() * dt,
            );

            let new_speed = (velocity.x().powi(2) + velocity.y().powi(2)).sqrt();
            let new_speed_clamped = new_speed.min(2.0).max(0.5); // Your speed limits

            if new_speed > EPSILON {
                velocity = Point::new(
                    (velocity.x() / new_speed) * new_speed_clamped,
                    (velocity.y() / new_speed) * new_speed_clamped,
                );
            }

            let desired_angle = velocity.y().atan2(velocity.x());
            let mut angle_diff = desired_angle - shark.rotation_rad;

            while angle_diff <= -PI {
                angle_diff += 2.0 * PI;
            }
            while angle_diff > PI {
                angle_diff -= 2.0 * PI;
            }

            let turn = angle_diff.max(-max_turn_rate).min(max_turn_rate);
            let new_angle = shark.rotation_rad + turn;

            let mut new_position = Point::new(
                shark.position.x() + velocity.x() * dt,
                shark.position.y() + velocity.y() * dt,
            );

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

// 7. NEW HELPER FUNCTION FOR GOAL SEEKING

/// Calculates a steering force towards the closest goal point within the radius.
fn calculate_goal_seeking(
    shark: &Shark,
    goals: &[Point<f64>],
    goal_seeking_radius: f64,
) -> Point<f64> {
    if goals.is_empty() {
        return Point::new(0.0, 0.0);
    }

    let mut closest_goal: Option<(Point<f64>, f64)> = None;

    // Find the closest goal within the seeking radius
    for goal in goals {
        let dist = shark.position.euclidean_distance(goal);
        if dist < goal_seeking_radius {
            match closest_goal {
                Some((_, current_dist)) if dist < current_dist => {
                    closest_goal = Some((*goal, dist));
                }
                None => {
                    closest_goal = Some((*goal, dist));
                }
                _ => {}
            }
        }
    }

    match closest_goal {
        Some((goal_point, _)) => {
            // Steer towards the goal
            let steer_vector = Point::new(
                goal_point.x() - shark.position.x(),
                goal_point.y() - shark.position.y(),
            );
            // Normalize the vector to get a unit direction force
            let norm = steer_vector.euclidean_distance(&Point::new(0.0, 0.0));
            if norm > EPSILON {
                Point::new(steer_vector.x() / norm, steer_vector.y() / norm)
            } else {
                Point::new(0.0, 0.0)
            }
        }
        None => Point::new(0.0, 0.0), // No goals in range, no force
    }
}

// --- EXISTING HELPER FUNCTIONS (KEEP THEM AS THEY ARE) ---

fn calculate_cohesion(shark: &Shark, nearby: &[&Shark]) -> Point<f64> {
    // ... (unchanged)
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
    Point::new(
        avg_pos.x() - shark.position.x(),
        avg_pos.y() - shark.position.y(),
    )
}

fn calculate_separation(shark: &Shark, nearby: &[&Shark], separation_distance: f64) -> Point<f64> {
    // ... (unchanged)
    let mut steer = Point::new(0.0, 0.0);
    for neighbor in nearby {
        let dist = shark.position.euclidean_distance(&neighbor.position);
        if dist > 0.0 && dist < separation_distance {
            let diff = Point::new(
                shark.position.x() - neighbor.position.x(),
                shark.position.y() - neighbor.position.y(),
            );
            steer = Point::new(steer.x() + diff.x() / dist, steer.y() + diff.y() / dist);
        }
    }
    steer
}

fn calculate_alignment(shark: &Shark, nearby: &[&Shark]) -> Point<f64> {
    // ... (unchanged)
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

    let shark_vel = Point::new(shark.rotation_rad.cos(), shark.rotation_rad.sin());
    Point::new(avg_vel.x() - shark_vel.x(), avg_vel.y() - shark_vel.y())
}

fn calculate_land_avoidance(
    _shark: &Shark,
    future_pos: &Point<f64>,
    land_shape_file: &[Polygon<f64>],
    land_bounds: &[Rect<f64>],
    land_avoid_radius: f64,
) -> Point<f64> {
    // ... (unchanged)
    let mut total_avoidance_force = Point::new(0.0, 0.0);

    for (i, poly) in land_shape_file.iter().enumerate() {
        if i >= land_bounds.len() {
            continue;
        }
        let poly_bound = &land_bounds[i];

        let shark_check_rect = Rect::new(
            (
                future_pos.x() - land_avoid_radius,
                future_pos.y() - land_avoid_radius,
            ),
            (
                future_pos.x() + land_avoid_radius,
                future_pos.y() + land_avoid_radius,
            ),
        );

        if !poly_bound.intersects(&shark_check_rect) {
            continue;
        }

        let closest = poly.closest_point(future_pos);
        let cp = match closest {
            Closest::Indeterminate => continue,
            Closest::Intersection(p) => p,
            Closest::SinglePoint(p) => p,
        };

        let dist = future_pos.euclidean_distance(&cp);
        if dist >= land_avoid_radius {
            continue;
        }

        let dir_vec = Point::new(cp.x() - future_pos.x(), cp.y() - future_pos.y());

        let is_inside = poly.contains(future_pos);

        let dir = if is_inside {
            dir_vec
        } else {
            Point::new(-dir_vec.x(), -dir_vec.y())
        };

        let norm = dir.euclidean_distance(&Point::new(0.0, 0.0));
        if norm > EPSILON {
            let unit = Point::new(dir.x() / norm, dir.y() / norm);
            let strength = if dist < EPSILON {
                1.0
            } else {
                (land_avoid_radius - dist) / land_avoid_radius
            };
            let force = Point::new(unit.x() * strength, unit.y() * strength);
            total_avoidance_force = Point::new(
                total_avoidance_force.x() + force.x(),
                total_avoidance_force.y() + force.y(),
            );
        }
    }

    total_avoidance_force
}

fn calculate_border_avoidance(
    shark: &Shark,
    future_pos: &Point<f64>,
    map_bounds: (f64, f64, f64, f64),
    border_margin: f64,
) -> Point<f64> {
    // ... (unchanged)
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
        let current_velocity = Point::new(shark.rotation_rad.cos(), shark.rotation_rad.sin());
        return Point::new(
            desired_velocity.x() - current_velocity.x(),
            desired_velocity.y() - current_velocity.y(),
        );
    }

    Point::new(0.0, 0.0)
}
