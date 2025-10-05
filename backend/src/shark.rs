use geo::Point;
use serde::Serialize;

#[derive(Debug, Serialize, Clone, Copy)]
pub struct Shark {
    pub position: Point<f64>,
    pub rotation_rad: f64,
    pub speed: f64,
}
