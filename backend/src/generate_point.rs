use geo::{ClosestPoint, Contains, Point, Polygon, Scale};
use rand::Rng;

pub fn random_point<R: Rng>(rng: &mut R) -> Point<f64> {
    let lat = rng.random_range(-90.0..=90.0);
    let lon = rng.random_range(-180.0..=180.0);
    Point::new(lon, lat)
}

pub fn random_point_in_water<R: Rng>(rng: &mut R, land_polygons: &[Polygon<f64>]) -> Point<f64> {
    loop {
        let random_point = random_point(rng);
        let is_in_water = !land_polygons.iter().any(|poly| {
            let poly = poly.scale_xy(1.1, 1.1);
            poly.contains(&random_point)
        });

        if is_in_water {
            return random_point;
        }
    }
}
