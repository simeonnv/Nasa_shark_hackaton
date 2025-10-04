use geo::LineString;
use geo::Polygon;
use shapefile::Reader;
use shapefile::Shape;
use std::error::Error;

pub fn load_land_polygons(shapefile_path: &str) -> Result<Vec<Polygon<f64>>, Box<dyn Error>> {
    let mut reader = Reader::from_path(shapefile_path)?;
    let mut polygons = Vec::new();

    for record in reader.iter_shapes_and_records() {
        let (shape, _) = record?;

        match shape {
            Shape::Polygon(p) => {
                // exterior ring
                let exterior = LineString::from(
                    p.rings()[0]
                        .points()
                        .iter()
                        .map(|pt| (pt.x, pt.y))
                        .collect::<Vec<_>>(),
                );

                // interior rings (holes)
                let interiors = p.rings()[1..]
                    .iter()
                    .map(|ring| {
                        LineString::from(
                            ring.points()
                                .iter()
                                .map(|pt| (pt.x, pt.y))
                                .collect::<Vec<_>>(),
                        )
                    })
                    .collect::<Vec<_>>();

                let poly = Polygon::new(exterior, interiors);
                polygons.push(poly);
            }
            _ => {
                // skip non-polygons
            }
        }
    }

    Ok(polygons)
}
