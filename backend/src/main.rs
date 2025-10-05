mod generate_point;
use std::net::SocketAddr;
use std::ops::Deref;
use std::sync::Arc;
use std::time::Duration;

use futures_util::SinkExt;
use futures_util::StreamExt;
pub use generate_point::random_point;
pub use generate_point::random_point_in_water;

mod tick;

mod shark;
use geo::Point;
use geo::Polygon;
pub use shark::Shark;

mod simulation;
pub use simulation::Simulation;

mod load_land_polygons;
pub use load_land_polygons::load_land_polygons;

use tokio::net::TcpListener;
use tokio::net::TcpStream;
use tokio::sync::RwLock;
use tokio_tungstenite::accept_async;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::tungstenite::Result;

use crate::tick::TPS;

pub const SHAPEFILE_PATH: &'static str = "land/ne_110m_land.shp";

#[tokio::main]
async fn main() -> Result<()> {
    let mut rng = rand::rng();
    let mut attraction_points = vec![];

    attraction_points.push(Point::new(167.0, -28.299544));
    attraction_points.push(Point::new(41.202671, -39.916056));
    attraction_points.push(Point::new(30.744196, 131.833367));
    attraction_points.push(Point::new(39.361909, -21.325484));
    attraction_points.push(Point::new(39.903416, -66.289550));
    attraction_points.push(Point::new(36.666216, -148.250722));
    attraction_points.push(Point::new(-143.194445, -18.377986));
    attraction_points.push(Point::new(171.527249, -13.651325));
    attraction_points.push(Point::new(186.228940, -26.049380));
    attraction_points.push(Point::new(55.205441, -28.730335));
    attraction_points.push(Point::new(52.043905, -36.138984));
    attraction_points.push(Point::new(-24.249463, 36.257563));
    attraction_points.push(Point::new(-48.143093, 44.800109));
    attraction_points.push(Point::new(-64.082863, 37.817378));
    attraction_points.push(Point::new(-68.246077, 32.671749));
    attraction_points.push(Point::new(-136.808344, 37.278424));
    attraction_points.push(Point::new(-142.077571, 28.743580));
    attraction_points.push(Point::new(-160.695504, 20.771523));

    let land_polygons = Arc::new(load_land_polygons(SHAPEFILE_PATH).unwrap());
    let simulation = Arc::new(RwLock::new(Simulation::new(
        300,
        &mut rng,
        &land_polygons,
        attraction_points,
    )));

    tokio::spawn(rerender_loop(simulation.clone(), land_polygons.clone()));

    println!("server is up vro");
    let server = TcpListener::bind("0.0.0.0:25555")
        .await
        .expect("Failed to bind to address");

    loop {
        let (stream, addr) = server.accept().await?;
        tokio::spawn(handle_connection(stream, addr, simulation.clone()));
    }
}

async fn handle_connection(
    stream: TcpStream,
    addr: SocketAddr,
    simulation: Arc<RwLock<Simulation>>,
) -> Result<()> {
    let ws_stream = accept_async(stream).await?;
    println!("New WebSocket connection: {}", addr);

    let (mut write, _read) = ws_stream.split();

    loop {
        tokio::time::sleep(Duration::from_millis(1000 / TPS)).await;
        let simulation_json;
        {
            let sim = simulation.read().await;
            simulation_json = serde_json::to_string(sim.deref()).unwrap();
        }

        // dbg!(&simulation_json);

        write.send(Message::Text(simulation_json.into())).await?;
    }
}

// let lat = rng.random_range(-90.0..=90.0);
//     let lon = rng.random_range(-180.0..=180.0);

async fn rerender_loop(
    simulation: Arc<RwLock<Simulation>>,
    land_polygons: Arc<Vec<Polygon<f64>>>,
) -> Result<()> {
    let mut ticks = 0;
    let map_bounds = (-180., -85., 180.0, 85.0);
    loop {
        ticks = ticks + 1;
        print!("\x1B[2J\x1B[1;1H");
        println!("ticks: {}", ticks);
        {
            let mut simulation = simulation.write().await;
            simulation.step(
                1.0 / TPS as f64, // dt
                4.0,
                2.0,
                0.1,
                0.1,
                0.05,
                &land_polygons,
                map_bounds,
                10.,
                100.,
                0.5,
                6.0,
                10.,
                0.3,
            );
        }
        tokio::time::sleep(Duration::from_millis(1000 / TPS)).await;
    }
}

//  &mut self,
//         dt: f64,
//         perception_radius: f64,
//         separation_distance: f64,
//         cohesion_strength: f64,
//         separation_strength: f64,
//         alignment_strength: f64,
//         land_shape_file: &[Polygon<f64>],
//         map_bounds: (f64, f64, f64, f64),
//         land_avoid_radius: f64,
//         land_avoid_strength: f64,
//         border_margin: f64,
//         border_strength: f64,
