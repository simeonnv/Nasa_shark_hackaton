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
    let land_polygons = load_land_polygons(SHAPEFILE_PATH).unwrap();
    let simulation = Arc::new(RwLock::new(Simulation::new(1, &mut rng, &land_polygons)));
    // dbg!(simulation);

    tokio::spawn(rerender_loop(simulation.clone()));

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

async fn rerender_loop(simulation: Arc<RwLock<Simulation>>) -> Result<()> {
    loop {
        println!("simulation tick");
        {
            let mut simulation = simulation.write().await;
            simulation.step(
                1.0 / TPS as f64, // dt
                100.0,            // perception_radius
                2.0,              // separation_distance
                0.01,             // cohesion_strength
                0.05,             // separation_strength
                0.02,             // alignment_strength
            );
        }
        tokio::time::sleep(Duration::from_millis(1000 / TPS)).await;
    }
}
