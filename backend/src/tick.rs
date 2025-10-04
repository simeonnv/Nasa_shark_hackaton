use std::time::Duration;

use lazy_static::lazy_static;
use tokio::{
    sync::{Mutex, RwLock, broadcast},
    time::Instant,
};

pub const TPS: u64 = 10;
lazy_static! {
    pub static ref LAST_TRIGGER: RwLock<Option<Instant>> = RwLock::new(None);
}

pub async fn should_trigger() -> bool {
    let now = Instant::now();

    {
        let last = LAST_TRIGGER.read().await;
        let last = last.unwrap_or(now);
        if now.duration_since(last) < Duration::from_millis(1000 / TPS) {
            return false;
        }
    }

    let mut last = LAST_TRIGGER.write().await;
    *last = Some(now);
    true
}
