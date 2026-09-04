//! Both drivers must be usable from one binary. Before the blocking/async
//! split this could not compile, because `async` replaced the blocking API
//! rather than adding to it.

#![cfg(feature = "async")]

use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

#[tokio::test]
async fn blocking_and_async_drivers_coexist() {
    let expectations = [Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])];

    let mut blocking = tmp108::blocking::Tmp108::new_with_a0_gnd(Mock::new(&expectations));
    let temperature = blocking.temperature().unwrap();
    assert!((temperature - 25.0).abs() < 1e-4);
    let mut mock = blocking.destroy();
    mock.done();

    let mut asynchronous = tmp108::asynch::Tmp108::new_with_a0_gnd(Mock::new(&expectations));
    let temperature = asynchronous.temperature().await.unwrap();
    assert!((temperature - 25.0).abs() < 1e-4);
    let mut mock = asynchronous.destroy();
    mock.done();
}
