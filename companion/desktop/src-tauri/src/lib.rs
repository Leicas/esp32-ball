use serde::Serialize;
use serialport::{SerialPort, SerialPortType};
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tauri::{AppHandle, Emitter, State};

const CONNECTION_EVENT: &str = "connection";
const DEVICE_LINE_EVENT: &str = "device-line";
const COMMAND_QUEUE_SIZE: usize = 16;
const SERIAL_TIMEOUT: Duration = Duration::from_millis(10);
const TELEMETRY_FRESH_FOR: Duration = Duration::from_millis(500);
const RECONNECT_DELAY: Duration = Duration::from_millis(300);
const MAX_LINE_BYTES: usize = 1024;

#[derive(Serialize)]
struct PortInfo {
    name: String,
    description: String,
}

#[derive(Clone, Serialize)]
struct ConnectionEvent {
    state: String,
    port: String,
    detail: String,
}

struct WorkerSession {
    port: String,
    stop: Arc<AtomicBool>,
    telemetry_live: Arc<AtomicBool>,
    last_telemetry: Arc<Mutex<Option<Instant>>>,
    commands: SyncSender<u8>,
    worker: Option<JoinHandle<()>>,
}

#[derive(Default)]
struct AppState {
    session: Mutex<Option<WorkerSession>>,
}

#[tauri::command]
fn list_ports() -> Vec<PortInfo> {
    let Ok(ports) = serialport::available_ports() else {
        return Vec::new();
    };
    ports
        .into_iter()
        .map(|port| PortInfo {
            name: port.port_name,
            description: describe_port(&port.port_type),
        })
        .collect()
}

fn describe_port(kind: &SerialPortType) -> String {
    match kind {
        SerialPortType::UsbPort(info) => {
            let model = info.product.as_deref().or(info.manufacturer.as_deref());
            if info.vid == 0x303A && info.pid == 0x1001 {
                format!("Espressif USB Serial/JTAG ({:04X}:{:04X})", info.vid, info.pid)
            } else if let Some(model) = model {
                format!("{model} ({:04X}:{:04X})", info.vid, info.pid)
            } else {
                format!("USB serial ({:04X}:{:04X})", info.vid, info.pid)
            }
        }
        SerialPortType::BluetoothPort => "Bluetooth serial".to_string(),
        SerialPortType::PciPort => "PCI serial".to_string(),
        SerialPortType::Unknown => "Serial port".to_string(),
    }
}

#[tauri::command]
fn connect(app: AppHandle, state: State<'_, AppState>, port: String) -> Result<(), String> {
    if port.trim().is_empty() {
        return Err("Choose a serial port first".to_string());
    }

    let mut slot = state
        .session
        .lock()
        .map_err(|_| "Serial state is unavailable".to_string())?;
    if let Some(current) = slot.as_ref() {
        if current.port == port && !current.stop.load(Ordering::Acquire) {
            return Ok(());
        }
    }

    let previous = slot.take();
    if let Some(old) = previous.as_ref() {
        old.stop.store(true, Ordering::Release);
    }

    let stop = Arc::new(AtomicBool::new(false));
    let telemetry_live = Arc::new(AtomicBool::new(false));
    let last_telemetry = Arc::new(Mutex::new(None));
    let (commands, receiver) = mpsc::sync_channel(COMMAND_QUEUE_SIZE);
    let worker_stop = Arc::clone(&stop);
    let worker_live = Arc::clone(&telemetry_live);
    let worker_last = Arc::clone(&last_telemetry);
    let worker_app = app.clone();
    let worker_port = port.clone();
    let previous_worker = previous.and_then(|mut old| old.worker.take());

    emit_connection(&app, "Connecting", &port, "Opening USB serial");
    let worker = thread::Builder::new()
        .name("rolling-stone-serial".to_string())
        .spawn(move || {
            if let Some(previous_worker) = previous_worker {
                let _ = previous_worker.join();
            }
            if worker_stop.load(Ordering::Acquire) {
                emit_connection(&worker_app, "Disconnected", &worker_port, "Connection cancelled");
                return;
            }
            serial_worker(worker_app, worker_port, worker_stop, worker_live, worker_last, receiver);
        })
        .map_err(|error| error.to_string())?;

    *slot = Some(WorkerSession {
        port,
        stop,
        telemetry_live,
        last_telemetry,
        commands,
        worker: Some(worker),
    });
    Ok(())
}

#[tauri::command]
fn disconnect(app: AppHandle, state: State<'_, AppState>) -> Result<(), String> {
    let slot = state
        .session
        .lock()
        .map_err(|_| "Serial state is unavailable".to_string())?;
    if let Some(session) = slot.as_ref() {
        session.stop.store(true, Ordering::Release);
        session.telemetry_live.store(false, Ordering::Release);
        emit_connection(&app, "Disconnecting", &session.port, "Closing serial port");
    } else {
        emit_connection(&app, "Disconnected", "", "No serial port is open");
    }
    Ok(())
}

#[tauri::command]
fn send_command(state: State<'_, AppState>, command: String) -> Result<(), String> {
    let Some(byte) = command_byte(&command) else {
        return Err("Unsupported command".to_string());
    };
    let slot = state
        .session
        .lock()
        .map_err(|_| "Serial state is unavailable".to_string())?;
    let Some(session) = slot.as_ref() else {
        return Err("Connect to the board before sending commands".to_string());
    };
    if session.stop.load(Ordering::Acquire) {
        return Err("The serial connection is closing".to_string());
    }
    if !session.telemetry_live.load(Ordering::Acquire)
        || !is_fresh(read_last_telemetry(&session.last_telemetry), Instant::now())
    {
        return Err("Commands require telemetry received within the last 0.5 seconds".to_string());
    }
    match session.commands.try_send(byte) {
        Ok(()) => Ok(()),
        Err(TrySendError::Full(_)) => Err("Serial command queue is full".to_string()),
        Err(TrySendError::Disconnected(_)) => Err("Serial connection is unavailable".to_string()),
    }
}

fn serial_worker(
    app: AppHandle,
    port_name: String,
    stop: Arc<AtomicBool>,
    telemetry_live: Arc<AtomicBool>,
    last_telemetry: Arc<Mutex<Option<Instant>>>,
    commands: Receiver<u8>,
) {
    let mut has_connected = false;
    while !stop.load(Ordering::Acquire) {
        let opened = serialport::new(&port_name, 115_200).timeout(SERIAL_TIMEOUT).open();
        let mut port = match opened {
            Ok(port) => port,
            Err(error) => {
                telemetry_live.store(false, Ordering::Release);
                clear_last_telemetry(&last_telemetry);
                drop_commands(&commands);
                emit_connection(
                    &app,
                    "Reconnecting",
                    &port_name,
                    &format!("Could not open port: {error}"),
                );
                interruptible_delay(&stop, RECONNECT_DELAY);
                continue;
            }
        };

        clear_last_telemetry(&last_telemetry);
        telemetry_live.store(false, Ordering::Release);
        if let Err(error) = port.write_all(b"?").and_then(|_| port.flush()) {
            drop(port);
            emit_connection(
                &app,
                "Reconnecting",
                &port_name,
                &format!("Initial status request failed: {error}"),
            );
            drop_commands(&commands);
            interruptible_delay(&stop, RECONNECT_DELAY);
            continue;
        }
        let detail = if has_connected {
            "Port reopened; waiting for fresh telemetry"
        } else {
            "Port open; waiting for telemetry"
        };
        emit_connection(&app, "Connecting", &port_name, detail);

        let mut framer = LineFramer::default();
        let mut line_bytes = [0_u8; 256];
        let mut live_on_this_open = false;
        let mut failure: Option<String> = None;

        while !stop.load(Ordering::Acquire) {
            let fresh = is_fresh(read_last_telemetry(&last_telemetry), Instant::now());
            if !fresh {
                telemetry_live.store(false, Ordering::Release);
                drop_commands(&commands);
                if live_on_this_open {
                    live_on_this_open = false;
                    emit_connection(
                        &app,
                        "Stale",
                        &port_name,
                        "No valid telemetry for more than 0.5 seconds",
                    );
                }
            }

            if fresh {
                if let Err(error) = write_queued_commands(&mut *port, &commands, &last_telemetry) {
                    failure = Some(format!("Serial write failed: {error}"));
                    break;
                }
            }

            match port.read(&mut line_bytes) {
                Ok(0) => {}
                Ok(count) => {
                    for line in framer.push(&line_bytes[..count]) {
                        let valid_telemetry = is_valid_telemetry_line(&line);
                        let _ = app.emit(DEVICE_LINE_EVENT, line.clone());
                        if valid_telemetry {
                            if let Ok(mut last) = last_telemetry.lock() {
                                *last = Some(Instant::now());
                            }
                            if !telemetry_live.swap(true, Ordering::AcqRel) {
                                live_on_this_open = true;
                                has_connected = true;
                                emit_connection(
                                    &app,
                                    "Connected",
                                    &port_name,
                                    "Receiving fresh telemetry",
                                );
                            }
                        }
                    }
                }
                Err(error)
                    if error.kind() == io::ErrorKind::TimedOut
                        || error.kind() == io::ErrorKind::WouldBlock => {}
                Err(error) => {
                    failure = Some(format!("Serial read failed: {error}"));
                    break;
                }
            }
        }

        // The port belongs to this worker and is dropped before the next retry.
        drop(port);
        telemetry_live.store(false, Ordering::Release);
        clear_last_telemetry(&last_telemetry);
        drop_commands(&commands);
        if stop.load(Ordering::Acquire) {
            emit_connection(&app, "Disconnected", &port_name, "Serial port closed");
            return;
        }
        let detail = failure.unwrap_or_else(|| "Serial transport stopped unexpectedly".to_string());
        emit_connection(&app, "Reconnecting", &port_name, &detail);
        interruptible_delay(&stop, RECONNECT_DELAY);
    }

    telemetry_live.store(false, Ordering::Release);
    drop_commands(&commands);
    emit_connection(&app, "Disconnected", &port_name, "Serial port closed");
}

fn write_queued_commands(
    port: &mut dyn SerialPort,
    commands: &Receiver<u8>,
    last_telemetry: &Mutex<Option<Instant>>,
) -> io::Result<()> {
    loop {
        match commands.try_recv() {
            Ok(command) => {
                if !is_fresh(read_last_telemetry(last_telemetry), Instant::now()) {
                    drop_commands(commands);
                    return Ok(());
                }
                port.write_all(&[command])?;
                port.flush()?;
            }
            Err(TryRecvError::Empty | TryRecvError::Disconnected) => return Ok(()),
        }
    }
}

fn command_byte(command: &str) -> Option<u8> {
    let bytes = command.as_bytes();
    if bytes.len() != 1
        || !matches!(bytes[0], b'r' | b'l' | b't' | b's' | b'm' | b'n' | b'v' | b'g' | b'b' | b'+' | b'-' | b'?')
    {
        return None;
    }
    Some(bytes[0])
}

fn is_valid_telemetry_line(line: &str) -> bool {
    let (body, expected_count) = if let Some(body) = line.strip_prefix('$') {
        (body, 3)
    } else if let Some(body) = line.strip_prefix('@') {
        (body, 0)
    } else {
        return false;
    };
    let values = body.split(',').collect::<Vec<_>>();
    (if expected_count == 0 { (8..=28).contains(&values.len()) && (values.len() - 8) % 4 == 0 } else { values.len() == expected_count })
        && values
            .iter()
            .all(|value| value.trim().parse::<f32>().is_ok_and(|number| number.is_finite()))
}

fn is_fresh(last: Option<Instant>, now: Instant) -> bool {
    last.is_some_and(|last| now.saturating_duration_since(last) <= TELEMETRY_FRESH_FOR)
}

fn read_last_telemetry(last: &Mutex<Option<Instant>>) -> Option<Instant> {
    last.lock().ok().and_then(|value| *value)
}

fn clear_last_telemetry(last: &Mutex<Option<Instant>>) {
    if let Ok(mut value) = last.lock() {
        *value = None;
    }
}

fn drop_commands(commands: &Receiver<u8>) {
    while commands.try_recv().is_ok() {}
}

fn interruptible_delay(stop: &AtomicBool, duration: Duration) {
    let deadline = Instant::now() + duration;
    while !stop.load(Ordering::Acquire) && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(10));
    }
}

fn emit_connection(app: &AppHandle, state: &str, port: &str, detail: &str) {
    let _ = app.emit(
        CONNECTION_EVENT,
        ConnectionEvent {
            state: state.to_string(),
            port: port.to_string(),
            detail: detail.to_string(),
        },
    );
}

#[derive(Default)]
struct LineFramer {
    buffer: Vec<u8>,
    discarding_oversize: bool,
}

impl LineFramer {
    fn push(&mut self, bytes: &[u8]) -> Vec<String> {
        let mut lines = Vec::new();
        for byte in bytes.iter().copied() {
            if self.discarding_oversize {
                if byte == b'\n' {
                    self.discarding_oversize = false;
                }
                continue;
            }
            if byte == b'\n' {
                if self.buffer.last() == Some(&b'\r') {
                    self.buffer.pop();
                }
                lines.push(String::from_utf8_lossy(&self.buffer).into_owned());
                self.buffer.clear();
            } else if self.buffer.len() < MAX_LINE_BYTES {
                self.buffer.push(byte);
            } else {
                self.buffer.clear();
                self.discarding_oversize = true;
            }
        }
        lines
    }
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .manage(AppState::default())
        .invoke_handler(tauri::generate_handler![list_ports, connect, disconnect, send_command])
        .run(tauri::generate_context!())
        .expect("error while running Rolling Stone desktop app");
}

#[cfg(test)]
mod tests {
    use super::{command_byte, is_fresh, is_valid_telemetry_line, LineFramer, MAX_LINE_BYTES};
    use std::time::{Duration, Instant};

    #[test]
    fn line_framer_handles_fragmentation_and_multiple_lines() {
        let mut framer = LineFramer::default();
        assert!(framer.push(b"$0.1,500").is_empty());
        assert_eq!(
            framer.push(b",0.2\r\n# mode=TEXTURED\n"),
            vec!["$0.1,500,0.2", "# mode=TEXTURED"]
        );
    }

    #[test]
    fn line_framer_discards_oversize_frame_without_corrupting_next_line() {
        let mut framer = LineFramer::default();
        let mut bytes = vec![b'x'; MAX_LINE_BYTES + 1];
        bytes.extend_from_slice(b"\n$0.1,500,0.2\n");
        assert_eq!(framer.push(&bytes), vec!["$0.1,500,0.2"]);
    }

    #[test]
    fn command_whitelist_accepts_only_single_supported_bytes() {
        for command in ["r", "l", "t", "s", "m", "n", "v", "g", "b", "+", "-", "?"] {
            assert!(command_byte(command).is_some());
        }
        for command in ["", "rt", "R", " ", "\n", "λ"] {
            assert!(command_byte(command).is_none());
        }
    }

    #[test]
    fn telemetry_validation_rejects_status_and_nonfinite_values() {
        assert!(is_valid_telemetry_line("$0.1,500,0.2"));
        assert!(is_valid_telemetry_line("@0,0,1,0,1,0,0,0,1,2,3,1,4,5,6,1,7,8,9,1"));
        assert!(is_valid_telemetry_line("@0,0,1,0,1,0,0,0"));
        assert!(is_valid_telemetry_line(&format!("@0,0,1,0,1,0,0,0{}", ",5,5,5,5".repeat(5))));
        assert!(!is_valid_telemetry_line(&format!("@0,0,1,0,1,0,0,0{}", ",5,5,5,5".repeat(6))));
        assert!(!is_valid_telemetry_line("# mode=ROLLING"));
        assert!(!is_valid_telemetry_line("$nan,500,0.2"));
        assert!(!is_valid_telemetry_line("$0.1,inf,0.2"));
        assert!(!is_valid_telemetry_line("@0,0,1,0,1,0,0,0,1,2,3,1,4,5,6,1,7,8,9"));
        assert!(!is_valid_telemetry_line("@0,0,1,0,1,0,0,0,1,2,3,1,4,5,6,1,7,8,9,1,10"));
    }

    #[test]
    fn telemetry_expires_after_half_a_second() {
        let now = Instant::now();
        assert!(is_fresh(Some(now - Duration::from_millis(400)), now));
        assert!(!is_fresh(Some(now - Duration::from_millis(501)), now));
        assert!(!is_fresh(None, now));
    }
}
