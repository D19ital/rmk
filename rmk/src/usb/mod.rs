use core::future::Future;

use embassy_futures::join::join4;
use embassy_futures::select::{Either, select};
use embassy_sync::signal::Signal;
#[cfg(feature = "host")]
use embassy_usb::class::hid::HidReaderWriter;
use embassy_usb::class::hid::{HidReader, HidWriter, ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use embassy_usb::driver::Driver;
use embassy_usb::{Builder, Handler, UsbDevice};
use rmk_types::connection::{ConnectionType, UsbState};
use static_cell::StaticCell;
use usbd_hid::descriptor::AsInputReport;

use crate::RawMutex;
use crate::channel::{QueuedReportPayload, USB_REPORT_CHANNEL, WideMouseReport};
use crate::config::DeviceConfig;
use crate::core_traits::Runnable;
#[cfg(feature = "steno")]
use crate::hid::StenoReport;
#[cfg(feature = "host")]
use crate::hid::ViaReport;
use crate::hid::{
    CompositeReport, CompositeReportType, HidError, HidWriterTrait, KeyboardReport, Report, run_led_reader,
};
use crate::light::UsbLedReader;
use crate::state::{current_usb_state, set_usb_state};

pub(crate) static USB_REMOTE_WAKEUP: Signal<RawMutex, ()> = Signal::new();
static USB_STATE_CHANGED: Signal<RawMutex, ()> = Signal::new();

fn set_usb_state_and_notify(state: UsbState) {
    let previous = current_usb_state();
    set_usb_state(state);
    if previous != state {
        USB_STATE_CHANGED.signal(());
    }
}

/// Return whether a report can be submitted immediately without touching an
/// inactive USB endpoint.
///
/// The nRF52840 endpoint driver can enter a blocking DMA wait when an IN write
/// starts while USBD is in low-power mode. Do not hold a report while waiting
/// for resume: the bounded report queues would eventually back-pressure the
/// whole input pipeline. A report received during suspend only requests remote
/// wakeup and is then dropped; a later input is delivered after the host has
/// resumed.
fn usb_report_write_ready() -> bool {
    match current_usb_state() {
        UsbState::Configured => true,
        UsbState::Suspended => {
            USB_REMOTE_WAKEUP.signal(());
            false
        }
        UsbState::Disabled | UsbState::Enabled => false,
    }
}

/// Run one HID IN write only while USB remains configured.
///
/// Checking the state before starting the write is not sufficient: the nRF
/// endpoint can already be waiting for the host's next IN token when suspend
/// is published. Race that pending write against the USB state transition so
/// dropping the future releases the writer before its bounded input queue can
/// back-pressure the keyboard task. The state signal is polled first, making
/// a simultaneous suspend win over starting another endpoint transaction.
async fn write_while_usb_configured<F: Future>(write: F) -> Option<F::Output> {
    // Discard a transition already handled by the previous iteration, then
    // read the authoritative state. If a new transition races this reset, the
    // state check observes it; if it happens afterwards, the select does.
    USB_STATE_CHANGED.reset();
    if !usb_report_write_ready() {
        return None;
    }

    match select(USB_STATE_CHANGED.wait(), write).await {
        Either::First(()) => {
            if current_usb_state() == UsbState::Suspended {
                USB_REMOTE_WAKEUP.signal(());
            }
            None
        }
        Either::Second(output) => Some(output),
    }
}

/// Borrowed view over the USB HID IN endpoints used by the report writer task.
///
/// `UsbTransport` owns the USB device, readers, writers, host interface, and
/// optional logger; `run` borrows those fields separately so they can run
/// concurrently without moving the whole transport into one task.
pub(crate) struct UsbKeyboardWriter<'a, 'd, D: Driver<'d>> {
    pub(crate) keyboard_writer: &'a mut HidWriter<'d, D, 8>,
    pub(crate) other_writer: &'a mut HidWriter<'d, D, 9>,
    #[cfg(feature = "steno")]
    pub(crate) steno_writer: &'a mut HidWriter<'d, D, 9>,
}

impl<'a, 'd, D: Driver<'d>> UsbKeyboardWriter<'a, 'd, D> {
    pub(crate) fn new(
        keyboard_writer: &'a mut HidWriter<'d, D, 8>,
        other_writer: &'a mut HidWriter<'d, D, 9>,
        #[cfg(feature = "steno")] steno_writer: &'a mut HidWriter<'d, D, 9>,
    ) -> Self {
        Self {
            keyboard_writer,
            other_writer,
            #[cfg(feature = "steno")]
            steno_writer,
        }
    }

    pub(crate) async fn run_writer(&mut self) -> ! {
        loop {
            match USB_REPORT_CHANNEL.receive().await.into_payload() {
                QueuedReportPayload::Hid(report) => {
                    if let Some(Err(e)) = write_while_usb_configured(self.write_report(&report)).await {
                        error!("Failed to send report: {:?}", e);
                    }
                }
                QueuedReportPayload::WideMouse(report) => self.write_wide_mouse(report).await,
            }
        }
    }

    async fn write_wide_mouse(&mut self, mut mouse: WideMouseReport) {
        loop {
            let has_relative_motion = mouse.x != 0 || mouse.y != 0 || mouse.wheel != 0 || mouse.pan != 0;
            let (x, y, wheel, pan) = if has_relative_motion {
                crate::mouse_chunk::take_vector_chunk(&mut mouse.x, &mut mouse.y, &mut mouse.wheel, &mut mouse.pan)
            } else {
                (0, 0, 0, 0)
            };
            let report = Report::MouseReport(usbd_hid::descriptor::MouseReport {
                buttons: mouse.buttons,
                x,
                y,
                wheel,
                pan,
            });

            match write_while_usb_configured(self.write_report(&report)).await {
                Some(Ok(_)) => {}
                Some(Err(e)) => error!("Failed to send report: {:?}", e),
                None => return,
            }

            if mouse.x == 0 && mouse.y == 0 && mouse.wheel == 0 && mouse.pan == 0 {
                return;
            }
        }
    }

    async fn write_composite<R: AsInputReport>(
        &mut self,
        kind: CompositeReportType,
        report: &R,
    ) -> Result<usize, HidError> {
        let mut buf = [0u8; 9];
        buf[0] = kind as u8;
        let n = report
            .serialize(&mut buf[1..])
            .map_err(|_| HidError::ReportSerializeError)?;
        self.other_writer
            .write(&buf[0..n + 1])
            .await
            .map_err(HidError::UsbEndpointError)?;
        Ok(n)
    }
}

impl<'d, D: Driver<'d>> HidWriterTrait for UsbKeyboardWriter<'_, 'd, D> {
    type ReportType = Report;

    async fn write_report(&mut self, report: &Self::ReportType) -> Result<usize, HidError> {
        match report {
            Report::KeyboardReport(keyboard_report) => {
                let mut buf: [u8; 8] = [0; 8];
                let n: usize = keyboard_report
                    .serialize(&mut buf)
                    .map_err(|_| HidError::ReportSerializeError)?;
                self.keyboard_writer
                    .write(&buf[0..n])
                    .await
                    .map_err(HidError::UsbEndpointError)?;
                Ok(n)
            }
            Report::MouseReport(r) => self.write_composite(CompositeReportType::Mouse, r).await,
            Report::MediaKeyboardReport(r) => self.write_composite(CompositeReportType::Media, r).await,
            Report::SystemControlReport(r) => self.write_composite(CompositeReportType::System, r).await,
            #[cfg(feature = "steno")]
            Report::StenoReport(steno_report) => {
                let mut buf: [u8; 9] = [0; 9];
                let n = steno_report
                    .serialize(&mut buf)
                    .map_err(|_| HidError::ReportSerializeError)?;

                // Cap on how long a steno report write is allowed to block. The host only
                // drains the steno IN endpoint while Plover is running; without this cap the
                // writer task stalls indefinitely (and starves keyboard reports) whenever
                // Plover is absent.
                match embassy_time::with_timeout(
                    embassy_time::Duration::from_millis(5),
                    self.steno_writer.write(&buf[0..n]),
                )
                .await
                {
                    Ok(Ok(())) => {}
                    Ok(Err(e)) => return Err(HidError::UsbEndpointError(e)),
                    Err(_) => {} // Plover not reading; drop this report and continue
                }
                Ok(n)
            }
        }
    }
}

pub(crate) fn new_usb_builder<'d, D: Driver<'d>>(driver: D, keyboard_config: DeviceConfig<'d>) -> Builder<'d, D> {
    let mut usb_config = embassy_usb::Config::new(keyboard_config.vid, keyboard_config.pid);
    usb_config.manufacturer = Some(keyboard_config.manufacturer);
    usb_config.product = Some(keyboard_config.product_name);
    usb_config.serial_number = Some(keyboard_config.serial_number);
    usb_config.device_release = keyboard_config.device_release;
    usb_config.max_power = 450;
    usb_config.supports_remote_wakeup = true;

    // Required for windows compatibility.
    usb_config.max_packet_size_0 = 64;
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    // Extra interfaces (usb_log, steno, dfu) overflow the 128-byte config descriptor buffer.
    #[cfg(any(feature = "usb_log", feature = "steno", feature = "dfu"))]
    const USB_BUF_SIZE: usize = 256;
    #[cfg(not(any(feature = "usb_log", feature = "steno", feature = "dfu")))]
    const USB_BUF_SIZE: usize = 128;

    // Control buffer must be large enough for the largest DFU transfer block.
    #[cfg(feature = "dfu")]
    const CONTROL_BUF_SIZE: usize = ::rmk::dfu::BLOCK_SIZE_DFU;
    #[cfg(not(feature = "dfu"))]
    const CONTROL_BUF_SIZE: usize = USB_BUF_SIZE;

    static CONFIG_DESC: StaticCell<[u8; USB_BUF_SIZE]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 16]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 16]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut CONFIG_DESC.init([0; USB_BUF_SIZE])[..],
        &mut BOS_DESC.init([0; 16])[..],
        &mut MSOS_DESC.init([0; 16])[..],
        &mut CONTROL_BUF.init([0; CONTROL_BUF_SIZE])[..],
    );

    static device_handler: StaticCell<UsbDeviceHandler> = StaticCell::new();
    builder.handler(device_handler.init(UsbDeviceHandler::new()));

    builder
}

/// USB transport runnable. Owns the embassy-usb device + every HID
/// reader/writer pair and runs them concurrently for the lifetime of the
/// program.
pub struct UsbTransport<D: Driver<'static>> {
    device: UsbDevice<'static, D>,
    keyboard_reader: HidReader<'static, D, 1>,
    keyboard_writer: HidWriter<'static, D, 8>,
    other_writer: HidWriter<'static, D, 9>,
    #[cfg(feature = "steno")]
    steno_writer: HidWriter<'static, D, 9>,
    #[cfg(feature = "host")]
    host_rw: HidReaderWriter<'static, D, 32, 32>,
    #[cfg(feature = "usb_log")]
    logger: Option<embassy_usb::class::cdc_acm::CdcAcmClass<'static, D>>,
}

impl<D: Driver<'static>> UsbTransport<D> {
    pub fn new(driver: D, device_config: DeviceConfig<'static>) -> Self {
        // nRF chips don't have a stable USB serial number unless one is derived
        // from the FICR. Override here so user code doesn't have to know.
        #[cfg(feature = "_nrf_ble")]
        let device_config = {
            let mut device_config = device_config;
            device_config.serial_number = crate::ble::nrf::get_serial_number();
            device_config
        };
        let mut builder: Builder<'static, D> = new_usb_builder(driver, device_config);
        // Linux's usbhid driver auto-enables power/wakeup when it probes a
        // boot-protocol keyboard, so advertise Boot/Keyboard on the primary
        // HID interface.
        let keyboard_rw = add_usb_reader_writer!(
            &mut builder,
            KeyboardReport,
            1,
            8,
            8,
            ::embassy_usb::class::hid::HidSubclass::Boot,
            ::embassy_usb::class::hid::HidBootProtocol::Keyboard
        );
        let other_writer = add_usb_writer!(&mut builder, CompositeReport, 9, 16);
        #[cfg(feature = "steno")]
        let steno_writer = add_usb_writer!(&mut builder, StenoReport, 9, 16);
        #[cfg(feature = "host")]
        let host_rw = add_usb_reader_writer!(&mut builder, ViaReport, 32, 32, 32);
        #[cfg(feature = "usb_log")]
        let logger = Some(add_usb_logger!(&mut builder));

        #[cfg(feature = "dfu")]
        {
            let product_name = device_config.product_name;
            #[cfg(any(feature = "dfu_rp", feature = "dfu_nrf"))]
            if let Some(mgr) = ::rmk::dfu::get_manager() {
                ::rmk::dfu::register_dfu_interface(
                    &mut builder,
                    mgr,
                    product_name,
                    #[cfg(feature = "dfu_split")]
                    crate::SPLIT_PERIPHERALS_NUM,
                );
            }
        }
        let (keyboard_reader, keyboard_writer) = keyboard_rw.split();
        let device = builder.build();

        Self {
            device,
            keyboard_reader,
            keyboard_writer,
            other_writer,
            #[cfg(feature = "steno")]
            steno_writer,
            #[cfg(feature = "host")]
            host_rw,
            #[cfg(feature = "usb_log")]
            logger,
        }
    }
}

impl<D: Driver<'static>> Runnable for UsbTransport<D> {
    async fn run(&mut self) -> ! {
        let Self {
            device,
            keyboard_reader,
            keyboard_writer,
            other_writer,
            #[cfg(feature = "steno")]
            steno_writer,
            #[cfg(feature = "host")]
            host_rw,
            #[cfg(feature = "usb_log")]
            logger,
        } = self;

        let usb_device_task = async {
            loop {
                device.run_until_suspend().await;
                match select(device.wait_resume(), USB_REMOTE_WAKEUP.wait()).await {
                    Either::First(_) => {
                        // The host resumed before the queued request won the
                        // select. That request belongs to the writer which is
                        // now unblocked, so it must not wake the next suspend.
                        USB_REMOTE_WAKEUP.reset();
                        continue;
                    }
                    Either::Second(_) => {
                        info!("USB remote wakeup requested");
                        // Whether this succeeds or not, loop back to the
                        // resume/wakeup select. A failed attempt must never
                        // park the USB task in a bare wait_resume(), otherwise
                        // every later wakeup request is ignored until reset.
                        if let Err(e) = device.remote_wakeup().await {
                            warn!("Remote wakeup failed: {:?}", e);
                        }
                    }
                }
            }
        };

        let mut writer = UsbKeyboardWriter::new(
            keyboard_writer,
            other_writer,
            #[cfg(feature = "steno")]
            steno_writer,
        );
        let writer_task = writer.run_writer();

        let mut led_reader = UsbLedReader::new(keyboard_reader);
        let led_task = run_led_reader(&mut led_reader, ConnectionType::Usb);

        let host_and_extras = async {
            #[cfg(feature = "host")]
            let host_task = crate::host::usb::run_usb_host(host_rw);
            #[cfg(not(feature = "host"))]
            let host_task = core::future::pending::<()>();

            #[cfg(feature = "usb_log")]
            {
                let logger_class = logger.take().expect("UsbTransport::run called twice");
                let logger_fut = embassy_usb_logger::with_custom_style!(
                    1024,
                    log::LevelFilter::Debug,
                    logger_class,
                    |record, writer| {
                        use core::fmt::Write;
                        let ms = embassy_time::Instant::now().as_millis();
                        let _ = write!(writer, "[{:>8}ms {:5}] {}\r\n", ms, record.level(), record.args());
                    }
                );
                embassy_futures::join::join(host_task, logger_fut).await;
            }
            #[cfg(not(feature = "usb_log"))]
            host_task.await;
        };

        join4(usb_device_task, writer_task, led_task, host_and_extras).await;
        unreachable!("UsbTransport sub-tasks must run forever");
    }
}

#[cfg(feature = "usb_log")]
macro_rules! add_usb_logger {
    ($usb_builder:expr) => {{
        use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
        use static_cell::StaticCell;

        // The usb logger can be only initialized once, so just use a fixed name for the state
        static LOGGER_STATE: StaticCell<State> = StaticCell::new();
        let state = LOGGER_STATE.init(State::new());
        CdcAcmClass::new($usb_builder, state, 64)
    }};
}

macro_rules! add_usb_writer {
    ($usb_builder:expr, $descriptor:ty, $n:expr) => {
        $crate::usb::add_usb_writer!($usb_builder, $descriptor, $n, 64)
    };
    // Size $max_packet to the actual report to conserve Packet Memory Area on tight parts.
    ($usb_builder:expr, $descriptor:ty, $n:expr, $max_packet:expr) => {{
        // `paste` generates per-descriptor `static`s so each writer keeps its own State/Handler.
        use usbd_hid::descriptor::SerializedDescriptor;
        paste::paste! {
            static [<$descriptor:snake:upper _STATE>]: ::static_cell::StaticCell<::embassy_usb::class::hid::State> = ::static_cell::StaticCell::new();
            static [<$descriptor:snake:upper _HANDLER>]: ::static_cell::StaticCell<$crate::usb::UsbRequestHandler> = ::static_cell::StaticCell::new();
        }

        let state = paste::paste! { [<$descriptor:snake:upper _STATE>].init(::embassy_usb::class::hid::State::new()) };
        let request_handler = paste::paste! { [<$descriptor:snake:upper _HANDLER>].init($crate::usb::UsbRequestHandler {}) };

        let hid_config = ::embassy_usb::class::hid::Config {
            report_descriptor: <$descriptor>::desc(),
            request_handler: Some(request_handler),
            poll_ms: 1,
            max_packet_size: $max_packet,
            hid_subclass: ::embassy_usb::class::hid::HidSubclass::No,
            hid_boot_protocol: ::embassy_usb::class::hid::HidBootProtocol::None,
        };

        let rw: ::embassy_usb::class::hid::HidWriter<_, $n> = ::embassy_usb::class::hid::HidWriter::new($usb_builder, state, hid_config);
        rw
    }};
}

macro_rules! add_usb_reader_writer {
    ($usb_builder:expr, $descriptor:ty, $read_n:expr, $write_n:expr) => {
        $crate::usb::add_usb_reader_writer!($usb_builder, $descriptor, $read_n, $write_n, 64)
    };
    // Size $max_packet to the actual report to conserve Packet Memory Area on tight parts.
    ($usb_builder:expr, $descriptor:ty, $read_n:expr, $write_n:expr, $max_packet:expr) => {
        $crate::usb::add_usb_reader_writer!(
            $usb_builder, $descriptor, $read_n, $write_n, $max_packet,
            ::embassy_usb::class::hid::HidSubclass::No,
            ::embassy_usb::class::hid::HidBootProtocol::None
        )
    };
    ($usb_builder:expr, $descriptor:ty, $read_n:expr, $write_n:expr, $max_packet:expr, $subclass:expr, $protocol:expr) => {{
        // `paste` generates per-descriptor `static`s so each reader/writer keeps its own State/Handler.
        use usbd_hid::descriptor::SerializedDescriptor;
        paste::paste! {
            static [<$descriptor:snake:upper _STATE>]: ::static_cell::StaticCell<::embassy_usb::class::hid::State> = ::static_cell::StaticCell::new();
            static [<$descriptor:snake:upper _HANDLER>]: ::static_cell::StaticCell<$crate::usb::UsbRequestHandler> = ::static_cell::StaticCell::new();
        }

        let state = paste::paste! { [<$descriptor:snake:upper _STATE>].init(::embassy_usb::class::hid::State::new()) };
        let request_handler = paste::paste! { [<$descriptor:snake:upper _HANDLER>].init($crate::usb::UsbRequestHandler {}) };

        let hid_config = ::embassy_usb::class::hid::Config {
            report_descriptor: <$descriptor>::desc(),
            request_handler: Some(request_handler),
            poll_ms: 1,
            max_packet_size: $max_packet,
            hid_subclass: $subclass,
            hid_boot_protocol: $protocol,
        };

        let rw: ::embassy_usb::class::hid::HidReaderWriter<_, $read_n, $write_n> = ::embassy_usb::class::hid::HidReaderWriter::new($usb_builder, state, hid_config);
        rw
    }};
}

#[cfg(feature = "usb_log")]
pub(crate) use add_usb_logger;
pub(crate) use add_usb_reader_writer;
pub(crate) use add_usb_writer;

pub(crate) struct UsbRequestHandler {}

impl RequestHandler for UsbRequestHandler {
    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {:?}", id, data);
        OutResponse::Accepted
    }
}

pub(crate) struct UsbDeviceHandler {
    /// State to restore on resume. Only a Configured device is ever published as
    /// Suspended (see `suspended()`), so this always holds Configured while the
    /// device is suspended; kept as a snapshot rather than a hardcoded value so
    /// resume stays correct if another pre-suspend state becomes publishable.
    pre_suspend: UsbState,
}

impl UsbDeviceHandler {
    fn new() -> Self {
        UsbDeviceHandler {
            pre_suspend: UsbState::Disabled,
        }
    }
}

impl Handler for UsbDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        if enabled {
            info!("Device enabled");
            set_usb_state_and_notify(UsbState::Enabled);
        } else {
            info!("Device disabled");
            set_usb_state_and_notify(UsbState::Disabled);
        }
    }

    fn reset(&mut self) {
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        if configured {
            set_usb_state_and_notify(UsbState::Configured);
            info!("Device configured, it may now draw up to the configured current from Vbus.")
        } else {
            set_usb_state_and_notify(UsbState::Enabled);
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }

    fn suspended(&mut self, suspended: bool) {
        if suspended {
            // Only publish Suspended when the device was configured before the
            // suspend. `usb_ready()` deliberately treats Suspended as routable
            // (a suspended host must stay reachable for remote wakeup), but that
            // only holds for a device the host has actually enumerated. A
            // never-configured device also sees bus-idle suspends — a charge-only
            // cable or wall charger leaves D+/D- idle, which e.g. on nRF52840
            // raises SUSPEND ~3 ms after enable — and publishing Suspended there
            // would route reports to endpoints that were never configured,
            // silently dropping keystrokes that BLE could have delivered.
            let live = current_usb_state();
            if live == UsbState::Configured {
                self.pre_suspend = live;
                set_usb_state_and_notify(UsbState::Suspended);
                info!(
                    "Device suspended, the Vbus current limit is 500µA (or 2.5mA for high-power devices with remote wakeup enabled)."
                );
            } else if live != UsbState::Suspended {
                info!("Bus suspended before enumeration (charger or charge-only cable?), USB stays inactive");
            }
        } else {
            // Only restore from Suspended; if we're somehow not in Suspended (out-of-order
            // callbacks), don't overwrite — `configured()`/`enabled()` will resync.
            if current_usb_state() == UsbState::Suspended {
                set_usb_state_and_notify(self.pre_suspend);
            }
            info!(
                "Device resumed, the Vbus current limit is 500µA (or 2.5mA for high-power devices with remote wakeup enabled)."
            );
        }
    }

    fn remote_wakeup_enabled(&mut self, enabled: bool) {
        info!("Remote wakeup enabled state: {}", enabled);
    }
}

// These tests mutate the process-global CONNECTION_STATUS; cargo-nextest's
// per-test process isolation keeps them from racing each other (plain
// `cargo test` is rejected at startup by `test_support::require_nextest`).
#[cfg(test)]
mod tests {
    use core::future::pending;

    use embassy_futures::join::join;
    use embassy_time::Timer;
    use embassy_usb::Handler;
    use rmk_types::connection::UsbState;

    use super::{
        USB_REMOTE_WAKEUP, USB_STATE_CHANGED, UsbDeviceHandler, set_usb_state_and_notify, usb_report_write_ready,
        write_while_usb_configured,
    };
    use crate::state::{current_usb_state, set_usb_state};
    use crate::test_support::test_block_on as block_on;

    fn reset_usb_signals() {
        USB_REMOTE_WAKEUP.reset();
        USB_STATE_CHANGED.reset();
    }

    #[test]
    fn configured_endpoint_is_immediately_write_ready() {
        reset_usb_signals();
        set_usb_state(UsbState::Configured);

        assert!(usb_report_write_ready());
        assert!(!USB_REMOTE_WAKEUP.signaled());
    }

    #[test]
    fn suspended_endpoint_requests_wakeup_and_drops_report() {
        reset_usb_signals();
        set_usb_state(UsbState::Suspended);

        assert!(!usb_report_write_ready());
        assert!(USB_REMOTE_WAKEUP.signaled());
    }

    #[test]
    fn inactive_endpoint_drops_report_without_requesting_wakeup() {
        for state in [UsbState::Disabled, UsbState::Enabled] {
            reset_usb_signals();
            set_usb_state(state);

            assert!(!usb_report_write_ready());
            assert!(!USB_REMOTE_WAKEUP.signaled());
        }
    }

    #[test]
    fn configured_write_completes_without_state_change() {
        reset_usb_signals();
        set_usb_state(UsbState::Configured);

        assert_eq!(block_on(write_while_usb_configured(async { 42 })), Some(42));
        assert!(!USB_REMOTE_WAKEUP.signaled());
    }

    #[test]
    fn suspend_cancels_write_already_waiting_for_host() {
        reset_usb_signals();
        set_usb_state(UsbState::Configured);

        let (write_result, ()) = block_on(join(write_while_usb_configured(pending::<()>()), async {
            Timer::after_millis(1).await;
            set_usb_state_and_notify(UsbState::Suspended);
        }));

        assert_eq!(write_result, None);
        assert!(USB_REMOTE_WAKEUP.signaled());
    }

    /// A charge-only cable / wall charger enables the device (VBUS present) but
    /// never enumerates it; the bus-idle suspend that follows must not publish
    /// Suspended, otherwise `usb_ready()` would route reports to endpoints that
    /// were never configured while a BLE host could have received them.
    #[test]
    fn suspend_without_enumeration_stays_enabled() {
        let mut handler = UsbDeviceHandler::new();
        handler.enabled(true);
        assert_eq!(current_usb_state(), UsbState::Enabled);

        handler.suspended(true);
        assert_eq!(current_usb_state(), UsbState::Enabled);

        // Spurious resume (bus activity without enumeration) changes nothing.
        handler.suspended(false);
        assert_eq!(current_usb_state(), UsbState::Enabled);

        // A host showing up later still enumerates normally.
        handler.configured(true);
        assert_eq!(current_usb_state(), UsbState::Configured);
    }

    /// A genuinely suspended (previously enumerated) host keeps the Suspended
    /// state so it stays routable for remote wakeup, and resume restores
    /// Configured.
    #[test]
    fn suspend_after_configured_publishes_suspended_and_resume_restores() {
        let mut handler = UsbDeviceHandler::new();
        handler.enabled(true);
        handler.configured(true);

        handler.suspended(true);
        assert_eq!(current_usb_state(), UsbState::Suspended);

        handler.suspended(false);
        assert_eq!(current_usb_state(), UsbState::Configured);
    }

    /// A stray duplicate `suspended(true)` while already Suspended must not
    /// clobber the pre-suspend snapshot that resume restores.
    #[test]
    fn duplicate_suspend_preserves_pre_suspend_state() {
        let mut handler = UsbDeviceHandler::new();
        handler.enabled(true);
        handler.configured(true);

        handler.suspended(true);
        handler.suspended(true);
        assert_eq!(current_usb_state(), UsbState::Suspended);

        handler.suspended(false);
        assert_eq!(current_usb_state(), UsbState::Configured);
    }

    /// Out-of-order resume while not suspended must not overwrite the live
    /// state.
    #[test]
    fn resume_without_suspend_is_a_no_op() {
        let mut handler = UsbDeviceHandler::new();
        set_usb_state(UsbState::Configured);

        handler.suspended(false);
        assert_eq!(current_usb_state(), UsbState::Configured);
    }
}
