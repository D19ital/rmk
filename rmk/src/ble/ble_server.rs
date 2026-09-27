use trouble_host::prelude::*;
use usbd_hid::descriptor::AsInputReport;
#[cfg(not(feature = "host"))]
use usbd_hid::descriptor::SerializedDescriptor;

use super::battery_service::BatteryService;
use super::device_info::DeviceConfigurationService;
#[cfg(not(feature = "host"))]
use crate::hid::{BLE_COMPOSITE_REPORT_MAP_LEN, BleCompositeReport};
#[cfg(feature = "host")]
use crate::hid::{BLE_REPORT_MAP_LEN, ble_report_map};
use crate::hid::{CompositeReportType, HidError, HidWriterTrait, Report};

#[cfg(feature = "mouse_ble_16bit_report")]
const BLE_MOUSE_REPORT_LEN: usize = 7;
#[cfg(not(feature = "mouse_ble_16bit_report"))]
const BLE_MOUSE_REPORT_LEN: usize = 5;

// Used for saving the client attribute (CCCD) table. Tracks the trouble-host
// per-connection client-specific attribute buffer size.
pub(crate) const CCCD_TABLE_SIZE: usize = trouble_host::config::CLIENT_ATT_TABLE_SIZE;

// `gatt_server` expands every field regardless of a field-level `cfg`, so the
// host and non-host layouts are defined separately. Keep the vendor service
// last: existing HID and Device Information handles remain stable.
#[cfg(feature = "host")]
#[gatt_server]
pub(crate) struct Server {
    pub(crate) battery_service: BatteryService,
    pub(crate) hid_service: HidService,
    pub(crate) device_config_service: DeviceConfigurationService,
    pub(crate) vial_gatt_service: VialGattService,
}

#[cfg(not(feature = "host"))]
#[gatt_server]
pub(crate) struct Server {
    pub(crate) battery_service: BatteryService,
    pub(crate) hid_service: HidService,
    pub(crate) device_config_service: DeviceConfigurationService,
}

/// BlueZ owns HID-over-GATT services and rejects application writes to them.
/// This parallel vendor service carries the same 32-byte Vial packets without
/// changing the standard HOGP reports used by keyboards and other Vial hosts.
///
/// UUIDs are UUIDv5(DNS) values derived from:
/// - `vial-gatt.ergohaven.xyz`
/// - `vial-gatt-input.ergohaven.xyz`
/// - `vial-gatt-output.ergohaven.xyz`
#[cfg(feature = "host")]
#[gatt_service(uuid = "8cfa65ff-3b6d-55f3-8b67-49693930420d")]
pub(crate) struct VialGattService {
    #[characteristic(uuid = "7a115e75-ae8e-51b4-9f46-dbd15af07dc3", read, notify, permissions(encrypted))]
    pub(crate) input: [u8; 32],
    #[characteristic(
        uuid = "70ca58d5-fdbf-5497-a33f-d1e8e1698678",
        write,
        write_without_response,
        permissions(encrypted)
    )]
    pub(crate) output: [u8; 32],
}

/// The single HID service carrying all reports, distinguished by report id via
/// each characteristic's Report Reference descriptor.
///
/// Platform HID hosts are not consistent when a peripheral exposes multiple
/// HOGP service instances, so Vial must live in this same service alongside the
/// keyboard, mouse, media, and system reports.
#[cfg(feature = "host")]
#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct HidService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = ble_report_map())]
    pub(crate) report_map: [u8; BLE_REPORT_MAP_LEN],
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Keyboard as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) input_keyboard: [u8; 8],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Keyboard as u8, 2u8])]
    #[characteristic(uuid = "2a4d", read, write, write_without_response)]
    pub(crate) output_keyboard: [u8; 1],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Mouse as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) mouse_report: [u8; BLE_MOUSE_REPORT_LEN],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Media as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) media_report: [u8; 2],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::System as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) system_report: [u8; 1],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Vial as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) vial_input: [u8; 32],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Vial as u8, 2u8])]
    #[characteristic(uuid = "2a4d", read, write, write_without_response)]
    pub(crate) vial_output: [u8; 32],
}

/// The single HID service carrying all reports, distinguished by report id via
/// each characteristic's Report Reference descriptor. Android's HID host only
/// attaches to the first HID service instance, so the reports must not be
/// spread over multiple service instances.
#[cfg(not(feature = "host"))]
#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct HidService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = BleCompositeReport::desc().try_into().expect("Failed to convert BleCompositeReport to the configured BLE report-map length"))]
    pub(crate) report_map: [u8; BLE_COMPOSITE_REPORT_MAP_LEN],
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Keyboard as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) input_keyboard: [u8; 8],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Keyboard as u8, 2u8])]
    #[characteristic(uuid = "2a4d", read, write, write_without_response)]
    pub(crate) output_keyboard: [u8; 1],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Mouse as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) mouse_report: [u8; BLE_MOUSE_REPORT_LEN],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Media as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) media_report: [u8; 2],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::System as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) system_report: [u8; 1],
}

pub(crate) struct BleHidServer<'stack, 'server, 'conn, P: PacketPool> {
    input_keyboard: Characteristic<[u8; 8]>,
    mouse_report: Characteristic<[u8; BLE_MOUSE_REPORT_LEN]>,
    media_report: Characteristic<[u8; 2]>,
    system_report: Characteristic<[u8; 1]>,
    conn: &'conn GattConnection<'stack, 'server, P>,
}

impl<'stack, 'server, 'conn, P: PacketPool> BleHidServer<'stack, 'server, 'conn, P> {
    pub(crate) fn new(server: &Server, conn: &'conn GattConnection<'stack, 'server, P>) -> Self {
        Self {
            input_keyboard: server.hid_service.input_keyboard,
            mouse_report: server.hid_service.mouse_report,
            media_report: server.hid_service.media_report,
            system_report: server.hid_service.system_report,
            conn,
        }
    }

    async fn notify_report<R: AsInputReport, const N: usize>(
        &self,
        characteristic: Characteristic<[u8; N]>,
        report: &R,
    ) -> Result<usize, HidError> {
        let mut buf = [0u8; N];
        let n = report.serialize(&mut buf).map_err(|_| HidError::ReportSerializeError)?;
        characteristic.notify(self.conn, &buf, true).await.map_err(|e| {
            error!("Failed to notify HID report: {:?}", e);
            HidError::BleError
        })?;
        Ok(n)
    }
}

impl<P: PacketPool> HidWriterTrait for BleHidServer<'_, '_, '_, P> {
    type ReportType = Report;

    async fn write_report(&mut self, report: &Self::ReportType) -> Result<usize, HidError> {
        match report {
            Report::KeyboardReport(r) => self.notify_report(self.input_keyboard, r).await,
            Report::MouseReport(r) => self.notify_report(self.mouse_report, r).await,
            Report::MediaKeyboardReport(r) => self.notify_report(self.media_report, r).await,
            Report::SystemControlReport(r) => self.notify_report(self.system_report, r).await,
            // Plover HID over BLE is not supported: the stock HID-over-GATT service
            // has no stenography characteristic. Drop silently at the writer.
            #[cfg(feature = "steno")]
            Report::StenoReport(_) => {
                debug!("Steno chord dropped: Plover HID over BLE is not supported");
                Ok(0)
            }
        }
    }

    #[cfg(feature = "mouse_ble_16bit_report")]
    async fn write_ble_mouse16_report(&mut self, report: &crate::hid::BleMouse16Report) -> Result<usize, HidError> {
        self.notify_report(self.mouse_report, report).await
    }
}
