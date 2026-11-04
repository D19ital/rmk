mod velvet_motion {
    use super::*;

    #[test]
    fn caret_overflow_full_width_quadrants_and_signed_limits() {
        for (x, y, key, residual) in [
            (20000, 20000, HidKeyCode::Right, (100, 0)),
            (20000, -20000, HidKeyCode::Right, (100, 0)),
            (-20000, 20000, HidKeyCode::Left, (-100, 0)),
            (-20000, -20000, HidKeyCode::Left, (-100, 0)),
            (32767, 32767, HidKeyCode::Right, (7267, 0)),
            (-32768, -32768, HidKeyCode::Left, (-7268, 0)),
            (32767, -32768, HidKeyCode::Up, (0, -7268)),
            (-32768, 32767, HidKeyCode::Left, (-7268, 0)),
            (-32768, 0, HidKeyCode::Left, (-7268, 0)),
            (0, -32768, HidKeyCode::Up, (0, -7268)),
            (0, 32767, HidKeyCode::Down, (0, 7267)),
        ] {
            let mut acc = MotionAccumulator::default();
            let count = if x == 20000 || x == -20000 { 199 } else { 255 };
            assert_eq!(
                compute_caret_taps(x, y, &mut acc, &CaretConfig::default()),
                Some((key, count)),
                "({x},{y})"
            );
            assert_eq!((acc.remainder_x, acc.remainder_y), residual);
        }
    }

    #[cfg(feature = "split")]
    #[test]
    fn caret_overflow_real_velvet_mode3_pipeline_releases_and_clears() {
        block_on(async {
            use crate::channel::{QueuedReportPayload, USB_REPORT_CHANNEL};
            use crate::event::{PeripheralSettingsEvent, PointingProcessorEvent};
            use crate::processor::Processor;
            use rmk_types::connection::UsbState;
            crate::state::set_usb_state(UsbState::Configured);
            USB_REPORT_CHANNEL.clear();
            let mut data = KeymapData::<1, 1, 1>::new([[[k!(A)]]]);
            let mut behavior = BehaviorConfig::default();
            let positional = PositionalConfig::<1, 1>::default();
            let keymap = KeyMap::new(&mut data, &mut behavior, &positional).await;
            let mut processor = PointingProcessor::new(&keymap, PointingProcessorConfig::default());
            let mut modes = PointingProcessorEvent::subscriber();
            let mut mode_owner = velvet_settings_test_source::VelvetPointingMode::new();
            let scenario = async {
                Timer::after_millis(1).await;
                // Dispatch the real User12 handler without allocating the host
                // build's zero-capacity ActionEvent subscription fixture.
                for pressed in [true, false] {
                    mode_owner
                        .process(
                            velvet_settings_test_source::VelvetPointingModeProcessorEventEnum::Action(
                                crate::event::ActionEvent {
                                    action: rmk_types::action::Action::User(12),
                                    keyboard_event: crate::event::KeyboardEvent::key(0, 0, pressed),
                                },
                            ),
                        )
                        .await;
                    let event = modes.next_message_pure().await;
                    assert_eq!(matches!(event.mode, PointingMode::Caret(_)), pressed);
                }
                let mut packet = velvet_settings_test_source::default_settings_packet();
                packet[1] = 3;
                mode_owner
                    .process(
                        velvet_settings_test_source::VelvetPointingModeProcessorEventEnum::PeripheralSettings(
                            PeripheralSettingsEvent(packet),
                        ),
                    )
                    .await;
                let event = modes.next_message_pure().await;
                assert!(matches!(event.mode, PointingMode::Caret(_)));
                processor.on_pointing_processor_event(event).await;
                for (x, y, key) in [
                    (20000, 20000, HidKeyCode::Right),
                    (-32768, 32767, HidKeyCode::Left),
                    (32767, -32768, HidKeyCode::Up),
                ] {
                    processor.accumulator.reset();
                    let start = Instant::now();
                    let motion = processor.on_pointing_event(device(x, y).take_report_event().unwrap());
                    let collect = async {
                        for _ in 0..255 {
                            for pressed in [true, false] {
                                let QueuedReportPayload::Hid(Report::KeyboardReport(report)) =
                                    USB_REPORT_CHANNEL.receive().await.into_payload()
                                else {
                                    panic!("caret must emit keyboard report")
                                };
                                assert_eq!(
                                    report.keycodes,
                                    if pressed { [key as u8, 0, 0, 0, 0, 0] } else { [0; 6] }
                                );
                                assert_eq!(report.modifier, 0);
                            }
                        }
                    };
                    embassy_futures::join::join(motion, collect).await;
                    assert_eq!(
                        start.elapsed().as_millis(),
                        2550,
                        "retain both 5ms tap gaps and 255 cap"
                    );
                    assert!(USB_REPORT_CHANNEL.is_empty());
                    processor.set_pointing_mode(PointingMode::Disabled);
                    assert_eq!(
                        (processor.accumulator.remainder_x, processor.accumulator.remainder_y),
                        (0, 0)
                    );
                    processor.on_pointing_processor_event(event).await;
                    processor
                        .on_pointing_event(device(1, 0).take_report_event().unwrap())
                        .await;
                    assert!(USB_REPORT_CHANNEL.is_empty(), "mode clear must not replay cap residual");
                }
            };
            scenario.await;
        });
    }

    fn device(x: i32, y: i32) -> PointingDevice<DummyDriver> {
        PointingDevice {
            sensor: DummyDriver {
                motion_pending: false,
                motion: MotionData { dx: 0, dy: 0 },
                init_called: true,
                fails_init: false,
                motion_gpio: None,
                read_called: false,
            },
            init_state: InitState::Ready,
            poll_interval: Duration::from_micros(500),
            report_interval: Duration::from_hz(125),
            last_poll: Instant::MIN,
            last_report: Instant::MIN,
            accumulated_x: x,
            accumulated_y: y,
            id: 0,
        }
    }

    #[cfg(feature = "split")]
    #[test]
    fn velvet_actual_settings_dpi_table_and_storage_packet_contract_are_unchanged() {
        use velvet_settings_test_source::{cpi_from_packet, default_settings_packet, sanitize_settings_packet};
        for (index, expected) in [
            200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200,
        ]
        .into_iter()
        .enumerate()
        {
            let mut packet = default_settings_packet();
            packet[3] = index as u8;
            packet[7] = 0x3f;
            let restored = sanitize_settings_packet(&packet);
            assert_eq!(restored, packet);
            assert_eq!(cpi_from_packet(&restored), Some(expected));
        }
        assert_eq!(cpi_from_packet(&default_settings_packet()), Some(1000));
    }

    #[test]
    fn healthy_polling_delivers_125_full_events_per_second_without_tail() {
        block_on(async {
            let mut sensor = device(0, 0);
            sensor.sensor.motion_pending = true;
            sensor.sensor.motion = MotionData { dx: 20, dy: -5 };
            // Start after the mock clock's zero/MIN sentinel.
            Timer::after_millis(1).await;
            let start = Instant::now();
            for _ in 0..125 {
                let event = sensor.read_pointing_event().await;
                assert!(event.axes[0].value > 127, "do not cap each 8 ms event at HID width");
                assert_eq!((sensor.accumulated_x, sensor.accumulated_y), (0, 0));
            }
            let elapsed = start.elapsed().as_millis();
            assert!((1000..=1002).contains(&elapsed), "125 Hz changed: {elapsed} ms");
            sensor.sensor.motion_pending = false;
            assert!(sensor.take_report_event().is_none());
        });
    }

    #[test]
    fn processor_queue_backpressure_retains_exact_event() {
        block_on(async {
            use crate::channel::{QueuedReport, QueuedReportPayload, USB_REPORT_CHANNEL};
            use core::future::Future;
            use core::task::{Context, Poll, Waker};
            use rmk_types::connection::UsbState;
            crate::state::set_usb_state(UsbState::Configured);
            USB_REPORT_CHANNEL.clear();
            let mut data = KeymapData::<1, 1, 1>::new([[[k!(A)]]]);
            let mut behavior = BehaviorConfig::default();
            let positional = PositionalConfig::<1, 1>::default();
            let keymap = KeyMap::new(&mut data, &mut behavior, &positional).await;
            let mut processor = PointingProcessor::new(&keymap, PointingProcessorConfig::default());
            for _ in 0..crate::REPORT_CHANNEL_SIZE {
                USB_REPORT_CHANNEL
                    .try_send(QueuedReport::new(Report::KeyboardReport(KeyboardReport::default())))
                    .unwrap();
            }
            let mut pending =
                core::pin::pin!(processor.on_pointing_event(device(-300, 90).take_report_event().unwrap()));
            let mut cx = Context::from_waker(Waker::noop());
            assert!(matches!(pending.as_mut().poll(&mut cx), Poll::Pending));
            for _ in 0..crate::REPORT_CHANNEL_SIZE {
                assert!(matches!(
                    USB_REPORT_CHANNEL.try_receive().unwrap().into_payload(),
                    QueuedReportPayload::Hid(Report::KeyboardReport(_))
                ));
            }
            assert!(matches!(pending.as_mut().poll(&mut cx), Poll::Ready(())));
            let QueuedReportPayload::WideMouse(report) = USB_REPORT_CHANNEL.try_receive().unwrap().into_payload()
            else {
                panic!("lost pending motion");
            };
            assert_eq!((report.x, report.y), (-300, 90));
            assert!(USB_REPORT_CHANNEL.is_empty());
        });
    }

    fn drain_chunks(mut report: crate::channel::WideMouseReport) -> (i32, i32, i32, i32) {
        let mut sum = (0, 0, 0, 0);
        let mut count = 0;
        loop {
            let (x, y, wheel, pan) =
                crate::mouse_chunk::take_vector_chunk(&mut report.x, &mut report.y, &mut report.wheel, &mut report.pan);
            sum.0 += i32::from(x);
            sum.1 += i32::from(y);
            sum.2 += i32::from(wheel);
            sum.3 += i32::from(pan);
            count += 1;
            assert!(count <= 259, "residual failed to drain");
            if (report.x, report.y, report.wheel, report.pan) == (0, 0, 0, 0) {
                break;
            }
        }
        sum
    }

    #[test]
    fn real_processor_modes_preserve_positive_negative_fast_motion_and_buttons() {
        block_on(async {
            use crate::channel::{QueuedReportPayload, USB_REPORT_CHANNEL};
            use rmk_types::connection::UsbState;
            crate::state::set_usb_state(UsbState::Configured);
            USB_REPORT_CHANNEL.clear();
            let mut data = KeymapData::<1, 1, 1>::new([[[k!(A)]]]);
            let mut behavior = BehaviorConfig::default();
            let positional = PositionalConfig::<1, 1>::default();
            let keymap = KeyMap::new(&mut data, &mut behavior, &positional).await;
            let mut processor = PointingProcessor::new(&keymap, PointingProcessorConfig::default());
            for buttons in [0, 1, 3, 0] {
                keymap.set_mouse_buttons(buttons);
                for (x, y) in [(300, 88), (300, -88), (-300, 88), (-300, -88), (32767, -32768)] {
                    for (mode, expected) in [
                        (PointingMode::Cursor(CursorConfig::default()), (x, y, 0, 0)),
                        (PointingMode::Sniper(SniperConfig::default()), (x / 4, y / 4, 0, 0)),
                        (PointingMode::Scroll(ScrollConfig::default()), (0, 0, -(y / 8), x / 8)),
                    ] {
                        processor.set_pointing_mode(PointingMode::Disabled);
                        processor.set_pointing_mode(mode);
                        processor
                            .on_pointing_event(device(x, y).take_report_event().unwrap())
                            .await;
                        assert_eq!(USB_REPORT_CHANNEL.len(), 1);
                        let QueuedReportPayload::WideMouse(report) =
                            USB_REPORT_CHANNEL.try_receive().unwrap().into_payload()
                        else {
                            panic!("premature i8 narrowing");
                        };
                        assert_eq!(report.buttons, buttons);
                        assert_eq!(drain_chunks(report), expected);
                        assert!(USB_REPORT_CHANNEL.is_empty());
                    }
                }
            }
            processor.set_pointing_mode(PointingMode::Disabled);
            processor
                .on_pointing_event(device(300, -90).take_report_event().unwrap())
                .await;
            assert!(USB_REPORT_CHANNEL.is_empty());
        });
    }

    #[test]
    fn sensor_runner_backpressures_instead_of_overwriting_relative_events() {
        use core::future::Future;
        use core::task::{Context, Poll, Waker};
        use embassy_sync::pubsub::WaitResult;
        use embassy_time::MockDriver;
        MockDriver::get().reset();
        let mut subscriber = PointingEvent::subscriber();
        for i in 0..crate::POINTING_EVENT_CHANNEL_SIZE {
            publish_event(device((i + 1) as i32, 0).take_report_event().unwrap());
        }
        let mut sensor = device(300, -90);
        let mut runner = core::pin::pin!(sensor.run());
        let mut cx = Context::from_waker(Waker::noop());
        assert!(matches!(runner.as_mut().poll(&mut cx), Poll::Pending));
        MockDriver::get().advance(Duration::from_millis(8));
        assert!(matches!(runner.as_mut().poll(&mut cx), Poll::Pending));
        for i in 0..crate::POINTING_EVENT_CHANNEL_SIZE {
            let Some(WaitResult::Message(event)) = subscriber.try_next_message() else {
                panic!("full event channel lost a relative motion sample instead of backpressuring");
            };
            assert_eq!(event.axes[0].value, (i + 1) as i16);
        }
        // Releasing capacity lets the exact pending full-width event through.
        assert!(matches!(runner.as_mut().poll(&mut cx), Poll::Pending));
        let Some(WaitResult::Message(event)) = subscriber.try_next_message() else {
            panic!("pending motion disappeared");
        };
        assert_eq!((event.axes[0].value, event.axes[1].value), (300, -90));
        assert!(subscriber.try_next_message().is_none());
    }

    #[test]
    fn full_width_negative_limit_is_safe_when_inverted() {
        block_on(async {
            use crate::channel::USB_REPORT_CHANNEL;
            use rmk_types::connection::UsbState;
            crate::state::set_usb_state(UsbState::Configured);
            USB_REPORT_CHANNEL.clear();
            let mut data = KeymapData::<1, 1, 1>::new([[[k!(A)]]]);
            let mut behavior = BehaviorConfig::default();
            let positional = PositionalConfig::<1, 1>::default();
            let keymap = KeyMap::new(&mut data, &mut behavior, &positional).await;
            let mut processor = PointingProcessor::new(
                &keymap,
                PointingProcessorConfig {
                    invert_x: true,
                    ..Default::default()
                },
            );
            processor
                .on_pointing_event(device(-32768, 0).take_report_event().unwrap())
                .await;
            let crate::channel::QueuedReportPayload::WideMouse(report) =
                USB_REPORT_CHANNEL.try_receive().unwrap().into_payload()
            else {
                panic!("native width lost");
            };
            // Positive 32768 is not representable in the existing i16 contract.
            assert_eq!(report.x, 32767);
        });
    }

    #[test]
    fn processor_keeps_native_vector_and_buttons_in_one_queue_item() {
        block_on(async {
            use crate::channel::{QueuedReportPayload, USB_REPORT_CHANNEL};
            use rmk_types::connection::UsbState;
            crate::state::set_usb_state(UsbState::Configured);
            USB_REPORT_CHANNEL.clear();
            let mut data = KeymapData::<1, 1, 1>::new([[[k!(A)]]]);
            let mut behavior = BehaviorConfig::default();
            let positional = PositionalConfig::<1, 1>::default();
            let keymap = KeyMap::new(&mut data, &mut behavior, &positional).await;
            keymap.set_mouse_buttons(3);
            let mut processor = PointingProcessor::new(&keymap, PointingProcessorConfig::default());
            let event = device(300, -90).take_report_event().unwrap();
            processor.on_pointing_event(event).await;
            assert_eq!(USB_REPORT_CHANNEL.len(), 1, "one event, not pre-expanded HID frames");
            let queued = USB_REPORT_CHANNEL.try_receive().unwrap();
            let QueuedReportPayload::WideMouse(report) = queued.into_payload() else {
                panic!("PointingProcessor prematurely narrowed native motion into lossy i8 HID");
            };
            assert_eq!(
                (report.buttons, report.x, report.y, report.wheel, report.pan),
                (3, 300, -90, 0, 0)
            );
        });
    }

    #[test]
    fn full_i16_event_does_not_leave_a_125hz_sensor_tail() {
        for (x, y) in [(300, -300), (-300, 300), (2047, -2048), (32767, -32768), (5, -2)] {
            let mut sensor = device(x, y);
            let event = sensor.take_report_event().unwrap();
            assert_eq!(
                (i32::from(event.axes[0].value), i32::from(event.axes[1].value)),
                (x, y),
                "native motion must leave the sensor in ONE event, not at most 127/-128 counts per 8 ms"
            );
            assert_eq!((sensor.accumulated_x, sensor.accumulated_y), (0, 0));
            assert!(
                sensor.take_report_event().is_none(),
                "no residual cursor slip after stopping"
            );
        }
    }
}
