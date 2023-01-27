#![no_std]
#![deny(warnings)]
#![no_main]

use adc_mcp3008::Mcp3008;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::RateExtU32;
use panic_halt as _;
use rp2040_hal::usb::UsbBus;
use rp_pico::entry;
use rp_pico::hal::{self, gpio::pin::bank0::Gpio16, pac, prelude::*};
use usb_device::{class_prelude::UsbBusAllocator, device::UsbVidPid, prelude::UsbDeviceBuilder};
use usbd_midi::{
    data::{
        byte::{from_traits::FromClamped, u7::U7},
        midi::{
            channel::Channel,
            message::{control_function::ControlFunction, Message},
            notes::Note,
        },
        usb::constants::USB_AUDIO_CLASS,
        usb_midi::{cable_number::CableNumber, usb_midi_event_packet::UsbMidiEventPacket},
    },
    midi_device::MidiClass,
};

type SPI = rp2040_hal::Spi<rp2040_hal::spi::Enabled, pac::SPI1, 8>;
type CS = rp2040_hal::gpio::Pin<Gpio16, rp2040_hal::gpio::Output<rp2040_hal::gpio::PushPull>>;

struct Controller {
    knob_values: [u8; 8],
    tap_detector_triggered: i32,
    tap_detector_average: u32,

    midi_channel: Channel,
}

impl Controller {
    fn new(midi_channel: Channel) -> Self {
        Controller {
            knob_values: [0; 8],
            tap_detector_triggered: 0,
            tap_detector_average: 0,
            midi_channel,
        }
    }

    fn process_knobs(&mut self, adc: &mut Mcp3008<SPI, CS>, midi: &mut MidiClass<UsbBus>) {
        for i in 1..4 {
            let channel = match i {
                0 => adc_mcp3008::Channels8::CH0,
                1 => adc_mcp3008::Channels8::CH1,
                2 => adc_mcp3008::Channels8::CH2,
                3 => adc_mcp3008::Channels8::CH3,
                4 => adc_mcp3008::Channels8::CH4,
                5 => adc_mcp3008::Channels8::CH5,
                6 => adc_mcp3008::Channels8::CH6,
                7 => adc_mcp3008::Channels8::CH7,
                _ => unreachable!(),
            };
            let r = adc.read_channel(channel).unwrap();
            let r1 = (r / 8) as u8;
            if r1 != self.knob_values[i] {
                self.knob_values[i] = r1;
                let midi_message = UsbMidiEventPacket::from_midi(
                    CableNumber::Cable0,
                    Message::ControlChange(
                        self.midi_channel,
                        ControlFunction(U7::from_clamped(i as u8 - 1)),
                        U7::from_clamped(r1),
                    ),
                );
                let _ = midi.send_message(midi_message);
            }
        }
    }

    fn process_tap_detector(&mut self, adc: &mut Mcp3008<SPI, CS>, midi: &mut MidiClass<UsbBus>) {
        let r = adc.read_channel(adc_mcp3008::Channels8::CH0).unwrap();
        self.tap_detector_average = (r as u32 / 10) + (self.tap_detector_average * 9) / 10;
        if self.tap_detector_triggered == 0
            && (r as i64 - self.tap_detector_average as i64).abs() > 15
        {
            let midi_message = UsbMidiEventPacket::from_midi(
                CableNumber::Cable0,
                Message::NoteOn(self.midi_channel, Note::C3, U7::from_clamped(127)),
            );
            let _ = midi.send_message(midi_message);
            let midi_message = UsbMidiEventPacket::from_midi(
                CableNumber::Cable0,
                Message::NoteOff(self.midi_channel, Note::C3, U7::from_clamped(0)),
            );
            let _ = midi.send_message(midi_message);
            self.tap_detector_triggered = 100;
        }
        self.tap_detector_triggered = (self.tap_detector_triggered - 1).max(0);
    }

    fn process(&mut self, adc: &mut Mcp3008<SPI, CS>, midi: &mut MidiClass<UsbBus>) {
        self.process_knobs(adc, midi);
        self.process_tap_detector(adc, midi);
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut midi = MidiClass::new(&usb_bus, 1, 0).unwrap();

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x5e4))
        .product("MIDI Test")
        .device_class(USB_AUDIO_CLASS)
        .build();

    let _spi_sclk = pins.gpio14.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio15.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let chip_enable = pins.gpio16.into_push_pull_output();

    let spi = hal::Spi::<_, _, 8>::new(pac.SPI1);
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1.MHz(),
        &adc_mcp3008::MODE,
    );
    let mut adc = adc_mcp3008::Mcp3008::new(spi, chip_enable).unwrap();

    let mut channel = 0;
    channel += pins.gpio6.into_pull_up_input().is_low().unwrap() as u8;
    channel += pins.gpio5.into_pull_up_input().is_low().unwrap() as u8 * 2;
    channel += pins.gpio4.into_pull_up_input().is_low().unwrap() as u8 * 4;
    channel += pins.gpio3.into_pull_up_input().is_low().unwrap() as u8 * 8;
    channel += pins.gpio2.into_pull_up_input().is_low().unwrap() as u8 * 16;
    channel += pins.gpio1.into_pull_up_input().is_low().unwrap() as u8 * 32;
    channel += pins.gpio0.into_pull_up_input().is_low().unwrap() as u8 * 64;
    let midi_channel = Channel::try_from(channel.min(15)).unwrap_or(Channel::Channel1);

    let mut controller = Controller::new(midi_channel);

    let mut led_pin = pins.led.into_push_pull_output();
    let _ = led_pin.set_high();

    loop {
        usb_dev.poll(&mut [&mut midi]);
        controller.process(&mut adc, &mut midi);
        delay.delay_ms(1);
    }
}
