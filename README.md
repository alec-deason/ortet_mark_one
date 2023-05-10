This is the electronics design and firmware for the Ortet Mark One MIDI controller.

## How to flash the firmware

In order to flash the firmware on the the controller's RP Pico you'll need to setup your rust environment by installing the `thumbv6m-none-eabi` target:
```
rustup target add thumbv6m-none-eabi
```

Then you will need `elf2uf2-rs` for building the UF2:
```
cargo install elf2uf2-rs
```

Then put the Pico into bootloader mode by holding the BOOTSEL button on the board while plugging it in to your computer. The board should mount as a USB drive at which point you can run:
```
cargo run --release
```

The project should build and install itself onto the Pico, then the Pico should reboot and present as a USB MIDI device named `Ortet MIDI Controller`.

For more information see the rp_pico documentation: https://github.com/rp-rs/rp-hal-boards/tree/main/boards/rp-pico

## License

This work is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
