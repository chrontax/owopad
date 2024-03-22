# OwOpad

Alternative firmware for the [minipad](https://github.com/minipadkb/minipad)

This firmware unlike the official doesn't bind keys 1 to 1, it uses key sequences for typing.

## Features:
* unlimited keys via sequences,
* rapid trigger,
* configuration via serial.

## Installation

Download a release or build:
```bash
cargo build --release
cd target/thumbv6m-none-eabi/release
elf2uf2-rs owopad
```

And flash:
```bash
picotool load owopad.uf2
picotool reboot
```

By default there a no defined keybinds

## Configuration

Until I write some utility you have to just send commands over serial.

### Serial Commands
All returned numbers are in little endian

#### `save`
Save current binds and key configuration to flash

#### `boot`
Puts the keypad into BOOTSEL mode

#### `echo<whatever>`
Sends back whatever you tell it to

#### `adc`
Sends last adc readings as [u16; 3]

#### `depth`
Sends last calculated key depth as [u16; 3]

#### `consts`
Sends firmware constants `[NODE_COUNT, SWITCH_TRAVEL, AUTOCALIBRATION_DEADZONE]` as [u16; 3]

#### `time`
Sends last main loop time as u64

#### `clear`
Clears all keybinds

#### `kreset`
Resets key configuration

#### `config`
Sends key configuration as [KeyConfig; 3] where KeyConfig:
```rust
struct KeyConfig {
    rt_up: u16,
    rt_down: u16,
    min: u16,
    max: u16,
}
```

#### `nodes`
Sends the table of nodes which is the tree representing keybinds as [Node; 1024] where Node:
```rust
struct Node {
    children: [u16; 3],
    key: Option<Keyboard>, // usb hid scancode
}
```

#### `key<i:u16>.<k:str>=<v:u16>`
Sets config property `k` of key of id `i` (zero-indexed) to `v`

#### `bind<seq:str>:<k:key>`
Sets sequence `seq` (keys, e.g. `LRM` for a seqence of: left key, right key, middle key) to correspond to key `k`. Besides the obvious characters `k` can be:
* `F<1..=12>` for function keys,
* `_RIGHT`, `_LEFT`, `_UP`, `_DOWN` for arrows,
* self explanatory: `_ENTER`, `_ESC`, `_TAB`, `_BACKSP`, `_CAPS`, `_PS`, `_INS`, `_HOME`, `_PGUP`, `_DEL`, `_END`, `_PGDOWN`, `_LCTRL`, `_LSHIFT`, `_LALT`, `_RCTRL`, `_RSHIFT`, `_RALT`.