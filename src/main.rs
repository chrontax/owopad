#![no_std]
#![no_main]

const SWITCH_TRAVEL: u16 = 400; // SWITCH_TRAVEL * 0.01mm
const AUTOCALIBRATION_DEADZONE: u16 = 10; // raw input
const NODE_TABLE_SIZE: usize = 2; // NODE_TABLE_SIZE * 4096B

use core::{
    array::from_fn,
    mem::{size_of, transmute},
    ptr::read_volatile,
    slice,
    str::from_utf8,
};

use rp2040_hal as hal;
use rp2040_panic_usb_boot as _;

use cortex_m::prelude::{_embedded_hal_adc_OneShot, _embedded_hal_timer_CountDown};
use fugit::ExtU32;
use hal::{
    adc::AdcPin,
    clocks::init_clocks_and_plls,
    entry,
    gpio::{
        self,
        bank0::{Gpio26, Gpio27, Gpio28},
        FunctionNull, Pins, PullDown,
    },
    pac,
    rom_data::reset_to_usb_boot,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
    Adc, Timer,
};
use rp2040_flash::flash::flash_range_erase_and_program;
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_human_interface_device::{
    device::keyboard::NKROBootKeyboardConfig, page::Keyboard, usb_class::UsbHidClassBuilder,
    UsbHidError,
};
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ: u32 = 12_000_000;

const NODE_COUNT: usize = (NODE_TABLE_SIZE * 4096) / size_of::<Node>();

type Pin<P> = AdcPin<gpio::Pin<P, FunctionNull, PullDown>>;
type AdcPins = (Adc, Pin<Gpio26>, Pin<Gpio27>, Pin<Gpio28>);

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let mut nodes = [Node::new(None); NODE_COUNT];
    let mut i = 0;
    let mut configs = [KeyConfig::new(); 3];

    unsafe {
        let init_flag = read_volatile((0x10100000 - 4096) as *const [u8; 4096]);
        if init_flag != PAN_TADEUSZ {
            save(nodes, configs);
            let mut tad = [0; 4096];
            tad.copy_from_slice(PAN_TADEUSZ);
            write(0x100000 - 4096, &tad);
        } else {
            nodes.copy_from_slice(&read_volatile(0x10100000 as *const [Node; NODE_COUNT]));
            configs.copy_from_slice(&read_volatile(
                (0x10100000 + NODE_TABLE_SIZE * 4096) as *const [KeyConfig; 3],
            ));
        }
    }

    let mut len = nodes[1..]
        .iter()
        .position(|n| *n == Node::new(None))
        .unwrap_or(NODE_COUNT);

    let mut keys: [Key; 3] = configs.map(|c| Key::new(c));

    let clocks = init_clocks_and_plls(
        XTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let mut keyboard = UsbHidClassBuilder::new()
        .add_device(NKROBootKeyboardConfig::default())
        .build(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x5566, 0x0001)).build();

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut pins = (
        Adc::new(pac.ADC, &mut pac.RESETS),
        AdcPin::new(pins.gpio26),
        AdcPin::new(pins.gpio27),
        AdcPin::new(pins.gpio28),
    );

    let mut kb_timer = timer.count_down();
    kb_timer.start(1.millis());

    let mut last_time = 0;
    let mut depth = [400; 3];

    loop {
        let start = timer.get_counter();

        let adc = read_adc(&mut pins);

        if kb_timer.wait().is_ok() {
            let keys = get_keys(adc, &nodes, &mut i, &mut keys);
            depth = keys.1;
            let keys = keys.0;

            match keyboard.device().write_report(keys) {
                Ok(_) | Err(UsbHidError::Duplicate | UsbHidError::WouldBlock) => {}
                Err(e) => panic!("failed to write kb report: {:?}", e),
            }

            match keyboard.tick() {
                Ok(_) | Err(UsbHidError::WouldBlock) => {}
                Err(e) => panic!("kb tick failed: {:?}", e),
            }
        }

        if !usb_dev.poll(&mut [&mut serial, &mut keyboard]) {
            last_time = (timer.get_counter() - start).to_micros();
            continue;
        }

        let mut buf = [0; 64];

        match serial.read(&mut buf) {
            Ok(count) => match &buf[..count] {
                b"boot" => reset_to_usb_boot(0, 0),
                b"adc" => send_serial(&mut serial, adc),
                b"depth" => send_serial(&mut serial, depth),
                b"save" => save(nodes, keys.map(|k| k.config)),
                b"time" => send_serial(&mut serial, last_time.to_le_bytes()),
                b"clear" => {
                    nodes = [Node::new(None); NODE_COUNT];
                    len = 1;
                    i = 0;
                }
                b"kreset" => {
                    for k in keys.iter_mut() {
                        k.config = KeyConfig::new();
                    }
                }
                b"config" => send_serial(&mut serial, keys.map(|k| k.config)),
                b"nodes" => send_serial(&mut serial, nodes),
                other if other.starts_with(b"key") => {
                    let mut iter = other
                        [3..other.iter().position(|c| *c == 0).unwrap_or(other.len())]
                        .split(|c| *c == b'.');
                    let i: usize = from_utf8(iter.next().unwrap()).unwrap().parse().unwrap();
                    let mut iter = iter.next().unwrap().split(|c| *c == b'=');
                    let setting = iter.next().unwrap();
                    let value = from_utf8(iter.next().unwrap()).unwrap().parse().unwrap();

                    match setting {
                        b"min" => keys[i].config.min = value,
                        b"max" => keys[i].config.max = value,
                        b"rt_up" => keys[i].config.rt_up = value,
                        b"rt_down" => keys[i].config.rt_down = value,
                        other => panic!("unknown setting: {}", from_utf8(other).unwrap()),
                    }
                }
                other if other.starts_with(b"bind") => {
                    let mut iter = other
                        [4..other.iter().position(|c| *c == 0).unwrap_or(other.len())]
                        .split(|c| *c == b':');
                    let path = iter.next().unwrap();
                    let char = to_key(iter.next().unwrap());
                    insert(&mut nodes, &mut len, path, char);
                }
                other if other.starts_with(b"echo") => {
                    let mut left = 60;
                    while left != 0 {
                        left -= serial.write(&buf[64 - left..]).unwrap()
                    }
                }
                other => panic!("unknown command: {}", from_utf8(other).unwrap()),
            },
            Err(UsbError::WouldBlock) => {}
            Err(e) => panic!("failed to read serial: {:?}", e),
        }

        last_time = (timer.get_counter() - start).to_micros();
    }
}

fn to_key(c: &[u8]) -> Keyboard {
    match c[0] {
        b'F' => {
            if c.len() == 1 {
                Keyboard::F
            } else {
                match &c[1..] {
                    b"10" => Keyboard::F10,
                    b"11" => Keyboard::F11,
                    b"12" => Keyboard::F12,
                    _ => Keyboard::from(c[1] + 9),
                }
            }
        }
        b'a'..=b'z' => Keyboard::from(c[0] - 93),
        b'A'..=b'Z' => Keyboard::from(c[0] - 61),
        b'1'..=b'9' => Keyboard::from(c[0] - 19),
        b'0' => Keyboard::Keyboard0,
        b'-' => Keyboard::Minus,
        b'=' => Keyboard::Equal,
        b'[' => Keyboard::LeftBrace,
        b']' => Keyboard::RightBrace,
        b'\\' => Keyboard::Backslash,
        b' ' => Keyboard::Space,
        b';' => Keyboard::Semicolon,
        b'\'' => Keyboard::Apostrophe,
        b'`' => Keyboard::Grave,
        b',' => Keyboard::Comma,
        b'.' => Keyboard::Dot,
        b'/' => Keyboard::ForwardSlash,
        _ => match c {
            b"_ENTER" => Keyboard::ReturnEnter,
            b"_ESC" => Keyboard::Escape,
            b"_TAB" => Keyboard::Tab,
            b"_BACKSP" => Keyboard::DeleteBackspace,
            b"_CAPS" => Keyboard::CapsLock,
            b"_PS" => Keyboard::PrintScreen,
            b"_INS" => Keyboard::Insert,
            b"_HOME" => Keyboard::Home,
            b"_PGUP" => Keyboard::PageUp,
            b"_DEL" => Keyboard::DeleteForward,
            b"_END" => Keyboard::End,
            b"_PGDOWN" => Keyboard::PageDown,
            b"_RIGHT" => Keyboard::RightArrow,
            b"_LEFT" => Keyboard::LeftArrow,
            b"_DOWN" => Keyboard::DownArrow,
            b"_UP" => Keyboard::UpArrow,
            b"_LCTRL" => Keyboard::LeftControl,
            b"_LSHIFT" => Keyboard::LeftShift,
            b"_LALT" => Keyboard::LeftAlt,
            b"_RCTRL" => Keyboard::RightControl,
            b"_RSHIFT" => Keyboard::RightShift,
            b"_RALT" => Keyboard::RightAlt,
            _ => panic!("unknown key: {}", from_utf8(c).unwrap()),
        },
    }
}

fn read_adc(pins: &mut AdcPins) -> [u16; 3] {
    let adc = &mut pins.0;
    [
        adc.read(&mut pins.3).unwrap(),
        adc.read(&mut pins.2).unwrap(),
        adc.read(&mut pins.1).unwrap(),
    ]
}

fn send_serial<T>(serial: &mut SerialPort<UsbBus>, ins: T) {
    let slice = unsafe { slice::from_raw_parts(transmute(&ins), size_of::<T>()) };
    let mut written = 0;
    while written != slice.len() {
        written += match serial.write(&slice[written..]) {
            Ok(n) => n,
            Err(UsbError::WouldBlock) => 0,
            Err(e) => panic!("failed to write serial: {:?}", e),
        }
    }
}

// for some reason I need another function
// when I tried to just put that line directly it would just hang
unsafe fn write(addr: u32, data: &[u8]) {
    cortex_m::interrupt::free(|_cs| flash_range_erase_and_program(addr, data, true));
}

fn save(nodes: [Node; NODE_COUNT], configs: [KeyConfig; 3]) {
    unsafe {
        write(
            0x100000,
            slice::from_raw_parts(transmute(&nodes), NODE_TABLE_SIZE),
        );
        write(
            0x100000 + NODE_TABLE_SIZE as u32 * 4096,
            slice::from_raw_parts(transmute(&configs), 4096),
        );
    }
}

fn get_keys(
    ins: [u16; 3],
    nodes: &[Node],
    i: &mut usize,
    keys: &mut [Key],
) -> ([Keyboard; 3], [u16; 3]) {
    let mut depth = [0; 3];
    (
        from_fn(|j| {
            let key = &mut keys[j];
            depth[j] = key.process(ins[j]);
            if let Some(key) = key.hold {
                key
            } else if key.just_pressed {
                *i = nodes[*i].children[j] as usize;
                if nodes[*i] == Node::new(None) {
                    *i = 0
                }
                if let Some(key) = nodes[*i].key {
                    *i = 0;
                    keys[j].hold = Some(key);
                    key
                } else {
                    Keyboard::NoEventIndicated
                }
            } else {
                Keyboard::NoEventIndicated
            }
        }),
        depth,
    )
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct Node {
    children: [u16; 3],
    key: Option<Keyboard>,
}

impl Node {
    const fn new(key: Option<Keyboard>) -> Self {
        Self {
            children: [0; 3],
            key,
        }
    }
}

fn insert(nodes: &mut [Node], len: &mut usize, path: &[u8], value: Keyboard) {
    let mut i = 0;
    for direction in path {
        nodes[i].key = None;
        i = match direction {
            b'L' => {
                if nodes[i].children[0] == 0 {
                    nodes[*len] = Node::new(None);
                    nodes[i].children[0] = *len as u16;
                    *len += 1;
                }
                nodes[i].children[0]
            }
            b'M' => {
                if nodes[i].children[1] == 0 {
                    nodes[*len] = Node::new(None);
                    nodes[i].children[1] = *len as u16;
                    *len += 1;
                }
                nodes[i].children[1]
            }
            b'R' => {
                if nodes[i].children[2] == 0 {
                    nodes[*len] = Node::new(None);
                    nodes[i].children[2] = *len as u16;
                    *len += 1;
                }
                nodes[i].children[2]
            }
            other => panic!("unknown key: {}", *other as char),
        } as usize;
    }
    nodes[i].key = Some(value);
}

#[derive(Clone, Copy)]
struct KeyConfig {
    rt_up: u16,
    rt_down: u16,
    min: u16,
    max: u16,
}

impl KeyConfig {
    const fn new() -> Self {
        Self {
            rt_up: 40,
            rt_down: 20,
            min: u16::MAX,
            max: u16::MIN,
        }
    }
}

impl Into<Key> for KeyConfig {
    fn into(self) -> Key {
        Key::new(self)
    }
}

#[derive(Clone, Copy)]
struct Key {
    just_pressed: bool,
    hold: Option<Keyboard>,
    pressed: bool,
    uwu: u16,
    config: KeyConfig,
}

impl Key {
    fn new(config: KeyConfig) -> Self {
        Self {
            just_pressed: false,
            hold: None,
            pressed: false,
            uwu: 0,
            config,
        }
    }

    fn process(&mut self, input: u16) -> u16 {
        if input < self.config.min - AUTOCALIBRATION_DEADZONE {
            self.config.min = input;
        } else if input > self.config.max + AUTOCALIBRATION_DEADZONE {
            self.config.max = input;
        }
        let depth = (((input - self.config.min) as f32
            / (self.config.max - self.config.min) as f32)
            .clamp(0., 1.)
            * SWITCH_TRAVEL as f32) as u16;
        self.just_pressed = false;
        if self.pressed {
            if depth < self.uwu {
                // pressed deeper
                self.uwu = depth;
            } else if depth > SWITCH_TRAVEL - 5 || depth > self.uwu + self.config.rt_up {
                // let go
                self.hold = None;
                self.pressed = false;
                self.uwu = depth;
            }
        } else {
            if depth > self.uwu {
                // let go further
                self.uwu = depth;
            } else if depth < self.uwu - self.config.rt_down {
                // just pressed
                self.just_pressed = true;
                self.pressed = true;
                self.uwu = depth;
            }
        }
        depth
    }
}

// this probably should be shorter since it takes a little while to compare
// but for now I made it fit into 1 flash sector (4096 bytes)
const PAN_TADEUSZ: &[u8] = "Adam Mickiewicz
Pan Tadeusz czyli ostatni zajazd na Litwie


Księga pierwsza
Gospodarstwo

    Litwo! Ojczyzno moja! ty jesteś jak zdrowie:
Ile cię trzeba cenić, ten tylko się dowie,
Kto cię stracił. Dziś piękność twą w całej ozdobie
Widzę i opisuję, bo tęsknię po tobie.

    Panno święta, co Jasnej bronisz Częstochowy
I w Ostrej świecisz Bramie! Ty, co gród zamkowy
Nowogródzki ochraniasz z jego wiernym ludem!
Jak mnie dziecko do zdrowia powróciłaś cudem
(Gdy od płaczącej matki, pod Twoją opiekę
Ofiarowany, martwą podniosłem powiekę;
I zaraz mogłem pieszo, do Twych świątyń progu
Iść za wrócone życie podziękować Bogu),
Tak nas powrócisz cudem na Ojczyzny łono.
Tymczasem przenoś moją duszę utęsknioną
Do tych pagórków leśnych, do tych łąk zielonych,
Szeroko nad błękitnym Niemnem rozciągnionych;
Do tych pól malowanych zbożem rozmaitem,
Wyzłacanych pszenicą, posrebrzanych żytem;
Gdzie bursztynowy świerzop, gryka jak śnieg biała,
Gdzie panieńskim rumieńcem dzięcielina pała,
A wszystko przepasane jakby wstęgą, miedzą
Zieloną, na niej z rzadka ciche grusze siedzą.

    Śród takich pól przed laty, nad brzegiem ruczaju,
Na pagórku niewielkim, we brzozowym gaju,
Stał dwór szlachecki, z drzewa, lecz podmurowany;
Świeciły się z daleka pobielane ściany,
Tym bielsze, że odbite od ciemnej zieleni
Topoli, co go bronią od wiatrów jesieni.
Dom mieszkalny niewielki, lecz zewsząd chędogi,
I stodołę miał wielką, i przy niej trzy stogi
Użątku, co pod strzechą zmieścić się nie może.
Widać, że okolica obfita we zboże,
I widać z liczby kopic, co wzdłuż i wszerz smugów
Świecą gęsto jak gwiazdy, widać z liczby pługów
Orzących wcześnie łany ogromne ugoru,
Czarnoziemne, zapewne należne do dworu,
Uprawne dobrze na kształt ogrodowych grządek:
Że w tym domu dostatek mieszka i porządek.
Brama na wciąż otwarta przechodniom ogłasza,
Że gościnna, i wszystkich w gościnę zaprasza.

    Właśnie dwukonną bryką wjechał młody panek
I obiegłszy dziedziniec zawrócił przed ganek.
Wysiadł z powozu; konie porzucone same,
Szczypiąc trawę ciągnęły powoli pod bramę.
We dworze pusto: bo drzwi od ganku zamknięto
Zaszczepkami i kołkiem zaszczepki przetknięto.
Podróżny do folwarku nie biegł sług zapytać,
Odemknął, wbiegł do domu, pragnął go powitać.
Dawno domu nie widział, bo w dalekim mieście
Kończył nauki, końca doczekał nareszcie.
Wbiega i okiem chciwie ściany starodawne
Ogląda czule, jako swe znajome dawne.
Też same widzi sprzęty, też same obicia,
Z którymi się zabawiać lubił od powicia,
Lecz mniej wielkie, mniej piękne niż się dawniej zdały.
I też same portrety na ścianach wisiały:
Tu Kościuszko w czamarce krakowskiej, z oczyma
Podniesionymi w niebo, miecz oburącz trzyma;
Takim był, gdy przysięgał na stopniach ołtarzów,
Że tym mieczem wypędzi z Polski trzech mocarzów,
Albo sam na nim padnie. Dalej w polskiej szacie
Siedzi Rejtan, żałośny po wolności stracie;
W ręku trzyma nóż ostrzem zwrócony do łona,
A przed nim leży Fedon i żywot Katona.
Dalej Jasiński, młodzian piękny i posępny;
Obok Korsak, towarzysz jego nieodstępny:
Stoją na szańcach Pragi, na stosach Moskali,
Siekąc wrogów, a Praga już się wkoło pali.
Nawet stary stojący zegar kurantowy
W drewnianej szafie poznał, u wniścia alkowy;
I z dziecinną radością pociągnął za sznurek,
By stary Dąbrowskiego usłyszeć mazurek.

    Biegał po całym domu i szukał komnaty,
Gdzie mieszkał dzieckiem będąc, przed dziesięciu laty.
Wchodzi, cofnął się, toczył zdumione źrenice
Po ścianach: w tej komnacie mieszkanie kobiéce!
Któż by tu mieszkał? Stary stryj nie był żonaty;
A ciotka w Petersburgu mieszkała przed laty.
To nie był ochmistrzyni pokój? Fortepiano?
Na nim nuty i książki; wszystko porzucano
Niedbale i bezładnie: nieporządek miły!
Niestare były rączki, co je tak rzuciły.
Tuż i sukienka biała, świeżo z kołka zdjęta
Do ubrania, na krzesła poręczu rozpięta;
A na oknach"
    .as_bytes();
