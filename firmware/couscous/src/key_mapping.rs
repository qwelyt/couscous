#[allow(dead_code)]
use heapless::FnvIndexSet;

use crate::layer_position::layer_position::LayerPosition;
use crate::position::position::Direction::{Col2Row, Row2Col};
use crate::position::position::Position;

pub fn meta_value(key: u8) -> u8 {
    match key {
        key::L_CTRL => mo::LCTRL,
        key::L_SHFT => mo::LSHFT,
        key::L_ALT => mo::LALT,
        key::L_SUPR => mo::LSUPR,
        key::R_CTRL => mo::RCTRL,
        key::R_SHFT => mo::RSHFT,
        key::R_ALT => mo::RALT,
        key::R_SUPR => mo::RSUPR,
        _ => 0,
    }
}

const LAYERS: [LayerPosition; 1] = [LayerPosition::new(Position::new(4, 2, Col2Row), 1)];
const LAYOUT: [[[(u8, u8); 6]; 5]; 2] = [
    [
        [(key::K1, key::K2), (key::K3, key::K4), (key::K5, key::K6), (key::K7, key::K8), (key::K9, key::K0), (key::DASH, key::EQUAL)],
        [(key::TAB, key::Q), (key::W, key::E), (key::R, key::T), (key::Y, key::U), (key::I, key::O), (key::P, key::OBRAKET), ],
        [(key::ESC, key::A), (key::S, key::D), (key::F, key::G), (key::H, key::J), (key::K, key::L), (key::COLON, key::QUOTE), ],
        [(key::L_SHFT, key::Z), (key::X, key::C), (key::V, key::B), (key::N, key::M), (key::COMMA, key::DOT), (key::SLASH, key::R_SHFT), ],
        [(key::L_CTRL, key::L_SUPR), (key::BS_N_PIPE, key::L_ALT), (key::RAISE, key::SPACE), (key::RETURN, key::BACKSPACE), (key::R_ALT, key::MENU), (key::R_SUPR, key::R_CTRL), ],
    ],
    [
        [(key::F1, key::F2), (key::F3, key::F4), (key::F5, key::F6), (key::F7, key::F8), (key::F9, key::F10), (key::F11, key::F12)],
        [(key::TAB, key::Q), (key::W, key::E), (key::R, key::T), (key::HOME, key::PGUP), (key::PGDWN, key::END), (key::PRNT_SCRN, key::CBRAKET), ],
        [(key::ESC, key::A), (key::S, key::D), (key::F, key::INSERT), (key::ARROW_L, key::ARROW_D), (key::ARROW_U, key::ARROW_R), (key::TILDE, key::GACC), ],
        [(key::L_SHFT, key::Z), (key::X, key::C), (key::V, key::B), (key::N, key::M), (key::COMMA, key::DOT), (key::SLASH, key::R_SHFT), ],
        [(key::L_CTRL, key::L_SUPR), (key::BS_N_PIPE, key::L_ALT), (key::RAISE, key::SPACE), (key::RETURN, key::DELETE), (key::R_ALT, key::MENU), (key::R_SUPR, key::R_CTRL), ],
    ],
];

pub fn layer(state: &FnvIndexSet<Position, 64>) -> u8 {
    let mut layer = 0;

    for l in LAYERS {
        if state.contains(&l.position()) {
            layer += l.layer();
        }
    }

    layer
}

pub fn map_pos_to_key(layer: u8, position: &Position) -> u8 {
    LAYOUT.get(layer as usize)
        .and_then(|layer| layer.get(position.row() as usize))
        .and_then(|row| row.get(position.col() as usize))
        .and_then(|pos| match position.direction() {
            Row2Col => { Some(pos.1) }
            Col2Row => { Some(pos.0) }
        })
        .unwrap_or(key::NONE)
}


mod key {
    pub const NONE: u8 = 0x00;
    pub const A: u8 = 0x04;
    pub const B: u8 = 0x05;
    pub const C: u8 = 0x06;
    pub const D: u8 = 0x07;
    pub const E: u8 = 0x08;
    pub const F: u8 = 0x09;
    pub const G: u8 = 0x0A;
    pub const H: u8 = 0x0B;
    pub const I: u8 = 0x0C;
    pub const J: u8 = 0x0D;
    pub const K: u8 = 0x0E;
    pub const L: u8 = 0x0F;
    pub const M: u8 = 0x10;
    pub const N: u8 = 0x11;
    pub const O: u8 = 0x12;
    pub const P: u8 = 0x13;
    pub const Q: u8 = 0x14;
    pub const R: u8 = 0x15;
    pub const S: u8 = 0x16;
    pub const T: u8 = 0x17;
    pub const U: u8 = 0x18;
    pub const V: u8 = 0x19;
    pub const W: u8 = 0x1A;
    pub const X: u8 = 0x1B;
    pub const Y: u8 = 0x1C;
    pub const Z: u8 = 0x1D;

    pub const K1: u8 = 0x1E;
    pub const K2: u8 = 0x1F;
    pub const K3: u8 = 0x20;
    pub const K4: u8 = 0x21;
    pub const K5: u8 = 0x22;
    pub const K6: u8 = 0x23;
    pub const K7: u8 = 0x24;
    pub const K8: u8 = 0x25;
    pub const K9: u8 = 0x26;
    pub const K0: u8 = 0x27;

    pub const RETURN: u8 = 0x28;
    pub const ESC: u8 = 0x29;
    pub const BACKSPACE: u8 = 0x2A;
    pub const TAB: u8 = 0x2B;
    pub const SPACE: u8 = 0x2C;
    pub const DASH: u8 = 0x2D;
    pub const EQUAL: u8 = 0x2E;
    pub const OBRAKET: u8 = 0x2F;
    pub const CBRAKET: u8 = 0x30;
    pub const BSLASH: u8 = 0x31;
    pub const TILDE: u8 = 0x32;
    pub const COLON: u8 = 0x33;
    pub const QUOTE: u8 = 0x34;
    pub const GACC: u8 = 0x35;
    pub const COMMA: u8 = 0x36;
    pub const DOT: u8 = 0x37;
    pub const SLASH: u8 = 0x38;
    pub const CAPSLOCK: u8 = 0x39;

    pub const F1: u8 = 0x3A;
    pub const F2: u8 = 0x3B;
    pub const F3: u8 = 0x3C;
    pub const F4: u8 = 0x3D;
    pub const F5: u8 = 0x3E;
    pub const F6: u8 = 0x3F;
    pub const F7: u8 = 0x40;
    pub const F8: u8 = 0x41;
    pub const F9: u8 = 0x42;
    pub const F10: u8 = 0x43;
    pub const F11: u8 = 0x44;
    pub const F12: u8 = 0x45;

    pub const PRNT_SCRN: u8 = 0x46;
    pub const SCRL_LCK: u8 = 0x47;
    pub const PAUSE: u8 = 0x48;
    pub const INSERT: u8 = 0x49;
    pub const HOME: u8 = 0x4A;
    pub const PGUP: u8 = 0x4B;
    pub const DELETE: u8 = 0x4C;
    pub const END: u8 = 0x4D;
    pub const PGDWN: u8 = 0x4E;
    pub const ARROW_R: u8 = 0x4F;
    pub const ARROW_L: u8 = 0x50;
    pub const ARROW_D: u8 = 0x51;
    pub const ARROW_U: u8 = 0x52;

    pub const NUM_LCK: u8 = 0x53;
    pub const N_DIV: u8 = 0x54;
    pub const N_MUL: u8 = 0x55;
    pub const N_SUB: u8 = 0x56;
    pub const N_ADD: u8 = 0x57;
    pub const N_ENTER: u8 = 0x58;
    pub const N1: u8 = 0x59;
    pub const N2: u8 = 0x5A;
    pub const N3: u8 = 0x5B;
    pub const N4: u8 = 0x5C;
    pub const N5: u8 = 0x5D;
    pub const N6: u8 = 0x5E;
    pub const N7: u8 = 0x5F;
    pub const N8: u8 = 0x60;
    pub const N9: u8 = 0x61;
    pub const N0: u8 = 0x62;
    pub const NDOT: u8 = 0x63;

    pub const BS_N_PIPE: u8 = 0x64;
    pub const MENU: u8 = 0x65;


    // Special
    pub const L_CTRL: u8 = 0xE0;
    pub const L_SHFT: u8 = 0xE1;
    pub const L_ALT: u8 = 0xE2;
    pub const L_SUPR: u8 = 0xE3;
    pub const R_CTRL: u8 = 0xE4;
    pub const R_SHFT: u8 = 0xE5;
    pub const R_ALT: u8 = 0xE6;
    pub const R_SUPR: u8 = 0xE7;

    pub const LOWER: u8 = 0x00;
    pub const RAISE: u8 = 0x00;
}

mod mo {
    pub const LCTRL: u8 = 0b00000001;
    pub const LSHFT: u8 = 0b00000010;
    pub const LALT: u8 = 0b00000100;
    pub const LSUPR: u8 = 0b00001000;
    pub const RCTRL: u8 = 0b00010000;
    pub const RSHFT: u8 = 0b00100000;
    pub const RALT: u8 = 0b01000000;
    pub const RSUPR: u8 = 0b10000000;
}