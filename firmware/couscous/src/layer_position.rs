pub mod layer_position {
    use crate::position::position::Position;

    #[derive(Eq, PartialEq, Copy, Clone)]
    pub struct LayerPosition {
        position: Position,
        layer: u8,
    }

    impl LayerPosition {
        pub const fn new(position: Position, layer: u8) -> Self {
            Self {
                position,
                layer,
            }
        }


        pub fn position(&self) -> Position {
            self.position
        }

        pub fn layer(&self) -> u8 {
            self.layer
        }
    }
}