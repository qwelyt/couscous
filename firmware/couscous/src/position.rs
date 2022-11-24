pub mod position {
    #[derive(Eq, PartialEq, Ord, PartialOrd, Debug, Copy, Clone)]
    pub enum Direction {
        Row2Col,
        Col2Row,
    }

    impl Direction {
        fn name(self) -> &'static str {
            stringify!(self)
        }
    }

    #[derive(Eq, PartialEq, Copy, Clone, Debug)]
    pub struct Position {
        row: u8,
        col: u8,
        direction: Direction,
    }

    impl Position {
        pub const fn new(row: u8, col: u8, direction: Direction) -> Self {
            Self {
                row,
                col,
                direction,
            }
        }

        pub fn row(&self) -> u8 {
            self.row
        }
        pub fn col(&self) -> u8 {
            self.col
        }
        pub fn direction(&self) -> Direction {
            self.direction
        }
    }

    impl hash32::Hash for Position {
        fn hash<H>(&self, state: &mut H)
            where
                H: hash32::Hasher,
        {
            self.row.hash(state);
            self.col.hash(state);
            self.direction.name().hash(state)
        }
    }
}