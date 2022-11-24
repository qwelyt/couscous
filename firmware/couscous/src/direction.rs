pub mod direction {
    #[derive(Eq, PartialEq, Ord, PartialOrd, Debug, Copy, Clone)]
    pub enum Direction {
        Row2Col,
        Col2Row,
    }

    impl Direction {
        pub fn name(self) -> &'static str {
            stringify!(self)
        }
    }
}
