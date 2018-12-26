pub trait Model {

    type Point;
    const MIN_SAMPLE_SIZE: usize;

    fn distance(&self, &Self::Point) -> f64;

    fn from_points(&Vec<Self::Point>) -> Self;

    fn is_degenerate(&Vec<Self::Point>) -> bool;
}