extern crate nalgebra as na;
use super::model::Model;

extern crate ordered_float;
use self::ordered_float::OrderedFloat;

use std::fmt;


pub struct Line2D {
    pub params: na::Vector3<f64>,
}


// Matrix which holds points as columns
type Points = na::Matrix<f64, na::U3, na::Dynamic, na::MatrixVec<f64, na::U3, na::Dynamic>>;


impl Model for Line2D {

    type Point = na::Vector3<f64>;
    const MIN_SAMPLE_SIZE:  usize = 2;

    fn distance(&self, p: &Self::Point) -> f64 {
        let p_norm = p / p[2];
        self.params.normalize().dot(&p_norm).abs()
    }

    fn from_points(points: &Vec<Self::Point>) -> Line2D {
        // TODO: take iterator as parameter

        assert!(points.len() >= Self::MIN_SAMPLE_SIZE);

        let point_matrix = Points::from_columns(&points[..]);

        Line2D {params: solve_homogeneous(&point_matrix)}
    }

    fn is_degenerate(points: &Vec<Self::Point>) -> bool {
        assert!(points.len() == Self::MIN_SAMPLE_SIZE);  // TODO: check at compile time

        points[0] != points[1]  // FIXME: relative_eq(epsilon...)
    }
}


impl fmt::Display for Line2D {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.params.fmt(f)
    }
}


pub fn solve_homogeneous(a: &Points) -> <Line2D as Model>::Point {
    // Solve linear equation system Ax = 0 (equations as columns)
    // TODO: make generic and put in math.rs
    //       problem: which traits to require?

    let ev = (a * a.transpose()).symmetric_eigen();
    let minimum = ev.eigenvalues
        .iter()
        .enumerate()
        .min_by_key(|&(_, lambda)| OrderedFloat(*lambda));
    let (index, _) = minimum.expect("No eigenvalues found");

    let v = ev.eigenvectors.column(index);

    <Line2D as Model>::Point::new(v[0], v[1], v[2])
}
