extern crate nalgebra as na;
extern crate rand;

mod lib;
use lib::Model;
use lib::ransac;



struct Line2D {
    params: na::Vector3<f64>,
}

impl std::fmt::Display for Line2D {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        self.params.fmt(f)
    }
}


impl Model for Line2D {

    type Point = na::Vector3<f64>;
    const MIN_SAMPLE_SIZE:  usize = 2;

    fn distance(&self, p: &Self::Point) -> f64 {
        
        self.params.normalize().dot(&p.normalize()).abs()
    }

    fn from_points(points: &Vec<Self::Point>) -> Line2D {
        // TODO: take iterator as parameter

        // TODO: simplify code. Would like  to write it  like this:
        // let A = DMatf64::from_columns(points.iter())
        // let (ev, lambdas) = (A.t() * A).eigen();
        // return ev[0]
        assert!(points.len() >= Self::MIN_SAMPLE_SIZE);    

        let mut a = na::DMatrix::<f64>::zeros(3, points.len());
        for (i, point) in points.iter().enumerate() {
            a.set_column(i, point);
        } 
        let ev = (&a * a.transpose()).symmetric_eigen();
        let mut mini = 0;
        let mut minv = std::f64::INFINITY;
        for (i, lambda) in ev.eigenvalues.iter().enumerate() {
            if lambda < &minv { mini = i; minv = *lambda; }
        }
        let c = ev.eigenvectors.column(mini);
        // TODO: convert dynamic to fixed
        Line2D {params: Self::Point::new(c[0], c[1], c[2])}
    }

    fn is_degenerate(points: &Vec<Self::Point>) -> bool {
        assert!(points.len() == Self::MIN_SAMPLE_SIZE);  // TODO: check at compile time

        points[0] != points[1]  // FIXME: relative_eq(epsilon...)
    }
}


fn main() {
    let p1 = na::Vector3::new(1., 2., 1.);
    let p2 = na::Vector3::new(2., 3., 1.);
    let p3 = na::Vector3::new(3., 4., 1.);
    let p4 = na::Vector3::new(50., 50., 1.);
    let all_points = vec![p1, p2, p3, p4];
    // let line = Line2D::from_points(&vec![p1, p2]);
    let estimate = ransac::<Line2D>(&all_points, 0.1, 100);
    // let sample = random_sample(&all_points);

    let mut line = estimate.params.clone();

    line /= -line[1];

    println!("Line: y = {} * x + {}", line[0], line[2]);
}
