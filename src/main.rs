extern crate nalgebra as na;
extern crate rand;

extern crate ordered_float;
use ordered_float::OrderedFloat;

extern crate plotlib;
use plotlib::scatter::Scatter;
use plotlib::view::View;
use plotlib::page::Page;
use plotlib::style::Line;

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
        let p_norm = p / p[2];
        self.params.normalize().dot(&p_norm).abs()
    }

    fn from_points(points: &Vec<Self::Point>) -> Line2D {
        // TODO: take iterator as parameter

        assert!(points.len() >= Self::MIN_SAMPLE_SIZE);

        type Points = na::Matrix<f64, na::U3, na::Dynamic, na::MatrixVec<f64, na::U3, na::Dynamic>>;
        let a = Points::from_columns(&points[..]);

        // let A = DMatf64::from_columns(points.iter())

        // Get  eigenvector of smallest eigen value
        let ev = (&a * a.transpose()).symmetric_eigen();
        let minimum = ev.eigenvalues
            .iter()
            .enumerate()
            .min_by_key(|&(_, lambda)| OrderedFloat(*lambda));
        let (index, _) = minimum.expect("No eigenvalues found");
        let v = ev.eigenvectors.column(index);

        Line2D {params: Self::Point::new(v[0], v[1], v[2])}
    }

    fn is_degenerate(points: &Vec<Self::Point>) -> bool {
        assert!(points.len() == Self::MIN_SAMPLE_SIZE);  // TODO: check at compile time

        points[0] != points[1]  // FIXME: relative_eq(epsilon...)
    }
}


fn plot(points: &Vec<<Line2D as Model>::Point>, estimate: &Line2D)
{
    let mut line = estimate.params.clone();
    line /= -line[1];

    // TODO: why can't I write map(...).collect()?
    let mut data = Vec::<(f64,f64)>::new();
    for p in points.iter() {
        data.push((p[0]/p[2], p[1]/p[2]));
    }

    let s1 = Scatter::from_vec(&data);

    let estimated_line = plotlib::function::Function::new(|x| line[0] * x + line[2], 0.0, 50.0)
        .style(plotlib::function::Style::new().colour("red"));

    let v = View::new().add(&s1).add(&estimated_line);

    Page::single(&v).save("ransac.svg");
}

fn main() {
    let points = vec![
        na::Vector3::new(1.1, 2.04, 1.),
        na::Vector3::new(2.2, 3.1, 1.),
        na::Vector3::new(3.05, 4.3, 1.),
         na::Vector3::new(4.2, 5.15, 1.),
        na::Vector3::new(50., 20., 1.),
    ];
    let estimate = ransac::<Line2D>(&points, 1., 100);

    plot(&points, &estimate);
}
