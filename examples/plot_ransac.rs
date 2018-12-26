extern crate nalgebra as na;
extern crate rand;


extern crate plotlib;
use plotlib::scatter::Scatter;
use plotlib::view::View;
use plotlib::page::Page;
use plotlib::style::Line;

extern crate model_fitting;
use model_fitting::model::Model;
use model_fitting::line2d::Line2D;
use model_fitting::ransac::ransac;


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


fn plot(points: &Vec<<Line2D as Model>::Point>, estimate: &Line2D)
{
    let mut line = estimate.params.clone();
    line /= -line[1];

    let data: Vec<(f64, f64)> = points.iter().map(
        |p| (p[0]/p[2], p[1]/p[2])
    ).collect();

    let s1 = Scatter::from_vec(&data);

    let estimated_line = plotlib::function::Function::new(|x| line[0] * x + line[2], 0.0, 50.0)
        .style(plotlib::function::Style::new().colour("red"));

    let v = View::new().add(&s1).add(&estimated_line);

    Page::single(&v).save("ransac.svg");
    println!("Saved ransac.svg");
}