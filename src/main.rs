extern crate nalgebra as na;
extern crate rand;
use rand::Rng;


trait Model {

    type Point;
    const MIN_SAMPLE_SIZE: usize; 

    fn distance(&self, &Self::Point) -> f64;

    fn from_points(&Vec<Self::Point>) -> Self;

    fn is_degenerate(&Vec<Self::Point>) -> bool;
}


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

fn random_sample<T: Clone>(items: &Vec<T>, size: usize) -> Vec<T> {
    let mut sample = Vec::<T>::new();
    for _ in 0..size {
        let index = rand::thread_rng().gen_range(0, items.len());
        sample.push(items[index].clone());
    }
    
    sample
}

fn ransac<T: Model>(points: &Vec<T::Point>, inlier_threshold: f64, num_iterations: u32) -> T
    where T::Point: Clone
{
  
    let candidates = (0..num_iterations)
        .map(|_| random_sample(points, T::MIN_SAMPLE_SIZE))
        .filter(|sample| ! T::is_degenerate(sample))
        .map(|sample| T::from_points(&sample));
    ;
    
    let inlier_sets = candidates.map(|candidate|
        points.iter().filter(|point|
            candidate.distance(point) < inlier_threshold
        ).map(|point| point.clone()).collect()
    );

    // Every model is created from MIN_SAMPLE_SIZE instances, so each
    // inlier set must have enough
    // assert!(inlier_sets.all(|inliers: Vec<T::Point>| inliers.len() > T::MIN_SAMPLE_SIZE));
    
    let largest_inlier_set = inlier_sets.max_by_key(
        |inliers: &Vec<T::Point>| inliers.len()
    ).expect("No");
    
    T::from_points(&largest_inlier_set)
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
