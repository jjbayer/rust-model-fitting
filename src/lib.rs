extern crate rand;
use rand::Rng;


pub trait Model {

    type Point;
    const MIN_SAMPLE_SIZE: usize; 

    fn distance(&self, &Self::Point) -> f64;

    fn from_points(&Vec<Self::Point>) -> Self;

    fn is_degenerate(&Vec<Self::Point>) -> bool;
}




fn random_sample<'a, T: Clone>(items: &'a Vec<T>, size: usize)
    -> impl Iterator<Item=T> + 'a
{
    (0..size).map(move |_| {
        let index = rand::thread_rng().gen_range(0, items.len());
        
        items[index].clone()
    })
}

pub fn ransac<T: Model>(points: &Vec<T::Point>, inlier_threshold: f64, num_iterations: u32) -> T
    where T::Point: Clone
{ 
    let candidates = (0..num_iterations)
        .map(|_| random_sample(points, T::MIN_SAMPLE_SIZE).collect())
        .filter(|sample| ! T::is_degenerate(sample))
        .map(|sample| T::from_points(&sample));
    ;
    
    let inlier_sets = candidates.map(|candidate|
        points.iter().filter(|point| 
            candidate.distance(point) < inlier_threshold
        ).map(|point| point.clone()).collect()
    );

    let largest_inlier_set = inlier_sets.max_by_key(
        |inliers: &Vec<T::Point>| inliers.len()
    ).expect("num_iterations zero or all models degenerate");
    
    T::from_points(&largest_inlier_set)
}