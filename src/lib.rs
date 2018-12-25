extern crate rand;
use rand::Rng;


pub trait Model {

    type Point;
    const MIN_SAMPLE_SIZE: usize; 

    fn distance(&self, &Self::Point) -> f64;

    fn from_points(&Vec<Self::Point>) -> Self;

    fn is_degenerate(&Vec<Self::Point>) -> bool;
}




fn random_sample<T: Clone>(items: &Vec<T>, size: usize) -> Vec<T> {
    let mut sample = Vec::<T>::new();
    for _ in 0..size {
        let index = rand::thread_rng().gen_range(0, items.len());
        sample.push(items[index].clone());
    }
    
    sample
}

pub fn ransac<T: Model>(points: &Vec<T::Point>, inlier_threshold: f64, num_iterations: u32) -> T
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