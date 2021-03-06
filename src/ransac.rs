use super::model::Model;
use super::random::random_sample;


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