extern crate rand;
use rand::Rng;


pub fn random_sample<'a, T: Clone>(items: &'a Vec<T>, size: usize)
    -> impl Iterator<Item=T> + 'a
{
    (0..size).map(move |_| {
        let index = rand::thread_rng().gen_range(0, items.len());

        items[index].clone()
    })
}