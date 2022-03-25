use crate::all::*;

pub struct Pyramid {
  pub levels: Vec<Vec<u8>>,
}

impl Pyramid {
  pub fn new(
    data: &[u8],
    unused_pyramid: Option<Pyramid>,
    level_count: usize,
  ) -> Pyramid {
    Pyramid {
      levels: compute_levels(
        data,
        unused_pyramid.map(|x| x.levels),
        level_count,
      ),
    }
  }
}

fn compute_levels(
  data: &[u8],
  unused_levels: Option<Vec<Vec<u8>>>,
  level_count: usize,
) -> Vec<Vec<u8>> {
  let mut levels = unused_levels.unwrap_or(vec![vec![]; PYRAMID_LEVEL_COUNT]);
  for level_ind in 0..PYRAMID_LEVEL_COUNT {
    levels[level_ind].clear();
    // TODO The usual issue: trying to borrow non-mutably level-1, and mutably
    // level+0 doesn't work.
    let parent_data = if level_ind > 0 { &levels[level_ind - 1] } else { data };
    downscale(parent_data, &mut levels[level_ind]);
  }
  vec![]
}

fn downscale(parent: &[u8], child: &mut Vec<u8>) {

}
