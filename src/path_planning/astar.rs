use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Ordering;

/// Grid cell for A* search.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GridCell {
    pub x: i32,
    pub y: i32,
}

impl GridCell {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// 4-connected neighbors.
    pub fn neighbors4(&self) -> Vec<GridCell> {
        vec![
            GridCell::new(self.x + 1, self.y),
            GridCell::new(self.x - 1, self.y),
            GridCell::new(self.x, self.y + 1),
            GridCell::new(self.x, self.y - 1),
        ]
    }

    /// 8-connected neighbors.
    pub fn neighbors8(&self) -> Vec<GridCell> {
        let mut n = self.neighbors4();
        n.push(GridCell::new(self.x + 1, self.y + 1));
        n.push(GridCell::new(self.x + 1, self.y - 1));
        n.push(GridCell::new(self.x - 1, self.y + 1));
        n.push(GridCell::new(self.x - 1, self.y - 1));
        n
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct OpenSetEntry {
    f: f64,
    cell: GridCell,
}

impl Eq for OpenSetEntry {}

impl PartialOrd for OpenSetEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.f.partial_cmp(&other.f)
    }
}

impl Ord for OpenSetEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

/// Manhattan distance heuristic.
pub fn manhattan_distance(a: &GridCell, b: &GridCell) -> f64 {
    ((a.x - b.x).abs() + (a.y - b.y).abs()) as f64
}

/// Euclidean distance heuristic.
pub fn euclidean_distance(a: &GridCell, b: &GridCell) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// A* path planning on a 2D grid.
/// `occupied` returns true if a cell is blocked.
/// Returns the path from start to goal (inclusive) or None.
pub fn astar_grid<F: Fn(GridCell) -> bool>(
    start: GridCell,
    goal: GridCell,
    occupied: F,
    heuristic: fn(&GridCell, &GridCell) -> f64,
    use_8_connected: bool,
) -> Option<Vec<GridCell>> {
    let max_explored = 100_000;
    let mut open_set = BinaryHeap::new();
    open_set.push(OpenSetEntry { f: 0.0, cell: start });

    let mut came_from: HashMap<GridCell, GridCell> = HashMap::new();
    let mut g_score: HashMap<GridCell, f64> = HashMap::new();
    let mut closed: HashSet<GridCell> = HashSet::new();
    g_score.insert(start, 0.0);

    let mut explored = 0;

    while let Some(OpenSetEntry { cell: current, .. }) = open_set.pop() {
        if closed.contains(&current) {
            continue;
        }
        closed.insert(current);
        explored += 1;

        if explored > max_explored {
            return None;
        }

        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        let neighbors = if use_8_connected {
            current.neighbors8()
        } else {
            current.neighbors4()
        };

        for neighbor in neighbors {
            if occupied(neighbor) || closed.contains(&neighbor) {
                continue;
            }

            let move_cost = if (neighbor.x - current.x).abs() + (neighbor.y - current.y).abs() == 2 {
                std::f64::consts::SQRT_2
            } else {
                1.0
            };

            let tentative_g = g_score.get(&current).copied().unwrap_or(f64::MAX) + move_cost;

            if tentative_g < g_score.get(&neighbor).copied().unwrap_or(f64::MAX) {
                came_from.insert(neighbor, current);
                g_score.insert(neighbor, tentative_g);
                let f = -(tentative_g + heuristic(&neighbor, &goal));
                open_set.push(OpenSetEntry { f, cell: neighbor });
            }
        }
    }

    None
}

fn reconstruct_path(came_from: &HashMap<GridCell, GridCell>, current: GridCell) -> Vec<GridCell> {
    let mut path = vec![current];
    let mut c = current;
    while let Some(&prev) = came_from.get(&c) {
        path.push(prev);
        c = prev;
    }
    path.reverse();
    path
}
