// Sokoban push-optimal solver — room decomposition + macro-push A*.
//
// Strategy (from the competition spec): the generator lays a 1-wide hallway
// across the board and 1-4 rooms off it, each opening through ONE door. A box
// pushed out through a door can never be pushed back, so in an optimal
// solution every box stays in its room and a level's push-optimum is the sum
// of its rooms' optima. We therefore split each level into independent rooms
// and solve every room push-optimally with A* over pushes, then concatenate
// the per-room move sequences (walking the hallway between rooms).
//
// A* state: sorted box cells + player cell. A step is one PUSH: the player
// walks (free) to the cell behind a box, then steps into the box. Cost is
// pushes only. Heuristic: minimum assignment of boxes to goals where the
// cost of a box to a goal is the shortest cell-distance on the room floor
// (Hungarian over <=4 boxes, brute-forced). Deadlock pruning: dead squares
// (reverse-pull reachability from goals), 2x2 freeze blocks, wall freeze.
//
// Deterministic: fixed direction order everywhere, heap tie-breaks by
// insertion order, no clock, no randomness, no environment reads. The same
// input always produces the same bytes.

import fs from 'node:fs';
import path from 'node:path';

// Directions in fixed order: up, down, left, right.
const DIRS = [
  { dr: -1, dc: 0, ch: 'u' },
  { dr: 1, dc: 0, ch: 'd' },
  { dr: 0, dc: -1, ch: 'l' },
  { dr: 0, dc: 1, ch: 'r' },
];

// ---------------------------------------------------------------- parsing

function parseLevel(text) {
  const lines = text.split('\n').map((l) => l.replace(/\r$/, ''));
  while (lines.length && lines[lines.length - 1] === '') lines.pop();
  const H = lines.length;
  const W = Math.max(...lines.map((l) => l.length));
  const grid = [];
  for (let r = 0; r < H; r++) {
    const row = [];
    for (let c = 0; c < W; c++) {
      const ch = c < lines[r].length ? lines[r][c] : '#';
      row.push(ch);
    }
    grid.push(row);
  }
  const walls = new Set();
  const goals = [];
  const boxes = [];
  let player = -1;
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      const ch = grid[r][c];
      const idx = r * W + c;
      if (ch === '#') walls.add(idx);
      if (ch === '.' || ch === '*' || ch === '+') goals.push(idx);
      if (ch === '$' || ch === '*') boxes.push(idx);
      if (ch === '@' || ch === '+') player = idx;
    }
  }
  return { H, W, walls, goals, boxes, player };
}

const isFloor = (level, idx) => !level.walls.has(idx);

// ------------------------------------------------------------- room split

function detectRooms(level) {
  const { H, W, player } = level;
  if (player < 0) return null;
  const hallRow = Math.floor(player / W);
  // Door cells: floor cells in the row above/below the hallway, adjacent to
  // a hallway floor cell.
  const doors = [];
  for (let c = 0; c < W; c++) {
    const hallCell = hallRow * W + c;
    if (!isFloor(level, hallCell)) continue;
    for (const dr of [-1, 1]) {
      const r = hallRow + dr;
      if (r < 0 || r >= H) continue;
      const cell = r * W + c;
      if (isFloor(level, cell)) doors.push({ cell, start: hallCell, above: dr < 0 });
    }
  }
  if (doors.length === 0) return null;
  const rooms = [];
  const seenCells = new Set();
  for (const door of doors) {
    // Flood fill the room: floor cells excluding the hallway row.
    const cells = new Set();
    const stack = [door.cell];
    cells.add(door.cell);
    while (stack.length) {
      const cur = stack.pop();
      const r = Math.floor(cur / W);
      const c = cur % W;
      for (const d of DIRS) {
        const nr = r + d.dr;
        const nc = c + d.dc;
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        if (nr === hallRow) continue;
        const nidx = nr * W + nc;
        if (!isFloor(level, nidx) || cells.has(nidx)) continue;
        cells.add(nidx);
        stack.push(nidx);
      }
    }
    const roomBoxes = level.boxes.filter((b) => cells.has(b));
    const roomGoals = level.goals.filter((g) => cells.has(g));
    // A room must hold boxes and exactly as many goals as boxes.
    if (roomBoxes.length === 0 || roomGoals.length !== roomBoxes.length) return null;
    for (const cell of cells) {
      if (seenCells.has(cell)) return null; // rooms must not overlap
      seenCells.add(cell);
    }
    rooms.push({ cells, boxes: roomBoxes, goals: roomGoals, door: door.cell, start: door.start });
  }
  // Every box and every goal must belong to exactly one room.
  const assignedBoxes = new Set(rooms.flatMap((r) => r.boxes));
  const assignedGoals = new Set(rooms.flatMap((r) => r.goals));
  if (assignedBoxes.size !== level.boxes.length) return null;
  if (assignedGoals.size !== level.goals.length) return null;
  // Sort rooms deterministically by door cell.
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// ------------------------------------------------------------ BFS helpers

// Shortest walk path (list of direction chars) from `from` to `to` over
// `floor` cells, avoiding `blocked` cells. Deterministic direction order.
function walkPath(level, floor, blocked, from, to) {
  if (from === to) return '';
  const W = level.W;
  const prev = new Map();
  const q = [from];
  prev.set(from, -1);
  let head = 0;
  while (head < q.length) {
    const cur = q[head++];
    if (cur === to) break;
    const r = Math.floor(cur / W);
    const c = cur % W;
    for (let di = 0; di < 4; di++) {
      const d = DIRS[di];
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= level.H || nc < 0 || nc >= W) continue;
      const nidx = nr * W + nc;
      if (!floor.has(nidx) || blocked.has(nidx) || prev.has(nidx)) continue;
      prev.set(nidx, cur * 4 + di);
      q.push(nidx);
    }
  }
  if (!prev.has(to)) return null;
  let out = '';
  let cur = to;
  while (cur !== from) {
    const enc = prev.get(cur);
    const di = enc % 4;
    out = DIRS[di].ch + out;
    cur = Math.floor(enc / 4);
  }
  return out;
}

// Reachable cells from `from` over `floor`, avoiding `blocked`.
function reachableSet(level, floor, blocked, from) {
  const W = level.W;
  const seen = new Set([from]);
  const q = [from];
  let head = 0;
  while (head < q.length) {
    const cur = q[head++];
    const r = Math.floor(cur / W);
    const c = cur % W;
    for (const d of DIRS) {
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= level.H || nc < 0 || nc >= W) continue;
      const nidx = nr * W + nc;
      if (!floor.has(nidx) || blocked.has(nidx) || seen.has(nidx)) continue;
      seen.add(nidx);
      q.push(nidx);
    }
  }
  return seen;
}

// ------------------------------------------------------- room A* solver

class BinaryHeap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(item) {
    const a = this.a;
    a.push(item);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (cmpNode(a[p], a[i]) <= 0) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && cmpNode(a[l], a[m]) < 0) m = l;
        if (r < a.length && cmpNode(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top;
  }
}

// Order by f, then h, then insertion sequence (deterministic).
function cmpNode(a, b) {
  if (a.f !== b.f) return a.f - b.f;
  if (a.h !== b.h) return a.h - b.h;
  return a.seq - b.seq;
}

// Solve one room push-optimally. Returns { pushes: [{from, dir}], moves, player }
// or null when no solution is found within the budget.
// `floor` = room cells; `roam` = floor + the hallway start cell (player may
// stand there, e.g. to push a box sitting on the door cell).
function solveRoom(level, room, needExit, nodeBudget) {
  const W = level.W;
  const floor = room.cells;
  const roam = new Set(floor);
  roam.add(room.start);
  const goals = room.goals;
  const goalSet = new Set(goals);
  const startBoxes = [...room.boxes].sort((a, b) => a - b);

  // Push-distance: BFS cell distances from each goal over the room floor.
  const distFromGoal = goals.map((g) => {
    const dist = new Map([[g, 0]]);
    const q = [g];
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      const r = Math.floor(cur / W);
      const c = cur % W;
      for (const d of DIRS) {
        const nr = r + d.dr;
        const nc = c + d.dc;
        if (nr < 0 || nr >= level.H || nc < 0 || nc >= W) continue;
        const nidx = nr * W + nc;
        if (!floor.has(nidx) || dist.has(nidx)) continue;
        dist.set(nidx, dist.get(cur) + 1);
        q.push(nidx);
      }
    }
    return dist;
  });

  // Hungarian assignment (brute force, <= 4 boxes) of boxes to goals.
  // Returns the min total push-distance, or Infinity.
  function heuristic(boxes) {
    const n = boxes.length;
    let best = Infinity;
    const used = new Array(n).fill(false);
    const assign = (i, sum) => {
      if (sum >= best) return;
      if (i === n) { best = sum; return; }
      for (let j = 0; j < n; j++) {
        if (used[j]) continue;
        const d = distFromGoal[j].get(boxes[i]);
        if (d === undefined) continue;
        used[j] = true;
        assign(i + 1, sum + d);
        used[j] = false;
      }
    };
    assign(0, 0);
    return best;
  }

  // Dead squares: cells from which a box can never be pushed to a goal.
  // Note the player may stand on the hallway start cell to push a box that
  // sits on the door cell, so the stand cell may be in `roam`, not `floor`.
  const live = new Set(goals);
  {
    const q = [...goals];
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      const r = Math.floor(cur / W);
      const c = cur % W;
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di];
        // Predecessor cell x' = cur - d: pushing from x' to cur needs
        // the player to stand at x' - d.
        const xr = r - d.dr;
        const xc = c - d.dc;
        const br = xr - d.dr;
        const bc = xc - d.dc;
        if (xr < 0 || xr >= level.H || xc < 0 || xc >= W) continue;
        if (br < 0 || br >= level.H || bc < 0 || bc >= W) continue;
        const x = xr * W + xc;
        const behind = br * W + bc;
        if (!roam.has(x) || !roam.has(behind) || live.has(x)) continue;
        live.add(x);
        q.push(x);
      }
    }
  }

  // Freeze deadlock (fixed point): a box is frozen when no push direction
  // has both its target cell and its stand cell free of walls and frozen
  // boxes. Boxes that can move are removed iteratively; what remains can
  // never move. A frozen box off a goal makes the state dead. (Neighbouring
  // boxes that can still move do NOT freeze a box — only walls and boxes
  // that are themselves frozen count as blocking.)
  function isDeadState(boxes) {
    for (const b of boxes) {
      if (goalSet.has(b)) continue;
      if (!live.has(b)) return true;
    }
    let frozen = new Set(boxes);
    let changed = true;
    while (changed) {
      changed = false;
      for (const b of [...frozen]) {
        const r = Math.floor(b / W);
        const c = b % W;
        let canMove = false;
        for (let di = 0; di < 4; di++) {
          const d = DIRS[di];
          const t = (r + d.dr) * W + (c + d.dc);
          const s = (r - d.dr) * W + (c - d.dc);
          const tBlocked = !roam.has(t) || frozen.has(t);
          const sBlocked = !roam.has(s) || frozen.has(s);
          if (!tBlocked && !sBlocked) { canMove = true; break; }
        }
        if (canMove) { frozen.delete(b); changed = true; }
      }
    }
    for (const b of frozen) if (!goalSet.has(b)) return true;
    return false;
  }

  function isGoalState(boxes, player) {
    for (const b of boxes) if (!goalSet.has(b)) return false;
    if (!needExit) return true;
    // Player must be able to reach the door cell to leave the room.
    const boxSet = new Set(boxes);
    const reach = reachableSet(level, roam, boxSet, player);
    return reach.has(room.door);
  }

  // A* over pushes.
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity || isDeadState(startBoxes)) return null;
  const nodes = [];
  const open = new BinaryHeap();
  let seq = 0;
  const startNode = {
    boxes: startBoxes, player: room.start, g: 0, h: h0,
    f: h0, seq: seq++, parent: -1, pushFrom: -1, pushDir: -1,
  };
  nodes.push(startNode);
  open.push(startNode);
  const best = new Map();
  best.set(startBoxes.join(',') + '|' + room.start, 0);
  let expansions = 0;
  let goalNode = null;

  while (open.size) {
    const node = open.pop();
    const key = node.boxes.join(',') + '|' + node.player;
    if (best.get(key) !== node.g) continue; // stale entry
    if (isGoalState(node.boxes, node.player)) { goalNode = node; break; }
    if (++expansions > nodeBudget) break;
    const boxSet = new Set(node.boxes);
    const reach = reachableSet(level, roam, boxSet, node.player);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const br = Math.floor(b / W);
      const bc = b % W;
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di];
        const tr = br + d.dr;
        const tc = bc + d.dc;
        const sr = br - d.dr;
        const sc = bc - d.dc;
        if (tr < 0 || tr >= level.H || tc < 0 || tc >= W) continue;
        if (sr < 0 || sr >= level.H || sc < 0 || sc >= W) continue;
        const t = tr * W + tc;
        const s = sr * W + sc;
        if (!floor.has(t) || boxSet.has(t)) continue;
        if (!reach.has(s)) continue;
        const nextBoxes = [...node.boxes];
        nextBoxes[bi] = t;
        nextBoxes.sort((a, b2) => a - b2);
        const g2 = node.g + 1;
        const key2 = nextBoxes.join(',') + '|' + b;
        if (best.has(key2) && best.get(key2) <= g2) continue;
        if (isDeadState(nextBoxes)) continue;
        const h2 = heuristic(nextBoxes);
        if (h2 === Infinity) continue;
        best.set(key2, g2);
        const nn = {
          boxes: nextBoxes, player: b, g: g2, h: h2,
          f: g2 + h2, seq: seq++, parent: nodes.indexOf(node), pushFrom: b, pushDir: di,
        };
        nodes.push(nn);
        open.push(nn);
      }
    }
  }
  if (!goalNode) return null;

  // Reconstruct the push list (from start to goal).
  const pushes = [];
  let cur = goalNode;
  const chain = [];
  while (cur.parent !== -1) {
    chain.push(cur);
    cur = nodes[cur.parent];
  }
  chain.reverse();
  for (const n of chain) pushes.push({ from: n.pushFrom, dir: n.pushDir });

  // Expand pushes to player moves, tracking box state as we go.
  let moves = '';
  let player = room.start;
  let runningBoxes = [...startBoxes];
  for (const p of pushes) {
    const d = DIRS[p.dir];
    const br = Math.floor(p.from / W);
    const bc = p.from % W;
    const stand = (br - d.dr) * W + (bc - d.dc);
    const blocked = new Set(runningBoxes);
    const walk = walkPath(level, roam, blocked, player, stand);
    if (walk === null) return null;
    moves += walk + d.ch;
    // Apply the push.
    const bi = runningBoxes.indexOf(p.from);
    runningBoxes[bi] = (br + d.dr) * W + (bc + d.dc);
    runningBoxes.sort((a, b) => a - b);
    player = p.from;
  }
  // Exit walk for non-final rooms.
  if (needExit) {
    const blocked = new Set(runningBoxes);
    const walk = walkPath(level, roam, blocked, player, room.start);
    if (walk === null) return null;
    moves += walk;
    player = room.start;
  }
  return { pushes, moves, player, boxes: runningBoxes };
}

// Whole-level fallback: macro-push A* on the entire level (no split).
function solveWhole(level, nodeBudget) {
  const floor = new Set();
  for (let r = 0; r < level.H; r++) {
    for (let c = 0; c < level.W; c++) {
      const idx = r * level.W + c;
      if (isFloor(level, idx)) floor.add(idx);
    }
  }
  const room = {
    cells: floor, boxes: [...level.boxes], goals: [...level.goals],
    door: -1, start: level.player,
  };
  // Reuse solveRoom with the whole level as one room and no exit need.
  // solveRoom expects room.start to be a roam cell; the player start is in
  // the floor set, so this works.
  const res = solveRoom(level, room, false, nodeBudget);
  if (!res) return null;
  return res.moves;
}

// ------------------------------------------------------------------ solve

function solveLevel(level) {
  const rooms = detectRooms(level);
  if (!rooms) {
    if (level.boxes.length > 8) return ''; // whole-level search would blow the budget
    const moves = solveWhole(level, 1000000);
    return moves === null ? '' : moves;
  }
  let moves = '';
  let player = level.player;
  const W = level.W;
  for (let i = 0; i < rooms.length; i++) {
    const room = rooms[i];
    const isLast = i === rooms.length - 1;
    // Walk along the hallway (boxes never enter it) to this room's door.
    const hallFloor = new Set();
    const hallRow = Math.floor(level.player / W);
    for (let c = 0; c < W; c++) {
      const idx = hallRow * W + c;
      if (isFloor(level, idx)) hallFloor.add(idx);
    }
    let walk = walkPath(level, hallFloor, new Set(), player, room.start);
    if (walk === null) {
      // Fall back: walk over all floor cells avoiding boxes.
      const blocked = new Set(level.boxes);
      const allFloor = new Set();
      for (let r = 0; r < level.H; r++) {
        for (let c = 0; c < W; c++) {
          const idx = r * W + c;
          if (isFloor(level, idx)) allFloor.add(idx);
        }
      }
      walk = walkPath(level, allFloor, blocked, player, room.start);
    }
    if (walk === null) return '';
    moves += walk;
    player = room.start;
    const res = solveRoom(level, room, !isLast, 1000000);
    if (!res) {
      // Room A* failed: fall back to the whole-level solver, but only for
      // small levels — a big whole-level search would blow the 60s budget.
      if (level.boxes.length > 8) return '';
      const whole = solveWhole(level, 1000000);
      return whole === null ? '' : whole;
    }
    moves += res.moves;
    player = res.player;
  }
  return moves;
}

// ------------------------------------------------------------------- main

function main() {
  const inputDir = process.argv[2] || '/input';
  const outputDir = process.argv[3] || '/output';
  const ids = fs.readdirSync(inputDir).filter((d) => {
    try { return fs.statSync(path.join(inputDir, d)).isDirectory(); } catch { return false; }
  }).sort();
  for (const id of ids) {
    const levelText = fs.readFileSync(path.join(inputDir, id, 'level.txt'), 'utf8');
    const level = parseLevel(levelText);
    const moves = solveLevel(level);
    const outDir = path.join(outputDir, id);
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(path.join(outDir, 'solution.txt'), moves + '\n', 'utf8');
  }
}

main();
