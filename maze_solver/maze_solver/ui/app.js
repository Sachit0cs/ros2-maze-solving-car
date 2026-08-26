/* maze_solver control panel.
 *
 * Draws the maze in CELL coordinates, not metres. The SVG viewBox is
 * 0..cols by 0..rows, so a wall is a unit-length line on an integer lattice
 * line and a cell is a unit square. Every click therefore lands on an integer
 * or a half-integer, and deciding which wall was clicked is arithmetic rather
 * than a hit-test against rendered geometry.
 *
 * y is flipped once, here, in cy(): the maze has row 0 at the BOTTOM (it is a
 * world, and the car drives on it) while SVG has y growing downward. Doing the
 * flip in one function rather than at each use is the difference between a
 * map that reads correctly and an afternoon of upside-down mazes.
 */
const $ = s => document.querySelector(s);
const SVGNS = 'http://www.w3.org/2000/svg';

let M = null;             // the maze on screen
let TOOL = 'off';
let LAST = null;          // last search result (for the animation)
let ANIM = null;
let META = null;          // /api/index payload
let POLL = null;

function el(tag, attrs) {
  const n = document.createElementNS(SVGNS, tag);
  for (const k in attrs) n.setAttribute(k, attrs[k]);
  return n;
}
function cy(r) { return M.rows - r; }          // maze row -> SVG y (see header)
async function post(url, body) {
  const r = await fetch(url, {method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {})});
  return r.json();
}
async function get(url) { return (await fetch(url)).json(); }

/* ------------------------------------------------------------------ drawing */

const TERR = {1: null, 2: 'var(--gravel)', 3: 'var(--mud)'};

function draw(extra) {
  if (!M) return;
  const svg = $('#map');
  svg.setAttribute('viewBox', `-0.25 -0.25 ${M.cols + 0.5} ${M.rows + 0.5}`);
  svg.innerHTML = '';
  const g = el('g', {});
  svg.appendChild(g);

  // terrain first, so everything else sits on top of it
  for (let r = 0; r < M.rows; r++)
    for (let c = 0; c < M.cols; c++) {
      const fill = TERR[Math.round(M.terrain[r][c])];
      if (fill) g.appendChild(el('rect',
        {x: c, y: cy(r) - 1, width: 1, height: 1, fill}));
    }

  // what the car has visited / not yet seen, in discovery mode
  if (extra && extra.known && extra.known.visited)
    for (let r = 0; r < M.rows; r++)
      for (let c = 0; c < M.cols; c++)
        if (!extra.known.visited[r][c])
          g.appendChild(el('rect', {x: c, y: cy(r) - 1, width: 1, height: 1,
            fill: '#000', opacity: 0.42}));

  // expansion order, revealed up to `upto` by the animation
  if (extra && extra.expanded) {
    const upto = extra.upto === undefined ? extra.expanded.length : extra.upto;
    for (let i = 0; i < upto && i < extra.expanded.length; i++) {
      const [c, r] = extra.expanded[i];
      const side = extra.sides && extra.sides[i] ? 1 : 0;
      g.appendChild(el('rect', {x: c + 0.12, y: cy(r) - 0.88,
        width: 0.76, height: 0.76, rx: 0.14,
        fill: side ? '#e879f9' : '#8b5cf6',
        opacity: 0.18 + 0.42 * (i / Math.max(upto, 1))}));
    }
  }

  // the path
  if (extra && extra.path && extra.path.length > 1) {
    const pts = extra.path.map(([c, r]) => `${c + 0.5},${cy(r) - 0.5}`).join(' ');
    g.appendChild(el('polyline', {points: pts, fill: 'none',
      stroke: 'var(--warn)', 'stroke-width': 0.13,
      'stroke-linejoin': 'round', 'stroke-linecap': 'round', opacity: 0.95}));
  }

  drawWalls(g, extra);

  // start and goal
  const [sc, sr] = M.start, [gc, gr] = M.goal;
  g.appendChild(el('circle', {cx: sc + 0.5, cy: cy(sr) - 0.5, r: 0.24,
    fill: 'var(--accent)', opacity: 0.85}));
  g.appendChild(el('circle', {cx: gc + 0.5, cy: cy(gr) - 0.5, r: 0.30,
    fill: 'var(--ok)', opacity: 0.85}));

  // the live car
  if (extra && extra.ego && extra.ego.x !== null) {
    const cx = extra.ego.x / M.pitch + M.cols / 2;
    const cyy = M.rows - (extra.ego.y / M.pitch + M.rows / 2);
    const yaw = -(extra.ego.yaw || 0) * 180 / Math.PI;   // SVG y is flipped
    const car = el('g', {transform: `translate(${cx} ${cyy}) rotate(${yaw})`});
    car.appendChild(el('polygon', {points: '0.30,0 -0.20,0.20 -0.20,-0.20',
      fill: 'var(--bad)'}));
    g.appendChild(car);
  }
  if (TOOL !== 'off') addHitTargets(g);
}

function drawWalls(g, extra) {
  const known = extra && extra.known ? extra.known : null;
  const th = Math.max(0.06, M.wall_thickness / M.pitch);
  const line = (x1, y1, x2, y2, opacity, colour) =>
    g.appendChild(el('line', {x1, y1, x2, y2, stroke: colour || 'var(--wall)',
      'stroke-width': th, 'stroke-linecap': 'square', opacity}));

  // In discovery mode the panel shows the CAR'S map, not the real one: solid
  // where it has committed, faint where it is still guessing. Drawing the true
  // maze here would make a discovery run look like a known-map run and hide
  // the entire point.
  for (let r = 0; r < M.rows; r++)
    for (let c = 0; c <= M.cols; c++) {
      const outer = (c === 0 || c === M.cols);
      let show = outer || !M.passages.h[r][c - 1], op = 1;
      if (known && !outer) {
        const st = known.h[r][c - 1];
        if (st === 0) { show = true; op = 0.10; }        // unknown
        else { show = st === 2; op = 1; }
      }
      if (show) line(c, cy(r) - 1, c, cy(r), op);
    }
  for (let c = 0; c < M.cols; c++)
    for (let r = 0; r <= M.rows; r++) {
      const outer = (r === 0 || r === M.rows);
      let show = outer || !M.passages.v[r - 1][c], op = 1;
      if (known && !outer) {
        const st = known.v[r - 1][c];
        if (st === 0) { show = true; op = 0.10; }
        else { show = st === 2; op = 1; }
      }
      if (show) line(c, cy(r), c + 1, cy(r), op);
    }
}

function addHitTargets(g) {
  // Invisible fat lines/squares so a wall is clickable without being thin.
  const grab = (x1, y1, x2, y2, fn) => {
    const n = el('line', {x1, y1, x2, y2, stroke: 'transparent',
      'stroke-width': 0.34, style: 'cursor:pointer'});
    n.addEventListener('click', ev => { ev.stopPropagation(); fn(); });
    g.appendChild(n);
  };
  if (TOOL === 'wall') {
    for (let r = 0; r < M.rows; r++)
      for (let c = 1; c < M.cols; c++)
        grab(c, cy(r) - 1, c, cy(r), () => edit({action: 'wall',
          a: [c - 1, r], b: [c, r]}));
    for (let c = 0; c < M.cols; c++)
      for (let r = 1; r < M.rows; r++)
        grab(c, cy(r), c + 1, cy(r), () => edit({action: 'wall',
          a: [c, r - 1], b: [c, r]}));
  } else {
    for (let r = 0; r < M.rows; r++)
      for (let c = 0; c < M.cols; c++) {
        const n = el('rect', {x: c, y: cy(r) - 1, width: 1, height: 1,
          fill: 'transparent', style: 'cursor:pointer'});
        n.addEventListener('click', ev => { ev.stopPropagation();
          edit({action: TOOL, c, r}); });
        g.appendChild(n);
      }
  }
}

/* ------------------------------------------------------------------- state */

function setMaze(d, extra) {
  if (d.error) { flash(d.error); return; }
  M = d;
  LAST = null;
  stopAnim();
  $('#lesson').style.display = d.lesson ? '' : 'none';
  if (d.lesson) $('#lesson').textContent = d.lesson;
  const s = d.stats || {};
  $('#mstats').innerHTML = [
    ['cells', s.cells], ['junctions', s.junctions], ['dead ends', s.dead_ends],
    ['slow cells', s.slow_cells],
    ['type', s.perfect ? 'perfect' : 'braided'],
    ['size', s.span_m ? s.span_m[0] + ' x ' + s.span_m[1] + ' m' : '-'],
  ].map(([k, v]) => `<div class="stat"><b>${v}</b><span>${k}</span></div>`).join('');
  draw(extra);
}

async function build() {
  setMaze(await post('/api/maze', {
    cols: +$('#cols').value, rows: +$('#rows').value, seed: +$('#seed').value,
    generator: $('#gen').value, braid: +$('#braid').value,
    rough: +$('#rough').value, corridor: +$('#corridor').value,
    wall_height: +$('#wallh').value,
    wall_thickness: META ? META.defaults.wall_thickness : 0.06,
  }));
  $('#pick').value = '';
}
async function edit(p) { setMaze(await post('/api/edit', p)); }

function flash(msg) {
  const r = $('#result-card');
  r.style.display = '';
  $('#result-title').textContent = 'Problem';
  $('#result').innerHTML = `<div class="err">${msg}</div>`;
}

/* ------------------------------------------------------------- the search */

function stopAnim() { if (ANIM) { cancelAnimationFrame(ANIM); ANIM = null; } }

function animate(res) {
  stopAnim();
  const total = res.expanded.length;
  // ~2.2 s for the whole expansion however big it is, so a 6x6 is not over
  // before you have looked at it and a 25x25 does not take a minute
  const per = Math.max(1, Math.ceil(total / 130));
  let i = 0;
  const step = () => {
    i = Math.min(total, i + per);
    draw({expanded: res.expanded, sides: res.sides, upto: i,
          path: i >= total ? res.path : null});
    if (i < total) ANIM = requestAnimationFrame(step);
    else ANIM = null;
  };
  step();
}

function finish(res) {
  // Draw the completed picture without animating. requestAnimationFrame does
  // not fire in a background tab, so an animation that also had the job of
  // revealing the numbers left them blank for anyone who clicked Solve and
  // then looked away. showResult now runs before the animation starts, and
  // this is the escape hatch that completes the drawing if the animation
  // never gets to run.
  stopAnim();
  draw({expanded: res.expanded, sides: res.sides, upto: res.expanded.length,
        path: res.path});
}

function showResult(res) {
  $('#result-card').style.display = '';
  $('#result-title').textContent = res.label + ' — one run';
  const rows = [
    ['nodes expanded', res.n_expanded],
    ['peak frontier', res.max_frontier],
    ['path length', res.steps + ' cells'],
    ['path cost', res.cost + ' cell-times'],
    ['the car will take', '~' + res.seconds + ' s'],
    ['search took', res.ms + ' ms'],
  ];
  $('#result').innerHTML =
    `<div class="note" style="margin:0 0 8px">${res.blurb}</div>` +
    '<div class="stats">' + rows.map(([k, v]) =>
      `<div class="stat"><b>${v}</b><span>${k}</span></div>`).join('') + '</div>' +
    (res.found ? '' : '<div class="err">No path. ' + (res.note || '') + '</div>') +
    (res.note && res.found ? `<div class="note">${res.note}</div>` : '');
}

async function solve() {
  const res = await post('/api/solve', {algorithm: $('#algo').value});
  if (res.error) { flash(res.error); return; }
  LAST = res;
  showResult(res);            // numbers first - never hostage to the animation
  if (document.hidden) finish(res);
  else animate(res);
}

// If the tab was hidden when Solve was pressed, the animation never started;
// draw the finished picture the moment it comes back rather than leaving an
// empty map.
document.addEventListener('visibilitychange', () => {
  if (!document.hidden && LAST && !ANIM) finish(LAST);
});

async function compareAll() {
  const t = await post('/api/compare', {});
  if (t.error) { flash(t.error); return; }
  $('#result-card').style.display = '';
  $('#result-title').textContent = 'Every algorithm on this maze';
  const sel = $('#algo').value;
  const head = ['algorithm', 'expanded', 'peak', 'cells', 'cost', 'seconds',
                'ms', 'optimal'];
  const body = t.rows.map(r => {
    const cls = [r.name === sel ? 'sel' : '', r.optimal ? 'best' : ''].join(' ');
    return `<tr class="${cls}"><td title="${r.blurb}">${r.label}</td>` +
      `<td>${r.n_expanded}</td><td>${r.max_frontier}</td>` +
      `<td>${r.found ? r.steps : '—'}</td><td>${r.found ? r.cost : '—'}</td>` +
      `<td>${r.found ? r.seconds : '—'}</td><td>${r.ms}</td>` +
      `<td class="${r.optimal ? 'yes' : 'no'}">${r.optimal ? 'yes'
        : (r.found ? '+' + r.excess : 'no path')}</td></tr>`;
  }).join('');
  $('#result').innerHTML =
    `<table><thead><tr>${head.map(h => `<th>${h}</th>`).join('')}</tr></thead>` +
    `<tbody>${body}</tbody></table>` +
    `<div class="note">Optimal means "same cost as uniform-cost search", which ` +
    `is ${t.best_cost} here. The "+" column is how much worse. Green rows are ` +
    `optimal; hover an algorithm for what it promises.</div>`;
}

/* ---------------------------------------------------------------- the sim */

async function start() {
  stopAnim();
  const r = await post('/api/start', {
    algorithm: $('#algo').value, mode: $('#mode').value,
    gui: $('#gui').checked, rviz: $('#rviz').checked});
  if (!r.ok) flash(r.message);
  else { $('#log').textContent = 'starting Gazebo — first scan in ~30 s…'; }
}
async function stop() { await post('/api/stop', {}); }

async function poll() {
  let st;
  try { st = await get('/api/status'); } catch (e) {
    $('#conn').textContent = 'offline'; $('#conn').className = 'pill off'; return;
  }
  $('#conn').textContent = st.running ? 'simulation running' : 'idle';
  $('#conn').className = 'pill ' + (st.running ? 'on' : 'off');
  if (st.lines && st.lines.length) $('#log').textContent = st.lines.join('\n');

  // While a run is live the map shows the run, not the last static search.
  if (st.running && (st.ego || st.plan)) {
    const extra = {ego: st.ego};
    if (st.plan && st.plan.path) extra.path = st.plan.path;
    if (st.map) extra.known = st.map;
    if (!ANIM) draw(extra);
    const p = st.plan || {};
    const e = st.ego || {};
    $('#result-card').style.display = '';
    $('#result-title').textContent = 'Live run';
    $('#result').innerHTML = '<div class="stats">' + [
      ['progress', (st.progress === null ? '—' : st.progress + '%')],
      ['cells to goal', e.cells_to_goal === undefined ? '—' : e.cells_to_goal],
      ['elapsed', (e.elapsed || 0) + ' s'],
      ['plans run', p.replans || 0],
      ['nodes expanded', p.total_expanded || p.n_expanded || 0],
      ['mapped', st.map ? Math.round(st.map.coverage * 100) + '%' : '—'],
      ['solved', st.goal], ['hit a wall', st.wall], ['gave up', st.stuck],
    ].map(([k, v]) => `<div class="stat"><b>${v}</b><span>${k}</span></div>`)
      .join('') + '</div>';
  }
}

/* -------------------------------------------------------------------- init */

async function init() {
  META = await get('/api/index');
  $('#gen').innerHTML = META.generators.map(g =>
    `<option value="${g}">${g === 'prim' ? "Prim's — bushy, many dead ends"
      : 'Recursive backtracker — long corridors'}</option>`).join('');
  $('#algo').innerHTML = META.algorithms.map(a =>
    `<option value="${a.name}">${a.label}</option>`).join('');
  $('#pick').innerHTML = '<option value="">— build one below —</option>' +
    META.mazes.map(m => `<option value="${m.name}">${m.name} (${m.cols}x${m.rows}` +
      `${m.perfect ? '' : ', braided'}${m.slow ? ', mud' : ''})</option>`).join('');
  const blurb = () => {
    const a = META.algorithms.find(x => x.name === $('#algo').value);
    $('#algo-blurb').textContent = a ? a.blurb : '';
    $('#mode').disabled = ($('#algo').value === 'wall_follower');
    if ($('#algo').value === 'wall_follower')
      $('#algo-blurb').textContent += ' — it has no map, so the mode above does not apply.';
  };
  $('#algo').addEventListener('change', blurb);
  blurb();

  for (const [id, out] of [['braid', 'braidV'], ['rough', 'roughV']]) {
    const show = () => $('#' + out).textContent = (+$('#' + id).value).toFixed(2);
    $('#' + id).addEventListener('input', show);
    // rebuild when the slider is RELEASED, not while it is moving - dragging
    // roughness across its range would otherwise regenerate and rewrite the
    // .sdf twenty times a second
    $('#' + id).addEventListener('change', () => { show(); build(); });
    show();
  }

  $('#gen-btn').addEventListener('click', build);
  $('#rnd').addEventListener('click', () => {
    $('#seed').value = Math.floor(Math.random() * 99999); build();
  });
  $('#pick').addEventListener('change', async e => {
    if (!e.target.value) return;
    setMaze(await get('/api/maze_view?name=' + encodeURIComponent(e.target.value)));
  });
  document.querySelectorAll('[data-tool]').forEach(b =>
    b.addEventListener('click', () => {
      TOOL = b.dataset.tool;
      document.querySelectorAll('[data-tool]').forEach(x =>
        x.classList.toggle('on', x === b));
      $('#tool-hint').textContent = {
        off: 'Pick a tool, then click the map.',
        wall: 'Click a wall to knock it through, or click a gap to build one.',
        terrain: 'Click a cell to cycle plain → gravel x2 → mud x3.',
        start: 'Click a cell to move the start there.',
        goal: 'Click a cell to move the goal. Put it in the middle of a braided ' +
              'maze and the wall follower will never find it.',
      }[TOOL];
      draw(LAST ? {expanded: LAST.expanded, path: LAST.path,
                   sides: LAST.sides} : null);
    }));
  $('#solve').addEventListener('click', solve);
  $('#cmp').addEventListener('click', compareAll);
  $('#run').addEventListener('click', start);
  $('#stop').addEventListener('click', stop);

  setMaze(await post('/api/maze', {
    cols: +$('#cols').value, rows: +$('#rows').value, seed: +$('#seed').value,
    generator: 'backtracker', braid: 0, rough: 0,
    corridor: +$('#corridor').value, wall_height: +$('#wallh').value,
    wall_thickness: META.defaults.wall_thickness}));

  POLL = setInterval(poll, 700);
  poll();
}
init();
