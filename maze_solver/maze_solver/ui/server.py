#!/usr/bin/env python3
"""Local control panel for maze_solver.

Runs inside WSL, reached from Chrome on Windows at http://localhost:8090
(WSL2 forwards localhost by default). Standard library only - no Flask, no npm,
nothing to install.

    python3 -m maze_solver.ui.server

8090, not 8088 or 8089: road_follower and traffic_dodger already hold those,
and you will want more than one panel open sooner or later.

THREE THINGS THIS PANEL DOES THAT THE OTHER PROJECTS' PANELS DID NOT

1. It EDITS the world. Click a wall to knock it through, drag the goal, paint
   mud. The maze is regenerated and rewritten on every edit, which costs a few
   milliseconds, and means the .sdf on disk always matches what is on screen.

2. It runs the search WITHOUT the simulator. /api/solve and /api/compare import
   search.py directly and answer in single-digit milliseconds. A student can
   try all seven algorithms on twenty mazes before ever starting Gazebo - and
   should, because that is where the algorithm lesson is. The simulator is
   where the ROBOT lesson is, and it costs a hundred thousand times as much
   per answer.

3. It animates the frontier. The expansion ORDER comes back with every search,
   so the panel can replay the search growing outward. That picture is the
   single most useful thing here; it is what makes 'greedy dives at the goal'
   and 'Dijkstra grows a circle' facts a student has seen rather than been
   told.

HOW IT SEES A RUNNING SIMULATION

Through files, not topics - this is a plain http.server, not a ROS node. The
nodes drop state in /tmp/ms_ego.json, /tmp/ms_plan.json and /tmp/ms_map.json,
each rewritten atomically so a read never catches half a file. That is what
lets the panel animate the car with the Gazebo window closed, which on a
machine with no GPU is the difference between a run you can watch and a run you
can afford to do at all.
"""
import json
import os
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from maze_solver.maze import DEFAULTS, GENERATORS, TERRAIN, Maze
from maze_solver.maze_gen import write_maze
from maze_solver.search import ALGORITHMS, ORDER, compare, run

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser('~/maze_solver_ws')
MAZES = os.path.join(WS, 'mazes')

# ROS_DOMAIN_ID=42 is not decoration - see scripts/env.sh. Without it a
# traffic_dodger node left running in another terminal publishes
# /episode/active on the same graph and this project's planner treats it as an
# episode restart.
SETUP = ('source /opt/ros/jazzy/setup.bash && '
         'source %s/install/setup.bash && '
         'export ROS_DOMAIN_ID=42 && '
         'export LIBGL_ALWAYS_SOFTWARE=1 && ' % WS)

LOG_SIM = '/tmp/ms_ui_sim.log'
LOG_DRV = '/tmp/ms_ui_drv.log'
EGO_STATE = '/tmp/ms_ego.json'
PLAN_STATE = '/tmp/ms_plan.json'
MAP_STATE = '/tmp/ms_map.json'

UI_NAME = 'maze_ui'          # the maze the panel edits


def sh(cmd, wait=True, log=None):
    full = SETUP + cmd
    if log:
        with open(log, 'wb') as f:
            p = subprocess.Popen(['bash', '-c', full], stdout=f,
                                 stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    else:
        p = subprocess.Popen(['bash', '-c', full], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    if wait:
        out, _ = p.communicate()
        return p.returncode, (out or b'').decode(errors='replace')
    return p, ''


def kill_all():
    """Scoped teardown - the same patterns scripts/kill_sim.sh uses.

    Deliberately does NOT pkill 'gz sim' or 'parameter_bridge' by name: those
    would take down whatever road_follower or traffic_dodger is running on this
    machine, from a button labelled Stop in a maze panel.
    """
    for pat in ('ros2 launch maze_solver',
                'install/maze_solver/lib/maze_solver',
                'maze_solver_ws/mazes'):
        subprocess.run(['pkill', '-f', pat], capture_output=True)
    for f in (EGO_STATE, PLAN_STATE, MAP_STATE):
        try:
            os.remove(f)
        except OSError:
            pass
    time.sleep(2)


def read_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def clean(line):
    m = re.search(r'\]: (.*)$', line)
    return (m.group(1) if m else line)[:170]


# --------------------------------------------------------------- the maze

class Editor:
    """Holds the maze the panel is showing, and writes it out on every change."""

    def __init__(self):
        self.lock = threading.Lock()
        self.maze = None
        self.name = UI_NAME

    def build(self, p):
        kw = {k: p.get(k, DEFAULTS[k]) for k in DEFAULTS}
        kw['cols'] = int(kw['cols'])
        kw['rows'] = int(kw['rows'])
        kw['seed'] = int(kw['seed'])
        for k in ('braid', 'rough', 'corridor', 'wall_thickness', 'wall_height'):
            kw[k] = float(kw[k])
        if kw['generator'] not in GENERATORS:
            kw['generator'] = 'backtracker'
        with self.lock:
            self.maze = Maze(**kw)
            self.name = UI_NAME
            return self._write()

    def load(self, name):
        meta = read_json(os.path.join(MAZES, name + '.json'))
        if not meta:
            return {'error': 'no maze called %s' % name}
        with self.lock:
            self.maze = Maze.from_meta(meta)
            self.name = name
            # Loading a PRESET must not rewrite it. Opening maze_classic to
            # look at it and silently overwriting it with whatever the sliders
            # happened to say is exactly the bug traffic_dodger's panel had.
            d = self._draw()
            d['lesson'] = meta.get('lesson', '')
            d['read_only'] = name != UI_NAME
            return d

    def edit(self, p):
        with self.lock:
            m = self.maze
            if m is None:
                return {'error': 'no maze loaded'}
            act = p.get('action')
            if act == 'wall':
                a, b = tuple(p['a']), tuple(p['b'])
                if b in m._grid_neighbours(*a):
                    now = b in m.neighbours(a)
                    m._open(a[0], a[1], b[0], b[1]) if not now else self._close(m, a, b)
            elif act == 'terrain':
                c, r = int(p['c']), int(p['r'])
                tiers = [t[1] for t in TERRAIN]
                cur = m.terrain[r][c]
                nxt = tiers[(tiers.index(cur) + 1) % len(tiers)] if cur in tiers else 1.0
                m.terrain[r][c] = nxt
            elif act in ('start', 'goal'):
                cell = (int(p['c']), int(p['r']))
                if m.in_bounds(cell):
                    setattr(m, act, cell)
            else:
                return {'error': 'unknown edit %r' % act}
            # An edit makes this a hand-made maze; it is no longer the preset
            # it may have been loaded from, so it is saved under the UI name.
            self.name = UI_NAME
            return self._write()

    @staticmethod
    def _close(m, a, b):
        if a[1] == b[1]:
            m.open_h[a[1]][min(a[0], b[0])] = False
        else:
            m.open_v[min(a[1], b[1])][a[0]] = False

    def _write(self):
        os.makedirs(MAZES, exist_ok=True)
        write_maze(os.path.join(MAZES, self.name + '.sdf'), maze=self.maze)
        return self._draw()

    def _draw(self):
        m = self.maze
        return {'name': self.name, 'cols': m.cols, 'rows': m.rows,
                'pitch': m.pitch, 'corridor': m.corridor,
                'wall_thickness': m.wall_thickness,
                'passages': m.passage_bits(), 'terrain': m.terrain,
                'start': list(m.start), 'goal': list(m.goal),
                'seed': m.seed, 'generator': m.generator,
                'braid': m.braid, 'rough': m.rough,
                'stats': m.stats(), 'read_only': False}

    def snapshot(self):
        with self.lock:
            return self.maze, self.name


ED = Editor()


# ------------------------------------------------------------- the simulator

class Sim:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.cfg = {}
        self.started_at = None

    def start(self, cfg):
        maze, name = ED.snapshot()
        if maze is None:
            return False, 'generate a maze first'
        kill_all()
        world = os.path.join(MAZES, name + '.sdf')
        meta = os.path.join(MAZES, name + '.json')
        if not os.path.exists(world):
            return False, 'maze %s has not been written yet' % name
        for f in (LOG_SIM, LOG_DRV):
            try:
                os.remove(f)
            except OSError:
                pass

        gui = 'true' if cfg.get('gui') else 'false'
        rviz = 'true' if cfg.get('rviz') else 'false'
        sh('ros2 launch maze_solver maze_sim.launch.py world:=%s gui:=%s rviz:=%s'
           % (world, gui, rviz), wait=False, log=LOG_SIM)

        algo = cfg.get('algorithm', 'astar')
        driver = 'wall' if algo == 'wall_follower' else 'plan'
        mode = cfg.get('mode', 'known')

        def later():
            for _ in range(90):
                rc, out = sh("ros2 topic list 2>/dev/null | grep -cx '/scan'")
                if out.strip().isdigit() and int(out.strip()) > 0:
                    break
                time.sleep(2)
            time.sleep(5)
            sh('ros2 launch maze_solver solve.launch.py meta:=%s '
               'algorithm:=%s mode:=%s driver:=%s corridor:=%.3f'
               % (meta, algo, mode, driver, maze.corridor),
               wait=False, log=LOG_DRV)

        threading.Thread(target=later, daemon=True).start()
        with self.lock:
            self.running = True
            self.cfg = cfg
            self.started_at = time.time()
        return True, 'starting'

    def stop(self):
        kill_all()
        with self.lock:
            self.running = False
            self.started_at = None
        return True, 'stopped'

    def status(self):
        with self.lock:
            st = {'running': self.running, 'config': dict(self.cfg),
                  'elapsed': int(time.time() - self.started_at)
                  if self.started_at else 0}
        st.update({'ego': None, 'plan': None, 'map': None, 'lines': [],
                   'goal': 0, 'wall': 0, 'stuck': 0, 'progress': None})

        ego = read_json(EGO_STATE)
        if ego:
            st['ego'] = ego
            st['progress'] = (round(ego['progress']) if ego.get('progress')
                              is not None else None)
            st['goal'] = ego.get('goal_n', 0)
            st['wall'] = ego.get('wall_n', 0)
            st['stuck'] = ego.get('stuck_n', 0)
        plan = read_json(PLAN_STATE)
        if plan:
            # the expansion list can be a couple of thousand pairs; the live
            # poll does not need it, only the replay does
            st['plan'] = {k: v for k, v in plan.items()
                          if k not in ('expanded', 'sides', 'known')}
            st['plan']['path'] = plan.get('path', [])
        mp = read_json(MAP_STATE)
        if mp:
            st['map'] = mp

        try:
            with open(LOG_DRV, errors='replace') as f:
                txt = f.read()
            keep = [l for l in txt.splitlines()
                    if re.search(r'episode|progress|replan|planner up|mapper up|'
                                 r'path_driver up|wall_follower up|holding|'
                                 r'scans,|no path', l)]
            st['lines'] = [clean(l) for l in keep[-16:]]
        except FileNotFoundError:
            pass
        total = st['goal'] + st['wall'] + st['stuck']
        st['episodes'] = total
        return st


SIM = Sim()


# ------------------------------------------------------------------- search

def solve(p):
    """Run ONE algorithm on the maze on screen. No simulator, no ROS."""
    maze, _ = ED.snapshot()
    if maze is None:
        return {'error': 'generate a maze first'}
    name = p.get('algorithm', 'astar')
    if name not in ALGORITHMS:
        return {'error': 'no algorithm called %r' % name}
    res = run(name, maze, maze.start, maze.goal)
    d = res.as_dict()
    d['label'] = ALGORITHMS[name][0]
    d['blurb'] = ALGORITHMS[name][2]
    # seconds, not cell-times: cost is in units of 'one plain cell at full
    # speed', and path_driver drives a plain cell at v_max. Quoting both is
    # what makes the cost model legible rather than abstract.
    d['seconds'] = round(res.cost * maze.pitch / 0.40, 1)
    return d


def compare_all(p):
    maze, _ = ED.snapshot()
    if maze is None:
        return {'error': 'generate a maze first'}
    tbl = compare(maze, maze.start, maze.goal)
    for row in tbl['rows']:
        row['seconds'] = round(row['cost'] * maze.pitch / 0.40, 1)
        row.pop('expanded', None)          # the table does not replay
        row.pop('sides', None)
    tbl['order'] = ORDER
    return tbl


def list_mazes():
    out = []
    if os.path.isdir(MAZES):
        for f in sorted(os.listdir(MAZES)):
            if f.endswith('.json') and f != 'index.json':
                m = read_json(os.path.join(MAZES, f))
                if not m:
                    continue
                st = m.get('stats', {})
                out.append({'name': f[:-5], 'cols': m.get('cols'),
                            'rows': m.get('rows'),
                            'perfect': st.get('perfect'),
                            'slow': st.get('slow_cells', 0),
                            'lesson': m.get('lesson', ''),
                            'preset': m.get('preset', '')})
    return out


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype='application/json'):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _file(self, name, ctype):
        try:
            with open(os.path.join(HERE, name), 'rb') as f:
                return self._send(200, f.read().decode(), ctype)
        except FileNotFoundError:
            return self._send(500, '%s missing' % name, 'text/plain')

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            return self._file('index.html', 'text/html; charset=utf-8')
        if self.path == '/app.js':
            return self._file('app.js', 'application/javascript')
        if self.path == '/api/status':
            return self._send(200, SIM.status())
        if self.path == '/api/replay':
            # the full expansion list, fetched once per plan rather than at
            # poll rate
            return self._send(200, read_json(PLAN_STATE, {}))
        if self.path.startswith('/api/maze_view'):
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            name = (q.get('name') or [''])[0]
            # only ever a maze that exists here - never a path from the client
            if not re.fullmatch(r'[A-Za-z0-9_]{1,40}', name):
                return self._send(400, {'error': 'bad maze name'})
            return self._send(200, ED.load(name))
        if self.path == '/api/index':
            return self._send(200, {
                'mazes': list_mazes(),
                'defaults': DEFAULTS,
                'generators': list(GENERATORS),
                'terrain': [{'name': t[0], 'cost': t[1]} for t in TERRAIN],
                'algorithms': [{'name': n, 'label': ALGORITHMS[n][0],
                                'blurb': ALGORITHMS[n][2]} for n in ORDER],
            })
        return self._send(404, {'error': 'not found'})

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            p = json.loads(self.rfile.read(n) or b'{}')
        except ValueError:
            return self._send(400, {'error': 'bad json'})
        try:
            if self.path == '/api/maze':
                return self._send(200, ED.build(p))
            if self.path == '/api/edit':
                return self._send(200, ED.edit(p))
            if self.path == '/api/solve':
                return self._send(200, solve(p))
            if self.path == '/api/compare':
                return self._send(200, compare_all(p))
            if self.path == '/api/start':
                ok, msg = SIM.start(p)
                return self._send(200 if ok else 400, {'ok': ok, 'message': msg})
            if self.path == '/api/stop':
                ok, msg = SIM.stop()
                return self._send(200, {'ok': ok, 'message': msg})
        except Exception as e:                        # noqa: BLE001
            return self._send(500, {'error': '%s: %s' % (type(e).__name__, e)})
        return self._send(404, {'error': 'not found'})


def main():
    port = int(os.environ.get('UI_PORT', '8090'))
    # Start on the default maze so the panel is never empty on first load.
    ED.build(dict(DEFAULTS))
    srv = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    print('')
    print('  maze_solver control panel')
    print('  open this in Chrome on Windows:   http://localhost:%d' % port)
    print('  Ctrl-C here to stop the server')
    print('')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\nshutting down, stopping any running simulation ...')
        kill_all()


if __name__ == '__main__':
    main()
