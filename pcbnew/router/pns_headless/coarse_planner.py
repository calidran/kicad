#!/usr/bin/env python3
"""
coarse_planner.py — global coarse waypoint planner for the headless PNS driver.

The naive point-to-point pns-route stalls its head against obstacles it cannot
shove past in a straight line. This planner supplies a rough waypoint polyline
from the source pad to the target pad on a given copper layer, which pns-route
then follows hop-by-hop while PNS does the local push-and-shove.

THE CRITICAL RULE (the whole point of the spike):
  * FIXED copper is an OBSTACLE — vias (blind escape + foreign through-vias) and
    pads. These PNS cannot move, so the coarse path must go around them.
  * MOVEABLE traces are PASSABLE (low cost). The greedy routers failed precisely
    because they treated traces as walls; PNS will shove them, so the planner
    routes THROUGH them.

Method: grid A* over the target layer. Fixed-copper cells (vias/pads inflated by
clearance) are blocked; everything else is free, with a mild penalty for cells
that overlap an existing trace of a *different* net (shoving has a cost but is
allowed). Emits waypoints as "x,y;x,y;..." in mm for pns-route --waypoints.

Board is read directly from the .kicad_pcb sexpr (KiCad 9 old format: numeric
nets, footprint-relative pad positions). No KiCad libs required.
"""

import argparse
import heapq
import math
import re
import sys


# --------------------------------------------------------------------------
# sexpr helpers
# --------------------------------------------------------------------------
def brace_blocks(txt, tag):
    """Yield each top-level-ish (tag ...) block by brace matching."""
    for m in re.finditer(r'\(' + tag + r'\b', txt):
        s = m.start()
        depth = 0
        j = s
        while j < len(txt):
            c = txt[j]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    yield txt[s:j + 1]
                    break
            j += 1


def net_name_map(txt):
    return dict((int(n), name) for n, name in re.findall(r'\(net (\d+) "([^"]*)"', txt))


def item_net_name(block, num2name):
    """Net name of an item block, handling both sexpr formats:
       old: (net 46)  -> look up in num2name
       new: (net "/SEMC_A4")  -> name directly
    """
    m = re.search(r'\(net "([^"]*)"\)', block)
    if m:
        return m.group(1)
    m = re.search(r'\(net (\d+)\)', block)
    if m:
        return num2name.get(int(m.group(1)), "")
    return ""


# --------------------------------------------------------------------------
# geometry extraction (mm, absolute board coords)
# --------------------------------------------------------------------------
def rot_xy(px, py, rot_deg):
    a = math.radians(-rot_deg)
    return (px * math.cos(a) - py * math.sin(a),
            px * math.sin(a) + py * math.cos(a))


def extract(txt, layer, own_net=None):
    """Return (fixed_obstacles, trace_cells, pads) for the given copper layer.

    fixed_obstacles: list of (x, y, radius_mm) circles (vias + pads on/through layer)
    trace_segments:  list of (x1, y1, x2, y2, width, netname) on this layer
    pads:            dict "REF.PAD" -> (x, y)

    own_net: the net being routed; its OWN vias/pads are not obstacles to itself
    (they are the endpoints / escape vias we route between).
    """
    nets = net_name_map(txt)
    fixed = []
    traces = []
    pads = {}

    # Copper-layer stack order (KiCad physical order F -> In1..In8 -> B).
    stack = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu',
             'In5.Cu', 'In6.Cu', 'In7.Cu', 'In8.Cu', 'B.Cu']
    li = stack.index(layer) if layer in stack else -1

    def span_covers(l1, l2):
        # does the via's [l1..l2] span include the target layer?
        if l1 not in stack or l2 not in stack or li < 0:
            return True  # be conservative if unknown
        a, b = sorted((stack.index(l1), stack.index(l2)))
        return a <= li <= b

    # Vias are fixed obstacles ONLY on the layers their span actually reaches.
    # A blind via that stops above In2 does not block In2.
    for v in brace_blocks(txt, 'via'):
        at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', v)
        size = re.search(r'\(size ([\-\d.]+)', v)
        lyr = re.search(r'\(layers "([^"]+)" "([^"]+)"\)', v)
        if not at or not size:
            continue
        if lyr and not span_covers(lyr.group(1), lyr.group(2)):
            continue
        if own_net and item_net_name(v, nets) == own_net:
            continue  # the net's own escape vias are endpoints, not obstacles
        x, y = float(at.group(1)), float(at.group(2))
        r = float(size.group(1)) / 2.0
        fixed.append((x, y, r))

    # Footprints: pads. SMD pads only sit on their own outer layer; through-hole
    # / NPTH pads pass through all copper. For an inner layer (In2) the relevant
    # fixed pad copper is through-hole pads + any pad explicitly on this layer.
    for fp in brace_blocks(txt, 'footprint'):
        refm = re.search(r'\(property "Reference" "([^"]+)"', fp)
        if not refm:
            continue
        ref = refm.group(1)

        # Footprint origin: old format uses a top-level (at x y rot); the newer
        # (20260624) format uses (transform (translate x y)(rotate r)). Support
        # both — the (at ...) regex must NOT accidentally match a pad's own (at).
        trm = re.search(r'\(transform\s*\(translate ([\-\d.]+) ([\-\d.]+)\)\s*\(rotate ([\-\d.]+)\)', fp)
        if trm:
            fx, fy, frot = float(trm.group(1)), float(trm.group(2)), float(trm.group(3))
        else:
            # take the FIRST (at ...) that is a direct child of the footprint,
            # i.e. appears before the first (pad ...)
            first_pad = fp.find('(pad ')
            head = fp[:first_pad] if first_pad >= 0 else fp
            atm = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)(?: ([\-\d.]+))?\)', head)
            if not atm:
                continue
            fx, fy = float(atm.group(1)), float(atm.group(2))
            frot = float(atm.group(3) or 0)

        for pm in re.finditer(
                r'\(pad "([^"]+)"\s+(\S+)\s+(\S+)[\s\S]*?\(at ([\-\d.]+) ([\-\d.]+)(?: ([\-\d.]+))?\)'
                r'[\s\S]*?\(size ([\-\d.]+) ([\-\d.]+)\)'
                r'([\s\S]*?)(?=\(pad "|\Z)', fp):
            name, ptype = pm.group(1), pm.group(2)
            prx, pry = float(pm.group(4)), float(pm.group(5))
            sx, sy = float(pm.group(7)), float(pm.group(8))
            tail = pm.group(9)
            ox, oy = rot_xy(prx, pry, frot)
            ax, ay = fx + ox, fy + oy
            pads[f"{ref}.{name}"] = (ax, ay)

            # through-hole / thru pads are fixed copper on inner layers
            layers = re.search(r'\(layers ([^\)]*)\)', tail)
            lstr = layers.group(1) if layers else ""
            through = (ptype in ('thru_hole', 'np_thru_hole')) or '"*.Cu"' in lstr
            on_layer = f'"{layer}"' in lstr
            if through or on_layer:
                r = max(sx, sy) / 2.0
                fixed.append((ax, ay, r))

    # Segments (moveable traces) on this layer — PASSABLE, mild cost.
    for seg in brace_blocks(txt, 'segment'):
        if f'(layer "{layer}")' not in seg:
            continue
        s = re.search(r'\(start ([\-\d.]+) ([\-\d.]+)\)', seg)
        e = re.search(r'\(end ([\-\d.]+) ([\-\d.]+)\)', seg)
        w = re.search(r'\(width ([\-\d.]+)\)', seg)
        if not (s and e):
            continue
        traces.append((float(s.group(1)), float(s.group(2)),
                       float(e.group(1)), float(e.group(2)),
                       float(w.group(1)) if w else 0.1,
                       item_net_name(seg, nets)))

    return fixed, traces, pads


# --------------------------------------------------------------------------
# grid A*
# --------------------------------------------------------------------------
def plan(fixed, traces, start, goal, net, cell_mm, clearance_mm, margin_mm,
         dp_mult=1.5, max_hop_mm=0.6):
    xs = [start[0], goal[0]] + [o[0] for o in fixed]
    ys = [start[1], goal[1]] + [o[1] for o in fixed]
    minx, maxx = min(xs) - margin_mm, max(xs) + margin_mm
    miny, maxy = min(ys) - margin_mm, max(ys) + margin_mm

    nx = max(2, int((maxx - minx) / cell_mm) + 1)
    ny = max(2, int((maxy - miny) / cell_mm) + 1)

    def to_cell(x, y):
        return (min(nx - 1, max(0, int((x - minx) / cell_mm))),
                min(ny - 1, max(0, int((y - miny) / cell_mm))))

    def to_xy(cx, cy):
        return (minx + (cx + 0.5) * cell_mm, miny + (cy + 0.5) * cell_mm)

    # blocked grid from fixed obstacles (inflated by clearance + half cell)
    blocked = [[False] * ny for _ in range(nx)]
    for (ox, oy, orad) in fixed:
        rr = orad + clearance_mm + cell_mm * 0.5
        cx0, cy0 = to_cell(ox - rr, oy - rr)
        cx1, cy1 = to_cell(ox + rr, oy + rr)
        for cx in range(cx0, cx1 + 1):
            for cy in range(cy0, cy1 + 1):
                px, py = to_xy(cx, cy)
                if (px - ox) ** 2 + (py - oy) ** 2 <= rr * rr:
                    blocked[cx][cy] = True

    # trace-overlap cost map (shoving a foreign-net trace costs, but is allowed)
    trace_cost = [[0.0] * ny for _ in range(nx)]
    for (x1, y1, x2, y2, w, tnet) in traces:
        if tnet == net:
            continue  # our own copper is free
        steps = max(1, int(math.hypot(x2 - x1, y2 - y1) / (cell_mm * 0.5)))
        for k in range(steps + 1):
            t = k / steps
            px, py = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
            cx, cy = to_cell(px, py)
            trace_cost[cx][cy] += 1.0

    # ensure start/goal cells are usable even if inside inflated pad copper
    sc, gc = to_cell(*start), to_cell(*goal)
    blocked[sc[0]][sc[1]] = False
    blocked[gc[0]][gc[1]] = False

    # A* (8-connected)
    def h(c):
        return math.hypot(c[0] - gc[0], c[1] - gc[1])

    openq = [(h(sc), 0.0, sc)]
    came = {}
    gscore = {sc: 0.0}
    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while openq:
        _, g, cur = heapq.heappop(openq)
        if cur == gc:
            break
        if g > gscore.get(cur, 1e18):
            continue
        for dx, dy in nbrs:
            ncx, ncy = cur[0] + dx, cur[1] + dy
            if not (0 <= ncx < nx and 0 <= ncy < ny):
                continue
            if blocked[ncx][ncy]:
                continue
            step = math.hypot(dx, dy)
            # shove penalty: crossing foreign trace copper is allowed but costs
            step += trace_cost[ncx][ncy] * 0.6
            ng = g + step
            if ng < gscore.get((ncx, ncy), 1e18):
                gscore[(ncx, ncy)] = ng
                came[(ncx, ncy)] = cur
                heapq.heappush(openq, (ng + h((ncx, ncy)), ng, (ncx, ncy)))

    if gc not in came and gc != sc:
        return None  # no path even through shoveable copper -> fixed-geometry boxed

    # reconstruct + simplify (collinear removal / Douglas-Peucker-lite)
    path = [gc]
    while path[-1] != sc:
        path.append(came[path[-1]])
    path.reverse()
    pts = [to_xy(cx, cy) for cx, cy in path]

    # Douglas-Peucker simplify so PNS gets a few clean steering corners rather
    # than dozens of grid-jitter micro-hops.
    def dp(points, eps):
        if len(points) < 3:
            return points[:]
        ax, ay = points[0]
        bx, by = points[-1]
        dmax, idx = 0.0, 0
        L = math.hypot(bx - ax, by - ay) or 1e-9
        for i in range(1, len(points) - 1):
            px, py = points[i]
            d = abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / L
            if d > dmax:
                dmax, idx = d, i
        if dmax > eps:
            left = dp(points[:idx + 1], eps)
            right = dp(points[idx:], eps)
            return left[:-1] + right
        return [points[0], points[-1]]

    simp = dp(pts, eps=cell_mm * dp_mult)

    # Resample so no hop exceeds max_hop_mm — PNS shoves one short segment per
    # hop; a hop that is too long stalls the head partway against copper it
    # can't shove in a single straight push.
    dense = [simp[0]]
    for i in range(1, len(simp)):
        ax, ay = dense[-1]
        bx, by = simp[i]
        d = math.hypot(bx - ax, by - ay)
        n = max(1, int(math.ceil(d / max_hop_mm)))
        for k in range(1, n + 1):
            t = k / n
            dense.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return dense


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('--from', dest='src', required=True, help='REF.PAD')
    ap.add_argument('--to', dest='dst', required=True, help='REF.PAD')
    ap.add_argument('--net', required=True)
    ap.add_argument('--layer', default='In2.Cu')
    ap.add_argument('--cell-mm', type=float, default=0.2)
    ap.add_argument('--clearance-mm', type=float, default=0.15)
    ap.add_argument('--margin-mm', type=float, default=1.0)
    ap.add_argument('--dp-mult', type=float, default=1.5,
                    help='Douglas-Peucker epsilon = cell_mm * dp_mult (lower = more corners)')
    ap.add_argument('--max-hop-mm', type=float, default=0.6,
                    help='resample so no waypoint hop is longer than this')
    ap.add_argument('--emit', choices=['waypoints', 'debug'], default='waypoints')
    args = ap.parse_args()

    txt = open(args.board).read()
    fixed, traces, pads = extract(txt, args.layer, own_net=args.net)

    if args.src not in pads or args.dst not in pads:
        sys.stderr.write(f"coarse_planner: pad not found ({args.src} / {args.dst})\n")
        sys.exit(1)

    start, goal = pads[args.src], pads[args.dst]
    path = plan(fixed, traces, start, goal, args.net,
                args.cell_mm, args.clearance_mm, args.margin_mm,
                dp_mult=args.dp_mult, max_hop_mm=args.max_hop_mm)

    if path is None:
        sys.stderr.write("coarse_planner: NO PATH — boxed by fixed geometry "
                         "(vias/pads) even treating traces as passable\n")
        sys.exit(2)

    # inner waypoints only (drop the start and goal pads; pns-route adds target)
    inner = path[1:-1]
    if args.emit == 'debug':
        sys.stderr.write(f"start={start} goal={goal} fixed={len(fixed)} "
                         f"traces={len(traces)} pathpts={len(path)} inner_wps={len(inner)}\n")
    print(";".join(f"{x:.4f},{y:.4f}" for x, y in inner))


if __name__ == '__main__':
    main()
