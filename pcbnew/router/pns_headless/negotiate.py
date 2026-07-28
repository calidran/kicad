#!/usr/bin/env python3
"""
negotiate.py — PathFinder-style negotiated-congestion orchestration on top of
the two-layer headless PNS driver (coarse_planner.py + pns-route).

The v2 driver closed 2/10 walled nets; the other 8 wedge at mid-corridor
congestion locks where PNS cannot displace enough neighbours in one pass.
This layer negotiates:

  1. RIP-UP-AND-RETRY: when a net wedges, rip the blocking already-routed
     sibling(s) (delete their copper on the layer), re-route the wedged net
     through the freed corridor, then re-route the ripped ones.
  2. CONGESTION PRICING (the PathFinder core): every wedge deposits a history
     penalty circle at the stall point; the coarse planner prices those cells
     up so later iterations route AROUND chronic pinches instead of
     re-fighting them.
  3. RIP COST FUNCTION: SEMC siblings (routed by this run, same machinery) are
     cheap; power straps cheap; foreign signal moderate (we never actually rip
     foreign copper — PNS shove handles it — but the cost table is in place);
     length-matched / diff-pair / pinned-in-tight-via-field nets expensive.
  4. ORDERING AS A FREE VARIABLE: the retry queue discovers the order.

Only copper this run created is ever ripped (the 10 target nets had zero
segments at baseline), so the rip surgery can never destroy pre-existing
routing. Escape vias are never touched.

Usage:
  negotiate.py <board.kicad_pcb> --out <result.kicad_pcb> [--layer In2.Cu]
               [--max-iters 30] [--clearance-mm 0.1] [--cell-mm 0.06]
"""

import argparse
import collections
import json
import math
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import coarse_planner as cp
import tx_guard

HERE = os.path.dirname(os.path.abspath(__file__))

# The 10 walled In2 ADDR nets of CAL-702 (net -> (from_pad, to_pad)).
DEFAULT_NETS = {
    '/SEMC_A0':  ('U6.A3', 'U7.G2'),
    '/SEMC_A1':  ('U6.A2', 'U7.G1'),
    '/SEMC_A2':  ('U6.C2', 'U7.J3'),
    '/SEMC_A4':  ('U6.D5', 'U7.J5'),
    '/SEMC_A5':  ('U6.B1', 'U7.H1'),
    '/SEMC_A6':  ('U6.C1', 'U7.J4'),
    '/SEMC_A7':  ('U6.D3', 'U7.J2'),
    '/SEMC_A10': ('U6.B2', 'U7.G3'),
    '/SEMC_A11': ('U6.C4', 'U7.H4'),
    '/SEMC_A12': ('U6.C3', 'U7.H5'),
}


# --------------------------------------------------------------------------
# board-text surgery + queries (dual sexpr format via coarse_planner helpers)
# --------------------------------------------------------------------------
def net_segments(txt, net, layer):
    """[(x1,y1,x2,y2,width)] of the net's segments on the layer."""
    nets = cp.net_name_map(txt)
    out = []
    for seg in cp.brace_blocks(txt, 'segment'):
        if f'(layer "{layer}")' not in seg:
            continue
        if cp.item_net_name(seg, nets) != net:
            continue
        s = re.search(r'\(start ([\-\d.]+) ([\-\d.]+)\)', seg)
        e = re.search(r'\(end ([\-\d.]+) ([\-\d.]+)\)', seg)
        w = re.search(r'\(width ([\-\d.]+)\)', seg)
        if s and e:
            out.append((float(s.group(1)), float(s.group(2)),
                        float(e.group(1)), float(e.group(2)),
                        float(w.group(1)) if w else 0.2))
    return out


def net_via_positions(txt, net):
    nets = cp.net_name_map(txt)
    out = []
    for v in cp.brace_blocks(txt, 'via'):
        if cp.item_net_name(v, nets) != net:
            continue
        at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', v)
        if at:
            out.append((float(at.group(1)), float(at.group(2))))
    return out


def rip_net(txt, net, layer):
    """Remove ALL of the net's segments on the layer (never vias/pads).
    Returns (new_txt, ripped_count)."""
    nets = cp.net_name_map(txt)
    spans = []
    for m in re.finditer(r'\(segment\b', txt):
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
                    break
            j += 1
        blk = txt[s:j + 1]
        if f'(layer "{layer}")' in blk and cp.item_net_name(blk, nets) == net:
            # extend to trailing newline/indent for clean removal
            e = j + 1
            while e < len(txt) and txt[e] in '\n\t ':
                if txt[e] == '\n':
                    e += 1
                    break
                e += 1
            spans.append((s, e))
    for s, e in reversed(spans):
        txt = txt[:s] + txt[e:]
    return txt, len(spans)


def is_connected(txt, net, layer, tol=0.02):
    """Fast check: do the net's two escape vias connect through its segments on
    the layer? Union-find over segment endpoints + via anchors."""
    segs = net_segments(txt, net, layer)
    vias = net_via_positions(txt, net)
    if len(vias) < 2 or not segs:
        return False

    pts = []       # canonical points
    parent = []

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    def pid(x, y, merge_tol):
        for i, (px, py) in enumerate(pts):
            if abs(px - x) <= merge_tol and abs(py - y) <= merge_tol:
                return i
        pts.append((x, y))
        parent.append(len(pts) - 1)
        return len(pts) - 1

    for (x1, y1, x2, y2, w) in segs:
        a = pid(x1, y1, tol)
        b = pid(x2, y2, tol)
        union(a, b)

    # via joins: a via counts as connected to any endpoint within via-ish reach
    via_ids = []
    for (vx, vy) in vias[:2]:
        vid = pid(vx, vy, tol)
        via_ids.append(vid)
        for i, (px, py) in enumerate(pts):
            if math.hypot(px - vx, py - vy) <= 0.25:   # via pad radius + slop
                union(vid, i)

    return find(via_ids[0]) == find(via_ids[1])


# --------------------------------------------------------------------------
# rip cost function (coordinator spec #3)
# --------------------------------------------------------------------------
def rip_cost(net, txt=None):
    """Lower = cheaper to rip."""
    if re.match(r'^/?SEMC_', net.lstrip('/')):
        # length-matched groups inside SEMC (CLK/DQS) would be pricier; the
        # ADDR bus re-routes with the same machinery = cheap
        if re.search(r'(CLK|DQS)', net):
            return 20.0
        return 1.0
    if re.search(r'(GND|VDD|VCC|PWR|[0-9]V[0-9])', net, re.I):
        return 1.0          # power straps: cheap
    if re.search(r'(_P$|_N$|\+$|-$)', net):
        return 50.0         # diff pair: expensive
    return 5.0              # foreign signal: moderate (never ripped in v1)


# --------------------------------------------------------------------------
# driver wrappers
# --------------------------------------------------------------------------
def run_planner(board, net, frm, to, layer, cell, clr, penalty_file):
    cmd = [sys.executable, os.path.join(HERE, 'coarse_planner.py'), board,
           '--from', frm, '--to', to, '--net', net, '--layer', layer,
           '--cell-mm', str(cell), '--clearance-mm', str(clr),
           '--max-hop-mm', '0.4']
    if penalty_file:
        cmd += ['--penalty-file', penalty_file]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return None
    return r.stdout.strip()


def run_pns_route(binary, board, net, frm, to, layer, wps, env):
    cmd = [binary, board, '--net', net, '--from', frm, '--to', to,
           '--layer', layer, '--waypoints', wps, '-o', board]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    stall = None
    m = re.search(r'STALLED at hop (\d+)/(\d+) near \(([\-\d.]+), ([\-\d.]+)\)',
                  r.stderr)
    if m:
        stall = (int(m.group(1)), int(m.group(2)),
                 float(m.group(3)), float(m.group(4)))
    return r.stdout + r.stderr, stall


# --------------------------------------------------------------------------
# negotiation loop
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('--out', required=True)
    ap.add_argument('--layer', default='In2.Cu')
    ap.add_argument('--binary', default=None, help='path to pns-route binary')
    ap.add_argument('--max-iters', type=int, default=30)
    ap.add_argument('--max-attempts-per-net', type=int, default=8)
    ap.add_argument('--clearance-mm', type=float, default=0.1)
    ap.add_argument('--cell-mm', type=float, default=0.06)
    ap.add_argument('--persistent', action='store_true',
                    help='lock-in mode: keep the board across passes; closed '
                         'nets stay routed, only unresolved nets re-route '
                         '(targeted blocker rip-up still allowed)')
    ap.add_argument('--kicad-cli', default=None,
                    help='path to the fork kicad-cli; enables the authoritative '
                         'DRC dominance gate on adoption (no board is adopted '
                         'unless it closes more targets AND breaks no net)')
    args = ap.parse_args()

    binary = args.binary or os.path.join(HERE, '..', '..', '..', 'build',
                                         'pcbnew', 'router', 'pns-route')
    env = dict(os.environ, DYLD_LIBRARY_PATH='/opt/homebrew/lib')

    with open(args.board) as fh:
        pristine = fh.read()

    work = args.out
    layer = args.layer
    nets = dict(DEFAULT_NETS)
    penalties = []                        # history circles, persist across passes
    penalty_file = work + '.penalties.json'
    fail_hist = collections.Counter()     # per-net wedge count (drives ordering)
    rips = collections.Counter()
    t0 = time.time()
    log = []
    best = (0, None, [])                  # (n_connected, board_txt, closed_list)

    PENALTY_CAP = 6.0

    def bump_penalty(x, y, amt=1.0):
        for p in penalties:
            if math.hypot(p['x'] - x, p['y'] - y) < 0.3:
                p['cost'] = min(PENALTY_CAP, p['cost'] + amt)
                return
        penalties.append({'x': x, 'y': y, 'r': 0.5, 'cost': amt})

    def decay_penalties(factor=0.85, floor=0.2):
        # recency-weighted history: stale pinches fade so the channel never
        # saturates into all-expensive (the classic PathFinder history decay)
        keep = []
        for p in penalties:
            p['cost'] *= factor
            if p['cost'] >= floor:
                keep.append(p)
        penalties[:] = keep

    def dangling_end(txt, net, goal_xy):
        """Where the net's laid copper actually stopped: the segment endpoint
        closest to the goal via. Used as the wedge position when the driver
        reported no stall (its false-'reached' cases)."""
        best_pt, best_d = None, 1e18
        for (x1, y1, x2, y2, w) in net_segments(txt, net, layer):
            for (px, py) in ((x1, y1), (x2, y2)):
                d = math.hypot(px - goal_xy[0], py - goal_xy[1])
                if d < best_d:
                    best_d, best_pt = d, (px, py)
        return best_pt

    def route_tx(net, frm, to, wps, ripped=()):
        """Run pns-route as a TRANSACTION. Snapshot the board, route, then check
        whether the shove severed any *other* net on this layer (component count
        rose). If so, roll the board back and report the step rejected. `ripped`
        = nets we intentionally removed this step (exempt, we re-route them). This
        is the fix for the v3 regression: a shove that closes the target but snaps
        a neighbour is undone instead of committed. Returns (accepted, stall, txt)."""
        snap = open(work).read()
        pre = tx_guard.layer_components(snap, layer)
        _out, stall = run_pns_route(binary, work, net, frm, to, layer, wps, env)
        txt = open(work).read()
        bad = tx_guard.regressed_nets(pre, tx_guard.layer_components(txt, layer),
                                      exempt={net, *ripped})
        if bad:
            open(work, 'w').write(snap)        # undo the damaging shove
            return False, stall, snap, bad
        return True, stall, txt, []

    order = list(nets.keys())
    persistent_routed = set()

    # authoritative baseline for the DRC dominance gate (optional)
    base_drc = tx_guard.drc_metrics(args.board, args.kicad_cli) if args.kicad_cli else None

    open(work, 'w').write(pristine)

    for pss in range(1, args.max_iters + 1):
        if args.persistent:
            # ---- lock-in: keep board + already-closed nets --------------
            routed = set(persistent_routed)
        else:
            # ---- fresh pass: pristine board, accumulated history --------
            open(work, 'w').write(pristine)
            routed = set()

        with open(penalty_file, 'w') as fh:
            json.dump(penalties, fh)

        for net in order:
            if net in routed:
                continue
            frm, to = nets[net]
            wps = run_planner(work, net, frm, to, layer, args.cell_mm,
                              args.clearance_mm,
                              penalty_file if penalties else None)

            if wps is None:
                fail_hist[net] += 1
                log.append(f"[pass {pss}] {net}: planner NO PATH")
                continue

            accepted, stall, txt, bad = route_tx(net, frm, to, wps)
            if not accepted:
                fail_hist[net] += 1
                log.append(f"[pass {pss}] {net}: shove REJECTED (would sever {bad})")
                continue

            if is_connected(txt, net, layer):
                routed.add(net)
                continue

            # -- wedge: rip own partial, try blocker rip-up once ----------
            sx = sy = None
            if stall:
                _, _, sx, sy = stall
            else:
                # driver saw no stall (its false-'reached' case) — penalise
                # where the laid copper actually stopped, nearest the goal via.
                # The goal (U7-side) escape via is the westernmost of the pair.
                gvias = net_via_positions(txt, net)
                if gvias:
                    goal = min(gvias, key=lambda v: v[0])
                    dpt = dangling_end(txt, net, goal)
                    if dpt:
                        sx, sy = dpt
            if sx is not None:
                bump_penalty(sx, sy)
            txt, _ = rip_net(txt, net, layer)
            fail_hist[net] += 1

            retried = False
            if sx is not None and routed:
                blockers = []
                for rn in routed:
                    for (x1, y1, x2, y2, w) in net_segments(txt, rn, layer):
                        dx, dy = x2 - x1, y2 - y1
                        L2 = dx * dx + dy * dy or 1e-12
                        t = max(0.0, min(1.0, ((sx - x1) * dx + (sy - y1) * dy) / L2))
                        if math.hypot(x1 + t * dx - sx, y1 + t * dy - sy) < 0.6:
                            blockers.append(rn)
                            break
                blockers.sort(key=rip_cost)

                if blockers:
                    # TRANSACTIONAL rip: snapshot; only keep the rip if it
                    # strictly improves the connected count, else roll back.
                    # (Unguarded rips destroyed locked wins: a wedging sibling
                    # would rip a good net and then neither would connect.)
                    snap_txt = txt
                    snap_routed = set(routed)

                    b = blockers[0]
                    txt, _ = rip_net(txt, b, layer)
                    routed.discard(b)
                    open(work, 'w').write(txt)

                    # wedged net retries through the freed corridor
                    wps2 = run_planner(work, net, frm, to, layer, args.cell_mm,
                                       args.clearance_mm, penalty_file)
                    if wps2:
                        acc2, stall2, txt, _bad2 = route_tx(net, frm, to, wps2,
                                                            ripped={b})
                        if acc2 and is_connected(txt, net, layer):
                            routed.add(net)
                        else:
                            txt, _ = rip_net(txt, net, layer)
                            if stall2:
                                bump_penalty(stall2[2], stall2[3])

                    # blocker re-routes right away
                    open(work, 'w').write(txt)
                    wpsb = run_planner(work, b, nets[b][0], nets[b][1], layer,
                                       args.cell_mm, args.clearance_mm,
                                       penalty_file)
                    if wpsb:
                        accb, _sb, txt, _badb = route_tx(b, nets[b][0], nets[b][1],
                                                         wpsb)
                        if accb and is_connected(txt, b, layer):
                            routed.add(b)
                        else:
                            txt, _ = rip_net(txt, b, layer)

                    if len(routed) > len(snap_routed):
                        rips[b] += 1          # rip paid off — commit
                        retried = net in routed
                    else:
                        txt = snap_txt        # roll back the whole transaction
                        routed = snap_routed

            open(work, 'w').write(txt)

            if not retried:
                log.append(f"[pass {pss}] {net}: WEDGED"
                           f"{' at (%.2f, %.2f)' % (sx, sy) if sx is not None else ''}")

        closed = sorted(routed, key=lambda x: int(re.search(r'\d+', x).group()))
        log.append(f"[pass {pss}] === {len(routed)}/{len(nets)} connected: "
                   f"{[n.split('_')[-1] for n in closed]}  "
                   f"(penalties={len(penalties)})")

        if args.persistent:
            persistent_routed = set(routed)

        if len(routed) > best[0]:
            cand = open(work).read()
            adopt = True
            if args.kicad_cli and base_drc is not None:
                m = tx_guard.drc_metrics(work, args.kicad_cli)
                if m is not None:
                    # authoritative PER-NET backstop: no net that was connected in
                    # the baseline may become unconnected (aggregate counts mask a
                    # break when target closes outnumber it), and no new shorts.
                    new_broken = m['unconnected_nets'] - base_drc['unconnected_nets']
                    adopt = (not new_broken and m['gating'] <= base_drc['gating'])
                    log.append(f"[pass {pss}] DRC gate: broke={sorted(new_broken)} "
                               f"gating {m['gating']}/{base_drc['gating']} -> "
                               f"{'ADOPT' if adopt else 'REJECT'}")
            if adopt:
                best = (len(routed), cand, closed)

        if len(routed) == len(nets):
            break

        # ordering for next pass: hardest (most-wedged) nets first — they get
        # first claim on corridors; ties keep current relative order
        order.sort(key=lambda n: -fail_hist[n])

        # recency-weight the congestion history so it never saturates
        decay_penalties()

    dt = time.time() - t0

    # adopt the best pass
    if best[1] is not None:
        open(work, 'w').write(best[1])

    print("=" * 70)
    for line in log:
        print(line)
    print("=" * 70)
    print(f"NEGOTIATION RESULT (best pass): {best[0]}/{len(nets)} connected, "
          f"{pss} passes, {dt:.0f}s wall")
    print(f"  connected: {best[2]}")
    print(f"  unresolved: {sorted(set(nets) - set(best[2]))}")
    print(f"  rip counts: {dict(rips)}")
    print(f"  penalty regions: {len(penalties)}")

    if os.path.exists(penalty_file):
        os.remove(penalty_file)

    return 0 if best[0] == len(nets) else 1


if __name__ == '__main__':
    sys.exit(main())
