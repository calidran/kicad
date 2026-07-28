#!/usr/bin/env python3
"""
tx_guard.py — the transactional guard that makes headless PNS rip-up all-or-nothing.

The v3 negotiator regressed (broke /SEMC_D6, /SEMC_RAS_B and left them dangling) because
its only success test was `len(routed)` over the TEN target ADDR nets — it was blind to
foreign nets that the pns-route *shove* severed as a side effect. A shove that closed a
target net while snapping a neighbour still passed the guard, and the damaged board was
committed. That is the exact "rip-up is not transactional" blocker in SPIKE.md.

This module supplies the missing oracle so every routing step can be wrapped in a
transaction: snapshot the board, run the step, and if ANY previously-intact net was
severed, roll the board back. Two tiers:

  * layer_components() / regressed_nets()  — a FAST per-layer connectivity oracle used to
    reject a damaging shove immediately (before it compounds across later attempts). It is
    deliberately CONSERVATIVE: it can over-reject (a net redundantly connected through
    another layer looks severed on this one) but it never MISSES an on-layer severance, so
    it can only cost close-count, never allow a regression.

  * drc_metrics()  — the AUTHORITATIVE KiCad connectivity/violation count (the fork's own
    kicad-cli). Used at adoption time: a candidate board is only adopted if it dominates
    the baseline (more target nets closed AND unconnected/gating not worse). This makes a
    regression impossible in the emitted board by construction.

Pure stdlib; reuses coarse_planner's board parsing (no duplicate sexpr logic).
"""
import math
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import coarse_planner as cp

STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu',
         'In5.Cu', 'In6.Cu', 'In7.Cu', 'In8.Cu', 'B.Cu']


class _UF:
    """Point union-find with coincidence merging at a tolerance."""
    def __init__(self, tol=0.02):
        self.tol = tol
        self.pts = []
        self.parent = []

    def _find(self, i):
        while self.parent[i] != i:
            self.parent[i] = self.parent[self.parent[i]]
            i = self.parent[i]
        return i

    def union(self, a, b):
        ra, rb = self._find(a), self._find(b)
        if ra != rb:
            self.parent[ra] = rb

    def pid(self, x, y, tol=None):
        t = self.tol if tol is None else tol
        for i, (px, py) in enumerate(self.pts):
            if abs(px - x) <= t and abs(py - y) <= t:
                return i
        self.pts.append((x, y))
        self.parent.append(len(self.pts) - 1)
        return len(self.pts) - 1

    def components(self, ids):
        return len({self._find(i) for i in ids})


def _via_anchors(txt, layer, nets):
    """{net: [(x,y)]} of vias whose span reaches `layer`."""
    li = STACK.index(layer) if layer in STACK else -1
    out = {}
    for v in cp.brace_blocks(txt, 'via'):
        at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', v)
        if not at:
            continue
        lyr = re.search(r'\(layers "([^"]+)" "([^"]+)"\)', v)
        if lyr and li >= 0 and lyr.group(1) in STACK and lyr.group(2) in STACK:
            lo, hi = sorted((STACK.index(lyr.group(1)), STACK.index(lyr.group(2))))
            if not (lo <= li <= hi):
                continue
        net = cp.item_net_name(v, nets)
        if net:
            out.setdefault(net, []).append((float(at.group(1)), float(at.group(2))))
    return out


def _pad_anchors(txt, layer, nets):
    """{net: [(x,y)]} of through-hole / on-layer pads (fixed copper on `layer`)."""
    out = {}
    for fp in cp.brace_blocks(txt, 'footprint'):
        trm = re.search(r'\(transform\s*\(translate ([\-\d.]+) ([\-\d.]+)\)\s*\(rotate ([\-\d.]+)\)', fp)
        if trm:
            fx, fy, frot = float(trm.group(1)), float(trm.group(2)), float(trm.group(3))
        else:
            first_pad = fp.find('(pad ')
            head = fp[:first_pad] if first_pad >= 0 else fp
            atm = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)(?: ([\-\d.]+))?\)', head)
            if not atm:
                continue
            fx, fy, frot = float(atm.group(1)), float(atm.group(2)), float(atm.group(3) or 0)
        for pm in re.finditer(
                r'\(pad "([^"]+)"\s+(\S+)\s+(\S+)[\s\S]*?\(at ([\-\d.]+) ([\-\d.]+)(?: ([\-\d.]+))?\)'
                r'[\s\S]*?\(size ([\-\d.]+) ([\-\d.]+)\)([\s\S]*?)(?=\(pad "|\Z)', fp):
            ptype = pm.group(2)
            prx, pry = float(pm.group(4)), float(pm.group(5))
            tail = pm.group(9)
            layers = re.search(r'\(layers ([^\)]*)\)', tail)
            lstr = layers.group(1) if layers else ""
            through = (ptype in ('thru_hole', 'np_thru_hole')) or '"*.Cu"' in lstr
            if not (through or f'"{layer}"' in lstr):
                continue
            netm = re.search(r'\(net (\d+) "([^"]*)"\)', tail)
            net = netm.group(2) if netm else None
            if not net:
                continue
            ox, oy = cp.rot_xy(prx, pry, frot)
            out.setdefault(net, []).append((fx + ox, fy + oy))
    return out


def layer_components(txt, layer):
    """{net: n_connected_components} of every net that has copper on `layer`.

    A net's on-layer graph = its segments (endpoints) + its via anchors (span reaches the
    layer) + its through-hole/on-layer pad anchors, merged by coincidence. An intact net
    that is fully joined on this layer has 1 component; a shove that SEVERS it (splits a
    trace, or detaches copper from its via/pad) raises the count. Comparing counts before
    vs after a step is what detects side-effect damage.
    """
    nets = cp.net_name_map(txt)
    seg_by_net = {}
    for seg in cp.brace_blocks(txt, 'segment'):
        if f'(layer "{layer}")' not in seg:
            continue
        s = re.search(r'\(start ([\-\d.]+) ([\-\d.]+)\)', seg)
        e = re.search(r'\(end ([\-\d.]+) ([\-\d.]+)\)', seg)
        if not (s and e):
            continue
        net = cp.item_net_name(seg, nets)
        if net:
            seg_by_net.setdefault(net, []).append(
                (float(s.group(1)), float(s.group(2)), float(e.group(1)), float(e.group(2))))

    vias = _via_anchors(txt, layer, nets)
    pads = _pad_anchors(txt, layer, nets)

    out = {}
    for net, segs in seg_by_net.items():
        uf = _UF(tol=0.02)
        node_ids = []
        for (x1, y1, x2, y2) in segs:
            a = uf.pid(x1, y1)
            b = uf.pid(x2, y2)
            uf.union(a, b)
            node_ids += [a, b]
        # anchors join to any nearby copper within via/pad reach
        for (ax, ay) in vias.get(net, []) + pads.get(net, []):
            aid = uf.pid(ax, ay)
            node_ids.append(aid)
            for i, (px, py) in enumerate(uf.pts):
                if i != aid and math.hypot(px - ax, py - ay) <= 0.25:
                    uf.union(aid, i)
        out[net] = uf.components(node_ids)
    return out


def regressed_nets(base_comp, cur_comp, exempt=()):
    """Nets whose on-layer component count INCREASED (i.e. got severed), excluding
    `exempt` (the net being routed and any net deliberately ripped this step)."""
    exempt = set(exempt)
    bad = []
    for net, cur in cur_comp.items():
        if net in exempt:
            continue
        if cur > base_comp.get(net, cur):   # unseen-before nets can't have regressed
            bad.append(net)
    return bad


def drc_metrics(board_path, kicad_cli, gate_types=None):
    """Authoritative connectivity/violation counts via the fork's kicad-cli.
    Returns {'unconnected': int, 'gating': int, 'violations': int} or None on failure."""
    import json
    import tempfile
    gate_types = gate_types or {'shorting_items', 'clearance',
                                'copper_edge_clearance', 'hole_clearance'}
    with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as tf:
        out = tf.name
    try:
        r = subprocess.run([kicad_cli, 'pcb', 'drc', '--format', 'json',
                            '--output', out, board_path],
                           capture_output=True, text=True)
        if not os.path.exists(out) or os.path.getsize(out) == 0:
            return None
        d = json.load(open(out))
        V = d.get('violations', [])
        from collections import Counter
        c = Counter(v['type'] for v in V)
        unconn = len(d.get('unconnected_items', []))
        return {'unconnected': unconn,
                'gating': sum(c[t] for t in gate_types),
                'violations': len(V)}
    finally:
        if os.path.exists(out):
            os.remove(out)
