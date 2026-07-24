# Headless PNS Router — Spike Plan & Scaffold

**Goal:** expose KiCad's Push-and-Shove (PNS) router — the interactive "shove neighbors
aside" engine — as a **headless CLI an agent can call**, so the fleet routes congested
crossings (e.g. CAL-702's CLK straddle) with no GUI, no contractor, no human. Durable, not
one-off: headless shove for every future board.

**Delivery model (same as the `freerouting` fork):** fork KiCad → add our headless driver as
a patch → build → ship a `pns-route` wrapper → wire it into the `board-route` skill so agents
call it exactly like they call `frroute`. Repo: `calidran/kicad` (mirrors `calidran/freerouting`).

Written 2026-07-23. KiCad is GPLv3 — a fork with our additions is clean.

---

## Why it's tractable (verified against source, not memory)

From `pcbnew/router/pns_router.h` — the engine already exposes a clean programmatic API; no
mouse events are needed at this level, just coordinates + items:

```
SetMode( PNS_MODE_ROUTE_SINGLE )                 // routing vs tuning mode
StartRouting( const VECTOR2I& p, ITEM* start, int layer )
Move( const VECTOR2I& p, ITEM* end )             // extend toward target (shove happens here)
FixRoute( const VECTOR2I& p, ITEM* end, bool forceFinish, bool forceCommit )
CommitRouting()                                   // write result back into the NODE/board
SwitchLayer( int layer );  SetIterLimit( int )
```

Point-to-point-with-shove ≈ `StartRouting(source)` → `FixRoute(target)`.

From `pcbnew/router/pns_kicad_iface.h` — the **only** GUI coupling is display/preview:
`SetView(KIGFX::VIEW*)`, `m_previewItems (VIEW_GROUP*)`, `SetHostTool(PCB_TOOL_BASE*)`. These
render the route as you drag; they are **not** the routing algorithm. Stub/no-op them and the
shove engine runs headless. That's the whole trick.

**So the risk is build friction, not research.** The algorithm is decoupled from rendering;
the API is right there.

---

## The scaffold (structure — fill the TODOs against source)

A new standalone tool, e.g. `pcbnew/router/pns_headless/pns_route_cli.cpp`:

```cpp
// pns_route_cli.cpp — headless PNS driver.  Usage:
//   pns-route board.kicad_pcb --net /SEMC_CLK --from <refdes.pad> --to <refdes.pad>
//             --layer In2.Cu [--iter-limit N]  -> writes board.kicad_pcb
//
// 1. LOAD BOARD headlessly.
//    BOARD* board = loadBoard(path);            // TODO: PCB_IO_KICAD_SEXPR::LoadBoard (io/kicad_sexpr)
//
// 2. HEADLESS IFACE — subclass PNS_KICAD_IFACE, no-op the display hooks:
//    class HEADLESS_IFACE : public PNS_KICAD_IFACE {
//        void SetView( KIGFX::VIEW* ) override {}          // no render
//        void DisplayItem(...) override {}                 // no preview
//        void HideItem(...) override {}
//        void UpdateNet(...) override {}                   // keep the board-write ones
//        // keep: SyncWorld(), the NODE build, and the commit-back path
//    };
//    HEADLESS_IFACE iface;  iface.SetBoard( board );       // TODO: confirm setter name
//
// 3. ROUTER + WORLD:
//    PNS::ROUTER router;  router.SetInterface( &iface );
//    router.SyncWorld();                                   // build PNS::NODE from BOARD
//                                                          // TODO: confirm who owns SyncWorld (iface vs router)
//    router.SetMode( PNS::PNS_MODE_ROUTE_SINGLE );
//    router.Settings().SetMode( PNS::RM_Shove );           // TODO: confirm shove-mode enum/setter
//
// 4. FIND ENDPOINTS: resolve --from/--to refdes.pad -> ITEM* (start) + VECTOR2I (target).
//    ITEM* start = findPadItem( iface.GetWorld(), fromRef ); // TODO: NODE::HitTest / item lookup
//    VECTOR2I target = padCenter( toRef );
//
// 5. ROUTE with shove:
//    router.StartRouting( startPos, start, layerId );
//    router.Move( target, nullptr );                       // let PNS shove/walk toward target
//    router.FixRoute( target, endItem, /*forceFinish*/true, /*forceCommit*/true );
//    router.CommitRouting();
//
// 6. SAVE: write board back.  // TODO: PCB_IO_KICAD_SEXPR::SaveBoard
```

### Source files the build session must read to fill the TODOs
- `pcbnew/router/pns_kicad_iface.{h,cpp}` — `SyncWorld`, board↔NODE build, the commit-back
  path, which display methods are virtual (to no-op) vs. which write the board (to keep).
- `pcbnew/router/router_tool.cpp` — the canonical StartRouting→Move→FixRoute *sequence* and
  how mode/shove settings are applied (copy this driving order).
- `pcbnew/router/pns_router.{h,cpp}` — `Settings()`, shove-mode enum, `SetInterface`.
- `pcbnew/router/pns_node.h` — item lookup (find a pad/via ITEM by net/position).
- `pcbnew/pcb_io/kicad_sexpr/` — headless board load/save.

---

## Build approach
- Fork `KiCad/kicad` → `calidran/kicad`, branch `headless-pns`.
- Add the tool under `pcbnew/router/pns_headless/` with a CMake target that links the router
  lib + `pcbnew_kiface`/`common` (the same libs `pnsrouter` already builds against — no new
  deps beyond KiCad's existing wx/boost/OCC toolchain).
- Main friction = a full KiCad source build (large CMake, wx/boost/OCC). Budget for that.
- The box already has KiCad 9.0.x installed (runtime); the build needs the dev deps —
  do it in a dedicated worktree/session, not inline.

## Validation target (the eval — prove it works)
1. Route `/SEMC_CLK` on CAL-702 (the exact net that walled): `pns-route calidran-rev0.kicad_pcb
   --net /SEMC_CLK --from U6.<ball> --to U7.<ball> --layer In2.Cu` → expect a shoved,
   DRC-clean CLK trace where hand-routing + frroute both failed.
2. Confirm neighbors were *shoved, not shorted* (DRC 0-new).
3. If CLK is blocked by **fixed** via-fields rather than shoveable copper (open question to
   electrical — see the routing review), shove won't rescue it and In7 is the real fix; the
   spike still stands as the durable capability for the shoveable cases.

## Wire into the fleet (mirror freerouting)
- Ship a `pns-route` wrapper (like the `frroute` wrapper): arg-marshal, run isolated,
  poll/return, best-effort.
- Add a section to the `board-route` skill: "when a net needs push-shove (crossing a
  congested bundle, neighbors are moveable copper), call `pns-route` — the headless PNS
  engine — instead of escalating for a GUI/contractor." This *closes the exact tool gap* that
  the option-generation doctrine says to close rather than buy.

---

## Status / RESULTS (build spike executed 2026-07-24)

**The driver builds, runs, and routes headlessly — validated end-to-end on the real
CAL-702 board.** Concrete outcomes below.

### Corrected iface approach (the scaffold had it backwards)
PNS has TWO ifaces. `PNS_KICAD_IFACE` (derived) owns the VIEW / `PCB_TOOL_BASE` /
`BOARD_COMMIT` — that IS the GUI coupling. `PNS_KICAD_IFACE_BASE` (its base) is *already*
headless: it holds only `BOARD*` + `PNS::NODE*`, every `Display*/Hide` hook is already a
no-op, and `SyncWorld`/`syncPad`/`syncTrack`/`syncVia` build the NODE straight from the
BOARD. So the real driver **subclasses the BASE iface** and implements
`AddItem`/`RemoveItem`/`UpdateItem`/`Commit` to mutate the BOARD directly (no
`BOARD_COMMIT`, no tool). `ROUTER::CommitRouting()` calls exactly those four during
`FixRoute`. This mirrors KiCad's own headless PNS test harness `qa/tools/pns`
(`pns_log_player.cpp`) — the proven pattern.

### Minimal build recipe (verified)
There is **no small PNS-only island**: `pnsrouter` compiles `pns_kicad_iface.cpp`
(references `BOARD_COMMIT`, `PCB_SELECTION_TOOL`) and the tool framework, so linking it
pulls the whole pcbnew backend. The minimal build = configure the full KiCad CMake tree
(GUI app not built) and `ninja` only the `pns-route` target. Recipe:

- `option( KICAD_BUILD_PNS_HEADLESS )` in `pcbnew/router/CMakeLists.txt` adds a `pns-route`
  executable linking `pcbnew_kiface_objects` + `pnsrouter` + `pcbcommon` + `connectivity`
  + `common` + `gal` + `3d-viewer` + `kicommon` + `${PCBNEW_IO_LIBRARIES}` (the same link
  set as `qa/tools/pns`). The CLI also provides a minimal `Kiface()` stub.
- Configure deps (macOS/Homebrew names): `wxwidgets boost glew glm curl libgit2 ninja
  opencascade libngspice protobuf nng harfbuzz fontconfig freetype cairo unixodbc gettext`.
  Ubuntu equivalents via KiCad's documented `build-deps`. NOTE: ngspice/OCC/nng/protobuf
  are `find_package(REQUIRED)` at top-level in KiCad 9.x — they must be present to
  *configure* even though the PNS lib doesn't use them.
- `cmake -G Ninja -DKICAD_BUILD_PNS_HEADLESS=ON -DKICAD_BUILD_QA_TESTS=OFF ...` then
  `ninja pns-route`. Full build of the ~1300 backend objects: ~5 min wall on an 10-core
  M-series mac (`ninja -j` default). Do NOT build on the prod box (2 cores / 3.7 GB).
- Build the tree's own `kicad-cli` target too (fast, reuses the libs) for DRC validation —
  the fork is off master (writes board format 20260624) which the box's KiCad 9.0.9
  cannot read.

### Validation on CAL-702 (`calidran-rev0.kicad_pcb`, In2.Cu ADDR nets)
Baseline: 10 of the 13 SEMC_Ax nets are entirely unrouted (A0,A1,A2,A4,A5,A6,A7,A10,A11,A12
— the walled set; A3,A8,A9 already routed). Ran `pns-route` deepest-first, cumulatively.

**Result: 1 of 10 walled nets closed DRC-clean (A5). Clearance violations unchanged
(499→499 — the shove was clean, no new shorts.)** Of the rest: 5 nets (A0/A1/A2/A6/A10)
laid partial copper stubs that didn't reach both pads; the 4 hardest (A4/A7/A11/A12) laid
**zero** copper — the head can't even leave the pad on In2 (StartRouting succeeds, Move
never extends). That matches the geometry review's open question: the hardest balls are
gated by the **fixed via field**, not shoveable copper, so shove can't rescue them — In7
is the real fix there.

### What's left (the gap between "runs" and "beats the wall")
A naive point-to-point `StartRouting→Move(target)→FixRoute` only completes nets threadable
in a single straight shot. The GUI user makes **intermediate waypoint moves** to steer the
head around obstacles; the CLI needs the same. Next iteration: (a) waypoint/auto-finish
steering (feed the router a path of intermediate points, or use `Finish()`/
`GetNearestRatnestAnchor`), (b) multi-layer escapes (via + layer change), (c) try the
partial-copper nets with steering before concluding they're via-gated. The capability
(headless shove that writes DRC-valid copper) is proven; the driving policy needs work.

### Wrapping into the fleet
Once completion improves: ship a `pns-route` wrapper mirroring `frroute` and add a
`board-route` skill section. Not done yet — premature until the driver closes more than 1/10.
