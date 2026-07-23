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

## Status / next step
Scaffold + build plan ready. **Next action needs a go on one outward step:** creating the
`calidran/kicad` fork on GitHub (large public fork — same call you made for `calidran/freerouting`).
On that go: fork → drop in the CLI above → read the 5 source files → build → validate on
`/SEMC_CLK` → wrapper + skill wire-in.
