#!/bin/bash
# Two-layer route of the 10 walled In2 ADDR nets: coarse planner -> pns-route waypoints.
# Usage: route_all.sh <order: deepest|easiest> <clearance_mm> [cell_mm]
set -u
ORDER="${1:-deepest}"
CLR="${2:-0.2}"
CELL="${3:-0.08}"
BIN=kicad/build/pcbnew/router/pns-route
PLAN=kicad/pcbnew/router/pns_headless/coarse_planner.py
export DYLD_LIBRARY_PATH=/opt/homebrew/lib

# net:from:to  — deepest-first order (hardest walled balls first)
DEEPEST=(
 "A4:U6.D5:U7.J5" "A11:U6.C4:U7.H4" "A7:U6.D3:U7.J2" "A12:U6.C3:U7.H5"
 "A2:U6.C2:U7.J3" "A6:U6.C1:U7.J4" "A10:U6.B2:U7.G3" "A5:U6.B1:U7.H1"
 "A1:U6.A2:U7.G1" "A0:U6.A3:U7.G2"
)
# easiest-first = reverse
EASIEST=(
 "A0:U6.A3:U7.G2" "A1:U6.A2:U7.G1" "A5:U6.B1:U7.H1" "A10:U6.B2:U7.G3"
 "A6:U6.C1:U7.J4" "A2:U6.C2:U7.J3" "A12:U6.C3:U7.H5" "A7:U6.D3:U7.J2"
 "A11:U6.C4:U7.H4" "A4:U6.D5:U7.J5"
)
if [ "$ORDER" = "easiest" ]; then NETS=("${EASIEST[@]}"); else NETS=("${DEEPEST[@]}"); fi

WORK="work_${ORDER}.kicad_pcb"
cp calidran-rev0.kicad_pcb "$WORK"
ok=0; total=0
for entry in "${NETS[@]}"; do
  IFS=: read nm f t <<<"$entry"; net="/SEMC_$nm"; total=$((total+1))
  wps=$(python3 "$PLAN" "$WORK" --from "$f" --to "$t" --net "$net" --layer In2.Cu --cell-mm "$CELL" --clearance-mm "$CLR" --max-hop-mm 0.4 2>/dev/null)
  planrc=$?
  if [ $planrc -ne 0 ]; then
    printf "%-4s %-12s PLANNER: no coarse path (fixed-geometry boxed)\n" "$nm" "$f->$t"
    continue
  fi
  out=$($BIN "$WORK" --net "$net" --from "$f" --to "$t" --layer In2.Cu --iter-limit 100 --waypoints "$wps" -o "$WORK" 2>&1)
  res=$(echo "$out" | grep -oE 'completed=(yes|no)')
  extra=$(echo "$out" | grep -oE 'traversed=[0-9]+/[0-9]+ moves=[0-9]+ added=[0-9]+')
  stall=$(echo "$out" | grep -oE 'STALLED at hop [0-9]+/[0-9]+' | head -1)
  nwp=$(echo "$wps" | tr ';' '\n' | grep -c ',')
  printf "%-4s %-12s %s %s wps=%s %s\n" "$nm" "$f->$t" "$res" "$extra" "$nwp" "$stall"
  echo "$out" | grep -q 'completed=yes' && ok=$((ok+1))
done
echo "----"
echo "[$ORDER, clr=$CLR cell=$CELL] COMPLETED $ok / $total walled ADDR nets"
