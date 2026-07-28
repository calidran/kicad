/*
 * pns_route_cli.cpp — headless KiCad Push-and-Shove (PNS) router CLI.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 * This program is free software under GPLv3 (see LICENSE / the KiCad source).
 *
 * Drives KiCad's interactive push-and-shove engine with NO GUI so an agent can
 * route a congested net point-to-point and let PNS shove moveable neighbour
 * copper aside — the capability that is otherwise only reachable by dragging
 * in the pcbnew canvas.
 *
 * Usage:
 *   pns-route board.kicad_pcb --net <netname> \
 *             --from <REFDES.PAD> --to <REFDES.PAD> \
 *             --layer <LayerName> [--iter-limit N] [--width-mm W] \
 *             [--clearance-mm C] [--mode shove|walkaround] [-o out.kicad_pcb]
 *
 * Writes the routed board to -o (default: overwrite input).
 *
 * ---------------------------------------------------------------------------
 * How it stays headless (verified against source, 2026-07-24):
 *   PNS has two ifaces.  PNS_KICAD_IFACE (derived) owns the VIEW / PCB_TOOL_BASE
 *   / BOARD_COMMIT — that is the GUI coupling.  PNS_KICAD_IFACE_BASE (its base)
 *   is already headless: it holds only BOARD* + PNS::NODE*, all Display and
 *   Hide hooks are no-ops, and SyncWorld/syncPad/syncTrack/syncVia build the NODE
 *   straight from the BOARD.  Its AddItem/RemoveItem/UpdateItem/Commit are empty
 *   stubs (the derived class does the board write via a BOARD_COMMIT).
 *
 *   So instead of the SPIKE's "subclass the derived iface and no-op display",
 *   we subclass the BASE iface and implement AddItem/RemoveItem/UpdateItem/Commit
 *   to mutate the BOARD directly (no BOARD_COMMIT, no tool).  ROUTER::CommitRouting
 *   calls exactly those four during FixRoute, so the shove result lands on the
 *   board with zero GUI dependency.
 * ---------------------------------------------------------------------------
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <optional>

#include <wx/init.h>
#include <wx/string.h>
#include <wx/filename.h>

#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <netinfo.h>
#include <layer_ids.h>
#include <lset.h>
#include <settings/settings_manager.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>

// DRC engine + project — REQUIRED for a live clearance resolver (see below).
#include <project.h>
#include <project/project_local_settings.h>
#include <drc/drc_engine.h>
#include <drc/drc_rule.h>
#include <wildcards_and_files_ext.h>

#include <router/pns_router.h>
#include <router/pns_kicad_iface.h>
#include <router/pns_routing_settings.h>
#include <router/pns_sizes_settings.h>
#include <router/pns_node.h>
#include <router/pns_item.h>
#include <router/pns_itemset.h>
#include <router/pns_solid.h>
#include <router/pns_segment.h>
#include <router/pns_arc.h>
#include <router/pns_via.h>
#include <router/pns_placement_algo.h>

#include <kiface_base.h>
#include <kiway.h>


// ---------------------------------------------------------------------------
// Minimal KIFACE stub.  pcbnew_kiface_objects references the global Kiface()
// accessor (normally provided by the pcbnew kiface DSO). For a headless CLI we
// supply a do-nothing KIFACE_BASE, exactly as KiCad's own QA harness does
// (qa/qa_utils/test_app_main.cpp).
// ---------------------------------------------------------------------------
static struct HEADLESS_KIFACE : public KIFACE_BASE
{
    HEADLESS_KIFACE( const char* aName, KIWAY::FACE_T aType ) : KIFACE_BASE( aName, aType ) {}

    bool OnKifaceStart( PGM_BASE*, int, KIWAY* ) override { return true; }
    void OnKifaceEnd() override {}
    wxWindow* CreateKiWindow( wxWindow*, int, KIWAY*, int = 0 ) override { return nullptr; }
    void* IfaceOrAddress( int ) override { return nullptr; }
}
g_headlessKiface( "pns_route", KIWAY::FACE_PCB );

KIFACE_BASE& Kiface() { return g_headlessKiface; }


// ---------------------------------------------------------------------------
// Headless iface: base (already GUI-free) + direct-to-board write path.
// ---------------------------------------------------------------------------
class HEADLESS_PNS_IFACE : public PNS_KICAD_IFACE_BASE
{
public:
    HEADLESS_PNS_IFACE() : m_addedCount( 0 ), m_removedCount( 0 ), m_updatedCount( 0 ) {}

    // Router calls these three during CommitRouting(node); then Commit().
    void AddItem( PNS::ITEM* aItem ) override
    {
        BOARD_CONNECTED_ITEM* bi = createBoardItem( aItem );

        if( bi )
        {
            aItem->SetParent( bi );
            bi->ClearFlags();
            m_toAdd.push_back( bi );
            m_addedCount++;
        }
    }

    void RemoveItem( PNS::ITEM* aItem ) override
    {
        BOARD_ITEM* parent = aItem->Parent();

        // Pad moves surface as SOLID removes; we never move footprints headlessly.
        if( aItem->OfKind( PNS::ITEM::SOLID_T ) )
            return;

        if( parent )
        {
            m_toRemove.push_back( parent );
            m_removedCount++;
        }
    }

    void UpdateItem( PNS::ITEM* aItem ) override
    {
        modifyBoardItem( aItem );
        m_updatedCount++;
    }

    // Flush the collected board mutations. Called once at end of CommitRouting.
    void Commit() override
    {
        for( BOARD_ITEM* bi : m_toRemove )
            m_board->Remove( bi );

        for( BOARD_ITEM* bi : m_toAdd )
            m_board->Add( bi );

        // Modified items are mutated in place by modifyBoardItem(); nothing to flush.
        m_toRemove.clear();
        m_toAdd.clear();
    }

    void UpdateNet( PNS::NET_HANDLE ) override {}

    int GetNetCode( PNS::NET_HANDLE aNet ) const override
    {
        return aNet ? static_cast<const NETINFO_ITEM*>( aNet )->GetNetCode() : -1;
    }

    wxString GetNetName( PNS::NET_HANDLE aNet ) const override
    {
        return aNet ? static_cast<const NETINFO_ITEM*>( aNet )->GetNetname() : wxString();
    }

    int Added() const   { return m_addedCount; }
    int Removed() const { return m_removedCount; }
    int Updated() const { return m_updatedCount; }

private:
    // Build a new PCB_TRACK / PCB_ARC / PCB_VIA from a PNS item.  Mirrors
    // PNS_KICAD_IFACE::createBoardItem but with no group/replacement bookkeeping.
    BOARD_CONNECTED_ITEM* createBoardItem( PNS::ITEM* aItem )
    {
        NETINFO_ITEM* net = static_cast<NETINFO_ITEM*>( aItem->Net() );

        if( !net )
            net = NETINFO_LIST::OrphanedItem();

        switch( aItem->Kind() )
        {
        case PNS::ITEM::ARC_T:
        {
            PNS::ARC* arc = static_cast<PNS::ARC*>( aItem );
            PCB_ARC*  na = new PCB_ARC( m_board,
                                        static_cast<const SHAPE_ARC*>( arc->Shape( -1 ) ) );
            na->SetWidth( arc->Width() );
            na->SetLayer( GetBoardLayerFromPNSLayer( arc->Layers().Start() ) );
            na->SetNet( net );
            return na;
        }
        case PNS::ITEM::SEGMENT_T:
        {
            PNS::SEGMENT* seg = static_cast<PNS::SEGMENT*>( aItem );
            const SEG&    s = seg->Seg();
            PCB_TRACK*    tr = new PCB_TRACK( m_board );
            tr->SetStart( VECTOR2I( s.A.x, s.A.y ) );
            tr->SetEnd( VECTOR2I( s.B.x, s.B.y ) );
            tr->SetWidth( seg->Width() );
            tr->SetLayer( GetBoardLayerFromPNSLayer( seg->Layers().Start() ) );
            tr->SetNet( net );
            return tr;
        }
        case PNS::ITEM::VIA_T:
        {
            PNS::VIA* via = static_cast<PNS::VIA*>( aItem );
            PCB_VIA*  nv = new PCB_VIA( m_board );
            nv->SetPosition( VECTOR2I( via->Pos().x, via->Pos().y ) );
            nv->SetWidth( PADSTACK::ALL_LAYERS, via->Diameter( 0 ) );
            nv->SetDrill( via->Drill() );
            nv->SetNet( net );
            nv->SetViaType( via->ViaType() );
            nv->SetLayerPair( GetBoardLayerFromPNSLayer( via->Layers().Start() ),
                              GetBoardLayerFromPNSLayer( via->Layers().End() ) );
            return nv;
        }
        default:
            return nullptr;   // SOLID_T (pad) — never created headlessly
        }
    }

    // Mutate an existing board item in place from its shoved PNS counterpart.
    // Mirrors PNS_KICAD_IFACE::modifyBoardItem minus the BOARD_COMMIT->Modify.
    void modifyBoardItem( PNS::ITEM* aItem )
    {
        BOARD_ITEM* bi = aItem->Parent();

        if( !bi )
            return;

        switch( aItem->Kind() )
        {
        case PNS::ITEM::ARC_T:
        {
            PNS::ARC*        arc = static_cast<PNS::ARC*>( aItem );
            PCB_ARC*         ab = static_cast<PCB_ARC*>( bi );
            const SHAPE_ARC* as = static_cast<const SHAPE_ARC*>( arc->Shape( -1 ) );
            ab->SetStart( VECTOR2I( as->GetP0() ) );
            ab->SetEnd( VECTOR2I( as->GetP1() ) );
            ab->SetMid( VECTOR2I( as->GetArcMid() ) );
            ab->SetWidth( arc->Width() );
            break;
        }
        case PNS::ITEM::SEGMENT_T:
        {
            PNS::SEGMENT* seg = static_cast<PNS::SEGMENT*>( aItem );
            PCB_TRACK*    tr = static_cast<PCB_TRACK*>( bi );
            const SEG&    s = seg->Seg();
            tr->SetStart( VECTOR2I( s.A.x, s.A.y ) );
            tr->SetEnd( VECTOR2I( s.B.x, s.B.y ) );
            tr->SetWidth( seg->Width() );
            break;
        }
        case PNS::ITEM::VIA_T:
        {
            PNS::VIA* via = static_cast<PNS::VIA*>( aItem );
            PCB_VIA*  vb = static_cast<PCB_VIA*>( bi );
            vb->SetPosition( VECTOR2I( via->Pos().x, via->Pos().y ) );
            vb->SetWidth( PADSTACK::ALL_LAYERS, via->Diameter( 0 ) );
            vb->SetDrill( via->Drill() );
            break;
        }
        default:
            break;   // pad SOLID move — not supported headlessly
        }
    }

    std::vector<BOARD_ITEM*> m_toAdd;
    std::vector<BOARD_ITEM*> m_toRemove;
    int m_addedCount;
    int m_removedCount;
    int m_updatedCount;
};


// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static PAD* findPad( BOARD* aBoard, const wxString& aRef )
{
    // aRef = "REFDES.PADNAME", e.g. "U6.A4"
    int dot = aRef.Find( '.' );

    if( dot == wxNOT_FOUND )
        return nullptr;

    wxString refdes = aRef.Left( dot );
    wxString padName = aRef.Mid( dot + 1 );

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        if( fp->GetReference() != refdes )
            continue;

        for( PAD* pad : fp->Pads() )
        {
            if( pad->GetNumber() == padName )
                return pad;
        }
    }

    return nullptr;
}


// Resolve the PNS::ITEM (SOLID for the pad, or a track/via already on the net)
// for a given board pad + position.  Prefer the exact parent->item map used by
// KiCad's own headless PNS log player; fall back to a geometric hover query
// (the headless equivalent of TOOL_BASE::pickSingleItem).
//
// For a BGA escape, the F.Cu pad drops to the target inner layer through an
// escape VIA. The route on the inner layer must anchor to that via, not the
// F.Cu pad — otherwise FixRoute "completes" near the pad XY without actually
// joining the net's inner-layer copper. So we prefer, in order:
//   1. the net's own VIA that reaches the target layer, nearest aWhere
//   2. any of the net's copper on the target layer (track/arc), nearest aWhere
//   3. the pad SOLID via FindItemByParent (fallback)
static PNS::ITEM* pickItem( PNS::ROUTER* aRouter, HEADLESS_PNS_IFACE* aIface,
                            const BOARD_ITEM* aParent, const VECTOR2I& aWhere,
                            PNS::NET_HANDLE aNet, int aPnsLayer )
{
    PNS::ITEM*  bestVia = nullptr;
    PNS::ITEM*  bestOnLayer = nullptr;
    SEG::ecoord bestViaDist = VECTOR2I::ECOORD_MAX;
    SEG::ecoord bestLayDist = VECTOR2I::ECOORD_MAX;

    for( int slop : { 0, 300000, 600000 } )   // 0, ~0.3mm, ~0.6mm
    {
        PNS::ITEM_SET cand = aRouter->QueryHoverItems( aWhere, slop );

        for( PNS::ITEM* item : cand.Items() )
        {
            if( aNet && item->Net() != aNet )
                continue;

            if( !item->Layers().Overlaps( aPnsLayer ) )
                continue;

            SEG::ecoord d = ( item->Shape( -1 )->Centre() - aWhere ).SquaredEuclideanNorm();

            if( item->OfKind( PNS::ITEM::VIA_T ) && d < bestViaDist )
            {
                bestViaDist = d;
                bestVia = item;
            }
            else if( item->OfKind( PNS::ITEM::SEGMENT_T | PNS::ITEM::ARC_T )
                     && d < bestLayDist )
            {
                bestLayDist = d;
                bestOnLayer = item;
            }
        }

        if( bestVia || bestOnLayer )
            break;
    }

    if( bestVia )
        return bestVia;

    if( bestOnLayer )
        return bestOnLayer;

    // Fallback: the pad solid itself.
    if( aParent )
    {
        if( PNS::ITEM* byParent = aRouter->GetWorld()->FindItemByParent( aParent ) )
            return byParent;
    }

    return nullptr;
}


static void usage()
{
    fprintf( stderr,
        "pns-route — headless KiCad push-and-shove router\n"
        "Usage: pns-route board.kicad_pcb --net <net> --from <REF.PAD> --to <REF.PAD>\n"
        "                 --layer <LayerName> [--iter-limit N] [--mode shove|walkaround]\n"
        "                 [--waypoints \"x,y;x,y;...\" (mm)] [--width-mm W]\n"
        "                 [--clearance-mm C] [-o out.kicad_pcb]\n" );
}


int main( int argc, char** argv )
{
    wxInitializer wxinit;

    if( !wxinit.IsOk() )
    {
        fprintf( stderr, "pns-route: wx init failed\n" );
        return 2;
    }

    std::string boardPath, outPath, netName, fromRef, toRef, layerName, modeStr = "shove";
    std::string waypointStr;
    int iterLimit = 0;
    int shoveIters = 20000;     // headless: no GUI latency budget to respect
    int shoveMs = 20000;        // (stock 250 iters / 1000 ms is a UI guard)
    double widthMM = 0.0, clearanceMM = 0.0;

    std::vector<std::string> args( argv + 1, argv + argc );

    for( size_t i = 0; i < args.size(); ++i )
    {
        auto next = [&]() -> std::string {
            if( i + 1 >= args.size() ) { usage(); exit( 2 ); }
            return args[++i];
        };

        const std::string& a = args[i];

        if( a == "--net" )            netName = next();
        else if( a == "--from" )      fromRef = next();
        else if( a == "--to" )        toRef = next();
        else if( a == "--layer" )     layerName = next();
        else if( a == "--iter-limit" ) iterLimit = std::stoi( next() );
        else if( a == "--shove-iters" ) shoveIters = std::stoi( next() );
        else if( a == "--shove-ms" )  shoveMs = std::stoi( next() );
        else if( a == "--width-mm" )  widthMM = std::stod( next() );
        else if( a == "--clearance-mm" ) clearanceMM = std::stod( next() );
        else if( a == "--mode" )      modeStr = next();
        else if( a == "--waypoints" ) waypointStr = next();
        else if( a == "-o" || a == "--out" ) outPath = next();
        else if( a == "-h" || a == "--help" ) { usage(); return 0; }
        else if( boardPath.empty() && a[0] != '-' ) boardPath = a;
        else { fprintf( stderr, "pns-route: unknown arg '%s'\n", a.c_str() ); usage(); return 2; }
    }

    if( boardPath.empty() || fromRef.empty() || toRef.empty() || layerName.empty() )
    {
        usage();
        return 2;
    }

    if( outPath.empty() )
        outPath = boardPath;

    // --- Load board (headless sexpr IO) ---------------------------------
    PCB_IO_KICAD_SEXPR io;
    BOARD* board = nullptr;

    try
    {
        board = io.LoadBoard( wxString::FromUTF8( boardPath ), nullptr, nullptr );
    }
    catch( const std::exception& e )
    {
        fprintf( stderr, "pns-route: failed to load board: %s\n", e.what() );
        return 1;
    }

    if( !board )
    {
        fprintf( stderr, "pns-route: LoadBoard returned null\n" );
        return 1;
    }

    board->BuildListOfNets();
    board->BuildConnectivity();

    // --- Initialise the DRC engine (MANDATORY) --------------------------
    // PCB_IO::LoadBoard() does NOT initialise the DRC engine, and it does NOT
    // load the .kicad_pro (netclasses live in the project, not the .kicad_pcb).
    // With a null engine PNS_PCBNEW_RULE_RESOLVER::QueryConstraint returns false
    // and Clearance() falls back to its `int rv = 0` initialiser: the router
    // then lays copper at 0.00000 mm from foreign nets and reports success.
    // Mirror BOARD_LOADER::initializeLoadedBoard / qa pns_log_file.cpp so the
    // resolver PNS queries is the REAL board+DRU resolver. Fail loud if the
    // project is missing — routing blind is never acceptable.
    wxFileName projFn( wxString::FromUTF8( boardPath ) );
    projFn.SetExt( FILEEXT::ProjectFileExtension );

    if( !projFn.FileExists() )
    {
        fprintf( stderr,
                 "pns-route: FATAL: project file '%s' not found next to the board.\n"
                 "  Netclasses/clearances come from the .kicad_pro; without it the DRC\n"
                 "  engine has no rules and PNS would route at 0.00 mm clearance.\n"
                 "  Refusing to route blind.\n",
                 projFn.GetFullPath().ToUTF8().data() );
        return 1;
    }

    SETTINGS_MANAGER settingsMgr( true /* headless */ );

    if( !settingsMgr.LoadProject( projFn.GetFullPath() ) )
    {
        fprintf( stderr, "pns-route: FATAL: failed to load project '%s'\n",
                 projFn.GetFullPath().ToUTF8().data() );
        return 1;
    }

    PROJECT* project = settingsMgr.GetProject( projFn.GetFullPath() );

    if( !project )
    {
        fprintf( stderr, "pns-route: FATAL: GetProject returned null for '%s'\n",
                 projFn.GetFullPath().ToUTF8().data() );
        return 1;
    }

    // Read-only so SaveProject can never rewrite the user's .kicad_pro.
    project->SetReadOnly();
    board->SetProject( project );

    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    std::shared_ptr<DRC_ENGINE> drcEngine = std::make_shared<DRC_ENGINE>();
    bds.m_DRCEngine = drcEngine;
    bds.m_UseConnectedTrackWidth = project->GetLocalSettings().m_AutoTrackWidth;

    board->SynchronizeNetsAndNetClasses( true );

    drcEngine->SetBoard( board );
    drcEngine->SetDesignSettings( &bds );

    // DRU rules sit next to the board as <board>.kicad_dru (optional — the
    // netclass clearances from the project still apply without it).
    wxFileName druFn( wxString::FromUTF8( boardPath ) );
    druFn.SetExt( FILEEXT::DesignRulesFileExtension );

    try
    {
        if( druFn.FileExists() )
            drcEngine->InitEngine( druFn );
        else
            drcEngine->InitEngine( wxFileName() );
    }
    catch( const std::exception& e )
    {
        fprintf( stderr, "pns-route: FATAL: DRC InitEngine failed: %s\n", e.what() );
        return 1;
    }

    fprintf( stderr, "pns-route: DRC engine initialised (project='%s' dru=%s)\n",
             projFn.GetFullName().ToUTF8().data(),
             druFn.FileExists() ? druFn.GetFullName().ToUTF8().data() : "(none)" );

    // --- Resolve endpoints ----------------------------------------------
    PAD* fromPad = findPad( board, wxString::FromUTF8( fromRef ) );
    PAD* toPad   = findPad( board, wxString::FromUTF8( toRef ) );

    if( !fromPad ) { fprintf( stderr, "pns-route: --from pad '%s' not found\n", fromRef.c_str() ); return 1; }
    if( !toPad )   { fprintf( stderr, "pns-route: --to pad '%s' not found\n", toRef.c_str() ); return 1; }

    PCB_LAYER_ID layer = board->GetLayerID( wxString::FromUTF8( layerName ) );

    if( layer == UNDEFINED_LAYER || !IsCopperLayer( layer ) )
    {
        fprintf( stderr, "pns-route: layer '%s' is not a copper layer\n", layerName.c_str() );
        return 1;
    }

    VECTOR2I startPos = fromPad->GetPosition();
    VECTOR2I endPos   = toPad->GetPosition();
    NETINFO_ITEM* net = fromPad->GetNet();

    if( !netName.empty() && net && net->GetNetname() != wxString::FromUTF8( netName ) )
        fprintf( stderr, "pns-route: warning: --from pad net '%s' != --net '%s'\n",
                 net->GetNetname().ToUTF8().data(), netName.c_str() );

    // --- CLEARANCE_PROBE: prove the engine is live ----------------------
    // Ask the SAME resolver PNS uses for the electrical clearance between the
    // routed net's start pad and a real FOREIGN pad on the route layer. A null
    // engine returns 0; a live engine returns the netclass/DRU rule (~0.1 mm on
    // this board). This is the smoking-gun the previous blind builds lacked.
    {
        PAD* foreignPad = nullptr;

        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* p : fp->Pads() )
            {
                if( p->GetNetCode() != fromPad->GetNetCode() && p->GetNetCode() > 0
                    && p->IsOnLayer( layer ) )
                {
                    foreignPad = p;
                    break;
                }
            }

            if( foreignPad )
                break;
        }

        if( foreignPad )
        {
            DRC_CONSTRAINT c = bds.m_DRCEngine->EvalRules( CLEARANCE_CONSTRAINT,
                                                           fromPad, foreignPad, layer );
            double mm = c.GetValue().HasMin() ? c.GetValue().Min() / 1e6 : -1.0;
            fprintf( stderr,
                     "CLEARANCE_PROBE=%.5f mm  (net='%s' pad=%s.%s vs foreign net='%s' "
                     "pad=%s.%s on %s)\n",
                     mm,
                     net ? net->GetNetname().ToUTF8().data() : "?",
                     fromPad->GetParentFootprint()->GetReference().ToUTF8().data(),
                     fromPad->GetNumber().ToUTF8().data(),
                     foreignPad->GetNet() ? foreignPad->GetNet()->GetNetname().ToUTF8().data() : "?",
                     foreignPad->GetParentFootprint()->GetReference().ToUTF8().data(),
                     foreignPad->GetNumber().ToUTF8().data(),
                     layerName.c_str() );
        }
        else
        {
            fprintf( stderr, "CLEARANCE_PROBE=n/a (no foreign pad on layer %s)\n",
                     layerName.c_str() );
        }
    }

    // --- Wire up the headless router ------------------------------------
    HEADLESS_PNS_IFACE iface;
    iface.SetBoard( board );

    PNS::ROUTER router;
    router.SetInterface( &iface );

    // Settings MUST be loaded before Settings() is touched (ctor leaves it null).
    PNS::ROUTING_SETTINGS settings( nullptr, "tools.pns" );
    settings.SetMode( modeStr == "walkaround" ? PNS::RM_Walkaround : PNS::RM_Shove );
    // NEVER shove vias. SHOVE::onCollidingVia skips a via only if ShoveVias()
    // is false OR the via IsLocked(); locking just our own endpoints leaves
    // every FOREIGN escape via shoveable, and routing one net has moved a
    // neighbour's via-in-pad 0.100 mm off its ball centre (12 manufactured
    // shorts). SetShoveVias(false) is the reliable lever — the lock guard can
    // be bypassed by a zero-force livelock bail before it is reached.
    settings.SetShoveVias( false );
    settings.SetRemoveLoops( true );
    settings.SetShoveIterationLimit( shoveIters );
    settings.SetShoveTimeLimit( shoveMs );
    router.LoadSettings( &settings );

    router.SetMode( PNS::PNS_MODE_ROUTE_SINGLE );

    if( iterLimit > 0 )
        router.SetIterLimit( iterLimit );

    router.ClearWorld();
    router.SyncWorld();

    int pnsLayer = iface.GetPNSLayerFromBoardLayer( layer );
    iface.SetStartLayerFromPNS( pnsLayer );

    // Sizes (track width / clearance) from the board, then optional overrides.
    PNS::SIZES_SETTINGS sizes( router.Sizes() );
    PNS::ITEM* startItem = pickItem( &router, &iface, fromPad, startPos, net, pnsLayer );

    iface.ImportSizes( sizes, startItem, net, startPos );

    if( widthMM > 0.0 )
        sizes.SetTrackWidth( static_cast<int>( widthMM * 1e6 ) );   // mm -> nm

    if( clearanceMM > 0.0 )
        sizes.SetClearance( static_cast<int>( clearanceMM * 1e6 ) );

    sizes.AddLayerPair( pnsLayer, pnsLayer );
    router.UpdateSizes( sizes );

    if( !startItem )
    {
        fprintf( stderr, "pns-route: could not find a start item (pad/track) at %s on %s\n",
                 fromRef.c_str(), layerName.c_str() );
        return 1;
    }

    PNS::ITEM* endItem = pickItem( &router, &iface, toPad, endPos, net, pnsLayer );

    // Anchor the endpoints to the actual resolved item centres (the inner-layer
    // escape vias), not the F.Cu pad XY, so the route starts and finishes on the
    // net's inner-layer copper. Lock those vias so shove cannot displace our own
    // endpoints out from under the route.
    if( startItem->OfKind( PNS::ITEM::VIA_T ) )
    {
        startPos = startItem->Shape( -1 )->Centre();
        startItem->Mark( startItem->Marker() | PNS::MK_LOCKED );
    }

    if( endItem && endItem->OfKind( PNS::ITEM::VIA_T ) )
    {
        endPos = endItem->Shape( -1 )->Centre();
        endItem->Mark( endItem->Marker() | PNS::MK_LOCKED );
    }

    // --- Build the hop list: waypoints (mm, board coords) then the target ---
    // The interactive engine places one "head" toward the cursor; a single
    // straight Move-to-target stalls against obstacles it can't shove past in
    // that direction. A coarse global planner supplies waypoints that steer the
    // head around FIXED copper while trusting PNS to shove MOVEABLE traces. We
    // drive each hop like the GUI user's per-corner click: Move to settle, then
    // FixRoute(hop, forceFinish=false) to lock the leader and continue.
    std::vector<VECTOR2I> hops;

    if( !waypointStr.empty() )
    {
        // Format: "x,y;x,y;..." in mm (board coordinates).
        std::string s = waypointStr;
        size_t pos = 0;

        while( pos < s.size() )
        {
            size_t semi = s.find( ';', pos );
            std::string tok = s.substr( pos, semi == std::string::npos ? std::string::npos : semi - pos );
            size_t comma = tok.find( ',' );

            if( comma != std::string::npos )
            {
                double mx = std::stod( tok.substr( 0, comma ) );
                double my = std::stod( tok.substr( comma + 1 ) );
                hops.emplace_back( static_cast<int>( mx * 1e6 ), static_cast<int>( my * 1e6 ) );
            }

            if( semi == std::string::npos )
                break;

            pos = semi + 1;
        }
    }

    hops.push_back( endPos );   // final hop is always the destination pad

    // --- Route with shove, hop by hop -----------------------------------
    if( !router.StartRouting( startPos, startItem, pnsLayer ) )
    {
        fprintf( stderr, "pns-route: StartRouting failed: %s\n",
                 router.FailureReason().ToUTF8().data() );
        return 3;
    }

    const int reachTol = std::max( 1, sizes.TrackWidth() );   // "arrived" tolerance
    int  totalMoves = 0;
    int  stalledHop = -1;                                     // -1 == none
    int  skippedHops = 0;
    int  hopsReached = 0;
    VECTOR2I stallPos;
    bool targetReached = false;

    // Walk the hop list. On a stall at hop h, skip ahead to h+1 and let PNS aim
    // there — a single grid waypoint can land just inside copper, but the next
    // one usually gives a clear line. Only when NO remaining waypoint (incl. the
    // target) can be reached do we declare the head boxed in.
    for( size_t h = 0; h < hops.size() && router.RoutingInProgress(); ++h )
    {
        const VECTOR2I&   hop = hops[h];
        const bool        isLast = ( h + 1 == hops.size() );
        PNS::ITEM*        hopEnd = isLast ? endItem : nullptr;
        bool              hopReached = false;
        VECTOR2I          lastHead;
        bool              haveLast = false;
        int               noProgress = 0;

        for( int step = 0; step < 48 && router.RoutingInProgress(); ++step )
        {
            router.Move( hop, hopEnd );
            totalMoves++;

            PNS::PLACEMENT_ALGO* placer = router.Placer();

            if( !placer )
                break;

            VECTOR2I head = placer->CurrentEnd();

            if( getenv( "PNS_DEBUG" ) )
                fprintf( stderr, "    hop %zu step %d: move->(%.3f,%.3f) head=(%.3f,%.3f) added=%d\n",
                         h, step, hop.x / 1e6, hop.y / 1e6, head.x / 1e6, head.y / 1e6, iface.Added() );

            if( ( head - hop ).EuclideanNorm() <= reachTol )
            {
                hopReached = true;
                break;
            }

            if( haveLast && ( head - lastHead ).EuclideanNorm() <= reachTol )
            {
                if( ++noProgress >= 2 )
                    break;
            }
            else
            {
                noProgress = 0;
            }

            lastHead = head;
            haveLast = true;
        }

        if( hopReached )
        {
            // Lock the leader up to this reached corner and keep routing
            // (forceFinish only when this is the destination pad).
            if( router.RoutingInProgress() )
                router.FixRoute( hop, hopEnd, /*forceFinish*/ isLast, /*forceCommit*/ isLast );

            hopsReached++;

            if( isLast )
                targetReached = true;
        }
        else
        {
            // Stalled reaching this waypoint. Remember the first stall for the
            // diagnostic, then skip ahead (do NOT FixRoute a corner we didn't
            // reach — that would lock a bad partial segment).
            if( stalledHop < 0 )
            {
                stalledHop = static_cast<int>( h );
                stallPos = router.Placer() ? router.Placer()->CurrentEnd() : startPos;
            }

            skippedHops++;

            // If we just failed the final (target) hop, try one last forced
            // finish from wherever the head is — sometimes the last pinch is
            // within the router's own snap range.
            if( isLast && router.RoutingInProgress() )
            {
                if( router.FixRoute( hop, hopEnd, /*forceFinish*/ true, /*forceCommit*/ true ) )
                    targetReached = true;
            }
        }
    }

    bool reached = targetReached;

    router.CommitRouting();

    // Honest success gate. FixRoute can snap-return true while placing almost
    // nothing, and skip-ahead can walk to the target after skipping the whole
    // corridor — both look "reached" but leave the two escape vias unjoined.
    // Require: target reached, real copper laid, AND the head actually traversed
    // most of the corridor (majority of waypoints reached, not skipped). This is
    // a heuristic; DRC connectivity is the authoritative check downstream.
    const size_t hopCount = hops.size();
    const bool   traversed = ( hopsReached * 2 >= static_cast<int>( hopCount ) );
    bool completed = reached && traversed && iface.Added() >= 2;

    printf( "pns-route: mode=%s net=%s from=%s to=%s layer=%s -> completed=%s "
            "(reached=%s traversed=%d/%zu moves=%d added=%d removed=%d updated=%d)\n",
            modeStr.c_str(), netName.c_str(), fromRef.c_str(), toRef.c_str(), layerName.c_str(),
            completed ? "yes" : "no", reached ? "yes" : "no", hopsReached, hopCount,
            totalMoves, iface.Added(), iface.Removed(), iface.Updated() );

    if( !completed )
    {
        if( stalledHop >= 0 )
            fprintf( stderr, "pns-route: STALLED at hop %d/%zu near (%.3f, %.3f) mm — %s\n",
                     stalledHop + 1, hops.size(),
                     stallPos.x / 1e6, stallPos.y / 1e6,
                     ( stalledHop + 1 == static_cast<int>( hops.size() ) )
                         ? "head could not reach target pad"
                         : "head boxed in before reaching waypoint (fixed geometry or bad coarse path)" );
        else
            fprintf( stderr, "pns-route: connection NOT completed (no copper laid)\n" );
    }

    bool fixed = completed;

    // --- Save board ------------------------------------------------------
    try
    {
        io.SaveBoard( wxString::FromUTF8( outPath ), board, nullptr );
    }
    catch( const std::exception& e )
    {
        fprintf( stderr, "pns-route: failed to save board: %s\n", e.what() );
        return 1;
    }

    printf( "pns-route: wrote %s\n", outPath.c_str() );
    return fixed ? 0 : 4;
}
