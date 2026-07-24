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
static PNS::ITEM* pickItem( PNS::ROUTER* aRouter, HEADLESS_PNS_IFACE* aIface,
                            const BOARD_ITEM* aParent, const VECTOR2I& aWhere,
                            PNS::NET_HANDLE aNet, int aPnsLayer )
{
    if( aParent )
    {
        if( PNS::ITEM* byParent = aRouter->GetWorld()->FindItemByParent( aParent ) )
            return byParent;
    }

    PNS::ITEM*  best = nullptr;
    SEG::ecoord bestDist = VECTOR2I::ECOORD_MAX;

    for( int slop : { 0, 250000 } )   // 0, then ~0.25mm slop
    {
        PNS::ITEM_SET cand = aRouter->QueryHoverItems( aWhere, slop );

        for( PNS::ITEM* item : cand.Items() )
        {
            if( !item->IsRoutable() )
                continue;

            if( !aIface->IsPNSCopperLayer( item->Layers().Start() ) )
                continue;

            if( aNet && item->Net() != aNet )
                continue;

            if( !item->Layers().Overlaps( aPnsLayer )
                    && !item->OfKind( PNS::ITEM::VIA_T | PNS::ITEM::SOLID_T ) )
                continue;

            SEG::ecoord d = ( item->Shape( -1 )->Centre() - aWhere ).SquaredEuclideanNorm();

            if( d < bestDist )
            {
                bestDist = d;
                best = item;
            }
        }

        if( best )
            break;
    }

    return best;
}


static void usage()
{
    fprintf( stderr,
        "pns-route — headless KiCad push-and-shove router\n"
        "Usage: pns-route board.kicad_pcb --net <net> --from <REF.PAD> --to <REF.PAD>\n"
        "                 --layer <LayerName> [--iter-limit N] [--mode shove|walkaround]\n"
        "                 [--width-mm W] [--clearance-mm C] [-o out.kicad_pcb]\n" );
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
    int iterLimit = 0;
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
        else if( a == "--width-mm" )  widthMM = std::stod( next() );
        else if( a == "--clearance-mm" ) clearanceMM = std::stod( next() );
        else if( a == "--mode" )      modeStr = next();
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

    // --- Wire up the headless router ------------------------------------
    HEADLESS_PNS_IFACE iface;
    iface.SetBoard( board );

    PNS::ROUTER router;
    router.SetInterface( &iface );

    // Settings MUST be loaded before Settings() is touched (ctor leaves it null).
    PNS::ROUTING_SETTINGS settings( nullptr, "tools.pns" );
    settings.SetMode( modeStr == "walkaround" ? PNS::RM_Walkaround : PNS::RM_Shove );
    settings.SetShoveVias( true );
    settings.SetRemoveLoops( true );
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

    // --- Route with shove ------------------------------------------------
    // The interactive engine places a "head" toward the cursor on each Move;
    // a single Move rarely threads a whole congested path. So we drive it like
    // the GUI user does: repeated Move steps toward the target, letting the
    // head advance / shove, then FixRoute to commit. We stop early when the
    // head reaches the target (CurrentEnd within one track width).
    if( !router.StartRouting( startPos, startItem, pnsLayer ) )
    {
        fprintf( stderr, "pns-route: StartRouting failed: %s\n",
                 router.FailureReason().ToUTF8().data() );
        return 3;
    }

    const int reachTol = std::max( 1, sizes.TrackWidth() );   // "arrived" tolerance
    int  moveSteps = 0;
    bool reached = false;

    for( int step = 0; step < 64; ++step )
    {
        router.Move( endPos, endItem );
        moveSteps++;

        if( !router.RoutingInProgress() )
            break;

        if( PNS::PLACEMENT_ALGO* placer = router.Placer() )
        {
            VECTOR2I head = placer->CurrentEnd();

            if( ( head - endPos ).EuclideanNorm() <= reachTol )
            {
                reached = true;
                break;
            }
        }
    }

    bool fixOk = router.RoutingInProgress()
                     && router.FixRoute( endPos, endItem, /*forceFinish*/ true, /*forceCommit*/ true );

    router.CommitRouting();

    // Honest success gate: copper must have actually been laid AND the head
    // must have reached the target pad.  FixRoute alone can return true while
    // placing nothing, so we do not trust it by itself.
    bool completed = fixOk && reached && iface.Added() > 0;

    printf( "pns-route: mode=%s from=%s to=%s layer=%s -> completed=%s "
            "(reached=%s fix=%s moves=%d added=%d removed=%d updated=%d)\n",
            modeStr.c_str(), fromRef.c_str(), toRef.c_str(), layerName.c_str(),
            completed ? "yes" : "no", reached ? "yes" : "no", fixOk ? "yes" : "no",
            moveSteps, iface.Added(), iface.Removed(), iface.Updated() );

    if( !completed )
    {
        fprintf( stderr, "pns-route: connection NOT completed (%s)\n",
                 router.FailureReason().IsEmpty() ? "head did not reach target / no copper laid"
                                                   : router.FailureReason().ToUTF8().data() );
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
