include_directories( BEFORE ${INC_BEFORE} )

include_directories(
    ./
    ../
    ../../include
    ../../3d-viewer
    ../../pcbnew
    ../../libs/kimath/include/
    ../../libs/core/include/
    ../../thirdparty/clipper2/Clipper2Lib/include
    ../../thirdparty/nlohmann_json
    ../../thirdparty/rtree
    ../../thirdparty/dynamic_bitset/
    ../../thirdparty/expected/include/
    ../../libs/kiplatform/include/
    ../../common

    ${INC_AFTER}
)

set( PCBNEW_PNS_SRCS
    time_limit.cpp
    pns_kicad_iface.cpp
    pns_algo_base.cpp
    pns_arc.cpp
    pns_component_dragger.cpp
    pns_diff_pair.cpp
    pns_diff_pair_placer.cpp
    pns_dp_meander_placer.cpp
    pns_dragger.cpp
    pns_helpers.cpp
    pns_hole.cpp
    pns_index.cpp
    pns_item.cpp
    pns_itemset.cpp
    pns_line.cpp
    pns_line_placer.cpp
    pns_logger.cpp
    pns_meander.cpp
    pns_meander_placer.cpp
    pns_meander_placer_base.cpp
    pns_meander_skew_placer.cpp
    pns_mouse_trail_tracer.cpp
    pns_node.cpp
    pns_optimizer.cpp
    pns_router.cpp
    pns_routing_settings.cpp
    pns_shove.cpp
    pns_sizes_settings.cpp
    pns_solid.cpp
    pns_tool_base.cpp
    pns_topology.cpp
    pns_utils.cpp
    pns_via.cpp
    pns_walkaround.cpp
    pns_multi_dragger.cpp
    router_preview_item.cpp
    router_status_view_item.cpp
    router_tool.cpp
)


set( PCBNEW_PNS_QA_SRCS
    time_limit.cpp
    pns_kicad_iface.cpp
    pns_algo_base.cpp
    pns_arc.cpp
    pns_component_dragger.cpp
    pns_diff_pair.cpp
    pns_diff_pair_placer.cpp
    pns_dp_meander_placer.cpp
    pns_dragger.cpp
    pns_hole.cpp
    pns_index.cpp
    pns_item.cpp
    pns_itemset.cpp
    pns_line.cpp
    pns_line_placer.cpp
    pns_logger.cpp
    pns_meander.cpp
    pns_meander_placer.cpp
    pns_meander_placer_base.cpp
    pns_meander_skew_placer.cpp
    pns_mouse_trail_tracer.cpp
    pns_node.cpp
    pns_optimizer.cpp
    pns_router.cpp
    pns_routing_settings.cpp
    pns_shove.cpp
    pns_sizes_settings.cpp
    pns_solid.cpp
    pns_topology.cpp
    pns_utils.cpp
    pns_via.cpp
    pns_walkaround.cpp
    pns_multi_dragger.cpp
    router_preview_item.cpp
)


add_library( pnsrouter STATIC ${PCBNEW_PNS_SRCS} )
add_library( pnsrouter_qa STATIC ${PCBNEW_PNS_QA_SRCS} )

if( KICAD_USE_PCH )
    target_precompile_headers( pnsrouter
        PRIVATE
            <board.h>
            <clipper2/clipper.h>
            <kiid.h>
            <pgm_base.h>
            <wx/wx.h>
            )

    target_precompile_headers( pnsrouter_qa
        PRIVATE
            <board.h>
            <clipper2/clipper.h>
            <kiid.h>
            <pgm_base.h>
            <wx/wx.h>
            )
endif()

target_link_libraries( pnsrouter PRIVATE
    common
)

target_link_libraries( pnsrouter_qa PRIVATE
    common
)

# ---------------------------------------------------------------------------
# Headless PNS CLI (calidran/kicad headless-pns spike).
# Standalone `pns-route` executable that drives push-and-shove with no GUI.
# Gated behind KICAD_BUILD_PNS_HEADLESS so a normal build is unaffected.
# Links the same backend set as qa/tools/pns (the proven headless PNS link
# recipe), minus the boost unit-test framework.
# ---------------------------------------------------------------------------
option( KICAD_BUILD_PNS_HEADLESS "Build the headless pns-route CLI" OFF )

if( KICAD_BUILD_PNS_HEADLESS )
    add_executable( pns-route
        pns_headless/pns_route_cli.cpp
    )

    target_compile_definitions( pns-route PRIVATE PCBNEW )

    add_dependencies( pns-route pcbnew )

    target_include_directories( pns-route PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/pcbnew
        ${CMAKE_SOURCE_DIR}/pcbnew/router
        ${CMAKE_SOURCE_DIR}/libs/kimath/include
    )

    # pcbnew_kiface_objects supplies Kiface(), the tool framework (PCB_TOOL_BASE /
    # PCB_SELECTION_TOOL) and the board-model glue that pcbcommon/pnsrouter
    # reference; the IO plugin libs supply the self-registering PCB_IO backends
    # (sexpr load/save + siblings). Order-independent target names.
    target_link_libraries( pns-route
        pcbnew_kiface_objects
        connectivity
        pcbcommon
        pnsrouter
        3d-viewer
        kicommon
        gal
        common
        core
        idf3
        markdown_lib
        dxflib_qcad
        tinyspline_lib
        nanosvg
        ${PCBNEW_IO_LIBRARIES}
        ${wxWidgets_LIBRARIES}
        ${GDI_PLUS_LIBRARIES}
        Boost::headers
        ${PCBNEW_EXTRA_LIBS}
    )
endif()
