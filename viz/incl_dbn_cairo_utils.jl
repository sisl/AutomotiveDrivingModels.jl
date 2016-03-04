# include(Pkg.dir("AutomotiveDrivingModels", "viz", "incl_dbn_cairo_utils.jl"))


function plot_dbn_indicators(
    runlog::RunLog,
    sn::StreetNetwork,
    BN::DynamicBayesianNetworkBehavior,
    colset::UInt,
    frame::Integer;
    fontsize::Integer=15, # [pix]
    linespacing::Integer=2, # [pix]
    canvas_width::Integer=1100, # [pix]
    )

    dbn = BN.model
    indicators_lat = get_indicators_lat(dbn)
    indicators_lon = get_indicators_lon(dbn)

    nlines_lat = 2+length(indicators_lat)
    nlines_lon = 2+length(indicators_lon)
    nlines = max(nlines_lat, nlines_lon)
    canvas_height = fontsize*nlines + (nlines+1)*linespacing


    observations = BN.observations
    assignment = BN.assignment
    observe!(BN.extractor, runlog, sn, colset, frame)
    DynamicBayesianNetworkBehaviors._copy_extracted_into_obs!(BN)
    DynamicBayesianNetworkBehaviors.encode!(assignment, dbn, observations)

    rendermodel = RenderModel()

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)

    #-------------------------------------

    textcolor = colorant"white"

    cursor_x = linespacing
    cursor_y = fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "Lat", "Val", "Bin"),
                                 cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)
    cursor_y += fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "-"^20, "-"^20, "-"^8),
                                  cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)
    cursor_y += fontsize + linespacing
    for feature in indicators_lat
        sym = symbol(feature)
        val = observations[sym]
        bin = assignment[sym]
        add_instruction!(rendermodel, render_text, (@sprintf("%-24s %20.8f %8d", string(sym), val, bin),
                                  cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)
        cursor_y += fontsize + linespacing
    end

    #-------------------------------------

    cursor_x = canvas_width/2 + linespacing
    cursor_y = fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "Lon", "Val", "Bin"),
                                 cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)

    cursor_y += fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "-"^20, "-"^20, "-"^8),
                                  cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)
    cursor_y += fontsize + linespacing
    for feature in indicators_lon
        sym = symbol(feature)
        val = observations[sym]
        bin = assignment[sym]
        add_instruction!(rendermodel, render_text, (@sprintf("%-24s %20.8f %8d", string(sym), val, bin),
                                  cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)
        cursor_y += fontsize + linespacing
    end

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end
    # function plot_dbn_indicators(
    #     basics::FeatureExtractBasicsPdSet,
    #     behavior::DynamicBayesianNetworkBehavior,
    #     carind::Int,
    #     validfind::Int;
    #     fontsize::Integer=15, # [pix]
    #     linespacing::Integer=2, # [pix]
    #     canvas_width::Integer=1100, # [pix]
    #     )

    #     dbn = behavior.model
    #     indicators_lat = get_indicators_lat(dbn)
    #     indicators_lon = get_indicators_lon(dbn)

    #     nlines_lat = 2+length(indicators_lat)
    #     nlines_lon = 2+length(indicators_lon)
    #     nlines = max(nlines_lat, nlines_lon)
    #     canvas_height = fontsize*nlines + (nlines+1)*linespacing

    #     observations = observe(basics, carind, validfind, behavior.indicators)
    #     assignment = encode(dbn, observations)

    #     rendermodel = RenderModel()

    #     s = CairoRGBSurface(canvas_width, canvas_height)
    #     ctx = creategc(s)
    #     clear_setup(rendermodel)

    #     #-------------------------------------

    #     cursor_x = linespacing
    #     cursor_y = fontsize + linespacing
    #     add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "Lat", "Val", "Bin"),
    #                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    #     cursor_y += fontsize + linespacing
    #     add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "-"^20, "-"^20, "-"^8),
    #                                   cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    #     cursor_y += fontsize + linespacing
    #     for feature in indicators_lat
    #         sym = symbol(feature)
    #         val = observations[sym]
    #         bin = assignment[sym]
    #         add_instruction!(rendermodel, render_text, (@sprintf("%-24s %20.8f %8d", string(sym), val, bin),
    #                                   cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    #         cursor_y += fontsize + linespacing
    #     end

    #     #-------------------------------------

    #     cursor_x = canvas_width/2 + linespacing
    #     cursor_y = fontsize + linespacing
    #     add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "Lon", "Val", "Bin"),
    #                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    #     cursor_y += fontsize + linespacing
    #     add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "-"^20, "-"^20, "-"^8),
    #                                   cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    #     cursor_y += fontsize + linespacing
    #     for feature in indicators_lon
    #         sym = symbol(feature)
    #         val = observations[sym]
    #         bin = assignment[sym]
    #         add_instruction!(rendermodel, render_text, (@sprintf("%-24s %20.8f %8d", string(sym), val, bin),
    #                                   cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    #         cursor_y += fontsize + linespacing
    #     end

    #     render(ctx, canvas_width, canvas_height, rendermodel)
    #     s
    # end
function plot_counts_over_conditioned_bin(
    runlog::RunLog,
    sn::StreetNetwork,
    BN::DynamicBayesianNetworkBehavior,
    colset::UInt,
    frame::Integer;
    fontsize::Integer=15, # [pix]
    linespacing::Integer=2, # [pix]
    canvas_width::Integer=1100, # [pix]
    )

    model = BN.model
    bincounts = get_bin_counts(model)

    observations = BN.observations
    assignment = BN.assignment
    observe!(BN.extractor, runlog, sn, colset, frame)
    DynamicBayesianNetworkBehaviors._copy_extracted_into_obs!(BN)
    DynamicBayesianNetworkBehaviors.encode!(assignment, model, observations)

    index_lat = indexof(get_target_lat(model), model)
    counts_lat = get_counts_for_assignment(model, index_lat, assignment, bincounts)

    index_lon = indexof(get_target_lon(model), model)
    counts_lon = get_counts_for_assignment(model, index_lon, assignment, bincounts)

    canvas_height = fontsize + 2linespacing

    # -----------------------------------

    textcolor = colorant"white"

    rendermodel = RenderModel()

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)

    #-------------------------------------

    cursor_x = linespacing
    cursor_y = fontsize + linespacing

    add_instruction!(rendermodel, render_text, (string(round(Int, counts_lat)),
                                 cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)

    #-------------------------------------

    cursor_x = canvas_width/2 + linespacing
    cursor_y = fontsize + linespacing
    add_instruction!(rendermodel, render_text, (string(round(Int, counts_lon)),
                                 cursor_x, cursor_y, fontsize, textcolor, false, "monospace"), incameraframe=false)

    render(rendermodel, ctx, canvas_width, canvas_height)
    s
end
    # function plot_counts_over_conditioned_bin(
    #     basics::FeatureExtractBasicsPdSet,
    #     behavior::DynamicBayesianNetworkBehavior,
    #     carind::Int,
    #     validfind::Int;
    #     fontsize::Integer=15, # [pix]
    #     linespacing::Integer=2, # [pix]
    #     canvas_width::Integer=1100, # [pix]
    #     )

    #     model = behavior.model
    #     bincounts = get_bin_counts(model)

    #     observations = observe(basics, carind, validfind, behavior.indicators)
    #     assignment = encode(model, observations)

    #     index_lat = indexof(get_target_lat(model), model)
    #     counts_lat = get_counts_for_assignment(model, index_lat, assignment, bincounts)

    #     index_lon = indexof(get_target_lon(model), model)
    #     counts_lon = get_counts_for_assignment(model, index_lon, assignment, bincounts)

    #     canvas_height = fontsize + 2linespacing

    #     # -----------------------------------

    #     rm = RenderModel()

    #     s = CairoRGBSurface(canvas_width, canvas_height)
    #     ctx = creategc(s)
    #     clear_setup(rm)

    #     #-------------------------------------

    #     cursor_x = linespacing
    #     cursor_y = fontsize + linespacing

    #     add_instruction!(rm, render_text, (string(int(counts_lat)),
    #                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)

    #     #-------------------------------------

    #     cursor_x = canvas_width/2 + linespacing
    #     cursor_y = fontsize + linespacing
    #     add_instruction!(rm, render_text, (string(int(counts_lon)),
    #                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)

    #     render(rm, ctx, canvas_width, canvas_height)
    #     s
    # end

#############################################################
## OVERLAYS

"""
Renders text showing the BN counts for the lat/lon conditioned on
the features observed in the current frame.

ex:
counts lat: [0,250,800,0,0]
counts lon: [0,83,2458,0,0,1]

NOTE: prior counts are not included
"""
type BNCountsOverlay <: Overlay
    BN::DynamicBayesianNetworkBehavior
    fontsize::Int # [pix]
    linespacing::Int # [pix]
    x_margin::Int # [pix]

    function BNCountsOverlay(
        BN::DynamicBayesianNetworkBehavior;
        fontsize::Integer=15,
        linespacing::Integer=2,
        x_margin::Integer=5,
        )

        retval = new()
        retval.BN = BN
        retval.fontsize = fontsize
        retval.linespacing = linespacing
        retval.x_margin = x_margin
        retval
    end
end
function render_overlay!(overlay::BNCountsOverlay,
    rendermodel::RenderModel,
    runlog::RunLog,
    sn::StreetNetwork,
    frame::Integer,
    active_carid::Integer,
    )

    # ---------------

    BN = overlay.BN
    model = BN.model
    bincounts = get_bin_counts(model)
    colset = id2colset(runlog, active_carid, frame)

    observations = BN.observations
    assignment = BN.assignment
    observe!(BN.extractor, runlog, sn, colset, frame)
    DynamicBayesianNetworkBehaviors._copy_extracted_into_obs!(BN)
    DynamicBayesianNetworkBehaviors.encode!(assignment, model, observations)

    index_lat = indexof(get_target_lat(model), model)
    counts_lat = get_counts_for_assignment(model, index_lat, assignment, bincounts)
    indicators_lat = get_indicators_lat(model)

    index_lon = indexof(get_target_lon(model), model)
    counts_lon = get_counts_for_assignment(model, index_lon, assignment, bincounts)
    indicators_lon = get_indicators_lon(model)

    # ---------------

    textcolor = colorant"white"
    text_y_jump = overlay.fontsize + overlay.linespacing
    text_y = text_y_jump


    add_instruction!(rendermodel, render_text, (@sprintf("counts lat: %s", string(round(Int, counts_lat))),
                                 overlay.x_margin, text_y, overlay.fontsize, textcolor, false, "monospace"), incameraframe=false)
    text_y += text_y_jump

    #-------------------------------------

    add_instruction!(rendermodel, render_text, (@sprintf("counts lon: %s", string(round(Int, counts_lon))),
                                 overlay.x_margin, text_y, overlay.fontsize, textcolor, false, "monospace"), incameraframe=false)
    text_y += text_y_jump

    #-------------------------------------
    # render the variables

    for feature in indicators_lat
        sym = symbol(feature)
        val = observations[sym]
        bin = assignment[sym]
        add_instruction!(rendermodel, render_text, (@sprintf("lat: %-20s %12.6f %3d", string(sym), val, bin),
                                  overlay.x_margin, text_y, overlay.fontsize, textcolor, false, "monospace"), incameraframe=false)
        text_y += text_y_jump
    end
    for feature in indicators_lon
        sym = symbol(feature)
        val = observations[sym]
        bin = assignment[sym]
        add_instruction!(rendermodel, render_text, (@sprintf("lon: %-20s %12.6f %3d", string(sym), val, bin),
                                  overlay.x_margin, text_y, overlay.fontsize, textcolor, false, "monospace"), incameraframe=false)
        text_y += text_y_jump
    end

    rendermodel
end

nothing