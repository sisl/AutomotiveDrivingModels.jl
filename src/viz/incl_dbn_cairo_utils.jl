function plot_dbn_indicators(
    basics::FeatureExtractBasicsPdSet,
    behavior::DynamicBayesianNetworkBehavior,
    carind::Int,
    validfind::Int;
    fontsize::Integer=15, # [pix]
    linespacing::Integer=2, # [pix]
    canvas_width::Integer=1100, # [pix]
    )
    
    dbn = behavior.model
    indicators_lat = get_indicators_lat(dbn)
    indicators_lon = get_indicators_lon(dbn)

    nlines_lat = 2+length(indicators_lat)
    nlines_lon = 2+length(indicators_lon)
    nlines = max(nlines_lat, nlines_lon)
    canvas_height = fontsize*nlines + (nlines+1)*linespacing

    observations = observe(basics, carind, validfind, behavior.indicators)
    assignment = encode(dbn, observations)

    rm = RenderModel()

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup(rm)

    #-------------------------------------

    cursor_x = linespacing
    cursor_y = fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "Lat", "Val", "Bin"), 
                                 cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    cursor_y += fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "-"^20, "-"^20, "-"^8), 
                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    cursor_y += fontsize + linespacing
    for feature in indicators_lat
        sym = symbol(feature)
        val = observations[sym]
        bin = assignment[sym]
        add_instruction!(rendermodel, render_text, (@sprintf("%-24s %20.8f %8d", string(sym), val, bin), 
                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
        cursor_y += fontsize + linespacing
    end

    #-------------------------------------

    cursor_x = canvas_width/2 + linespacing
    cursor_y = fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "Lon", "Val", "Bin"), 
                                 cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    cursor_y += fontsize + linespacing
    add_instruction!(rendermodel, render_text, (@sprintf("%-24s %-20s %-8s", "-"^20, "-"^20, "-"^8), 
                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
    cursor_y += fontsize + linespacing
    for feature in indicators_lon
        sym = symbol(feature)
        val = observations[sym]
        bin = assignment[sym]
        add_instruction!(rendermodel, render_text, (@sprintf("%-24s %20.8f %8d", string(sym), val, bin), 
                                  cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)
        cursor_y += fontsize + linespacing
    end

    render(ctx, canvas_width, canvas_height, rm)
    s
end
function plot_counts_over_conditioned_bin(
    basics::FeatureExtractBasicsPdSet,
    behavior::DynamicBayesianNetworkBehavior,
    carind::Int,
    validfind::Int;
    fontsize::Integer=15, # [pix]
    linespacing::Integer=2, # [pix]
    canvas_width::Integer=1100, # [pix]
    )

    model = behavior.model
    bincounts = get_bin_counts(model)

    observations = observe(basics, carind, validfind, behavior.indicators)
    assignment = encode(model, observations)

    index_lat = indexof(get_target_lat(model), model)
    counts_lat = get_counts_for_assignment(model, index_lat, assignment, bincounts)

    index_lon = indexof(get_target_lon(model), model)
    counts_lon = get_counts_for_assignment(model, index_lon, assignment, bincounts)

    canvas_height = fontsize + 2linespacing

    # -----------------------------------

    rm = RenderModel()

    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    clear_setup(rm)

    #-------------------------------------

    cursor_x = linespacing
    cursor_y = fontsize + linespacing

    add_instruction!(rm, render_text, (string(int(counts_lat)), 
                                 cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)

    #-------------------------------------

    cursor_x = canvas_width/2 + linespacing
    cursor_y = fontsize + linespacing
    add_instruction!(rm, render_text, (string(int(counts_lon)), 
                                 cursor_x, cursor_y, fontsize, [1.0,1.0,1.0], false, "monospace"), false)

    render(rm, ctx, canvas_width, canvas_height)
    s
end