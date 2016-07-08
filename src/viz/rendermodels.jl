export
        RenderModel,

        render,
        add_instruction!,
        camera_fit_to_content!,
        camera_move!,
        camera_move_pix!,
        camera_rotate!,
        camera_setrotation!,
        camera_zoom!,
        camera_setzoom!,
        camera_set_pos!,
        camera_set_x!,
        camera_set_y!,
        camera_reset!,
        camera_set!,
        clear_setup!,
        set_background_color!,

        render_text,
        render_circle,
        render_arc,
        render_round_rect,
        render_car,
        render_vehicle,
        render_point_trail,
        render_line,
        render_fill_region,
        render_line_segment,
        render_dashed_line,
        render_arrow,
        render_colormesh,

        get_surface_and_context

# config variables
type RenderModel
    instruction_set  :: Vector{Tuple}  # set of render instructions (function, array of inputs sans ctx, incameraframe)
    camera_center    :: VecE2          # position of camera in [N,E] relative to the mean point. meters
    camera_zoom      :: Float64        # [pix/m]
    camera_rotation  :: Float64        # [rad]
    background_color :: RGB

    RenderModel() = new(Array(Tuple,0), VecE2(0.0,0.0), 1.0, 0.0, RGB(0, 0, 0))
end

# Functions
# ===================================================

# primitivesposF
function render_text(
    ctx          :: CairoContext,
    text         :: AbstractString,
    x            :: Real,
    y            :: Real,
    fontsize     :: Real,
    color        :: Colorant,
    align_center :: Bool        = false,
    fontfamily   :: AbstractString      = "Sans" # ∈ "serif", "sans-serif", "cursive", "fantasy", "monospace"
    )

    save(ctx)
    select_font_face(ctx, fontfamily, Cairo.FONT_SLANT_NORMAL, Cairo.FONT_WEIGHT_NORMAL)
    set_font_size(ctx, fontsize)
    set_source_rgba(ctx, color)

    if align_center
        extents = text_extents(ctx, text)
        x -= (extents[3]/2 + extents[1]);
        y -= (extents[4]/2 + extents[2]);
    end

    move_to(ctx, x, y)
    show_text(ctx, text)
    restore(ctx)
end
function render_circle(
    ctx          :: CairoContext,
    x            :: Real,
    y            :: Real,
    radius       :: Real,
    color_fill   :: Colorant,
    color_stroke :: Colorant = color_fill,
    line_width   :: Real = 1.0
    )

    save(ctx)
    translate(ctx, x, y)

    arc(ctx, 0.0, 0.0, radius, 0, 2pi)
    set_source_rgba(ctx, color_fill)
    fill(ctx)

    arc(ctx, 0.0, 0.0, radius, 0, 2pi)
    set_line_width(ctx, line_width)
    set_source_rgba(ctx, color_stroke)
    stroke(ctx)

    restore(ctx)
end
function render_arc(
    ctx          :: CairoContext,
    x            :: Real,
    y            :: Real,
    radius       :: Real,
    start_ang    :: Real,
    end_ang      :: Real,
    color_fill   :: Colorant,
    color_stroke :: Colorant = color_fill,
    line_width   :: Real = 1.0
    )

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    translate(ctx, x, y)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_source_rgba(ctx, color_fill)
    fill(ctx)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_line_width(ctx, line_width)
    set_source_rgba(ctx, color_stroke)
    stroke(ctx)

    restore(ctx)
end
function render_round_rect(
    ctx           :: CairoContext,
    x             :: Real,
    y             :: Real,
    width         :: Real,
    height        :: Real,
    aspect        :: Real,
    corner_radius :: Real,
    color_fill    :: Colorant,
    ffill         :: Bool        = false,
    fstroke       :: Bool        = true,
    color_stroke  :: Colorant    = color_fill,
    line_width    :: Real        = 2.0
    )
    # (x,y) are the center of the rectangle

    save(ctx)
    radius = corner_radius / aspect

    d2r = pi/180

    new_sub_path(ctx)
    arc(ctx, x + width/2 - radius, y - height/2 + radius, radius, -90 * d2r,   0 * d2r)
    arc(ctx, x + width/2 - radius, y + height/2 - radius, radius,   0 * d2r,  90 * d2r)
    arc(ctx, x - width/2 + radius, y + height/2 - radius, radius,  90 * d2r, 180 * d2r)
    arc(ctx, x - width/2 + radius, y - height/2 + radius, radius, 180 * d2r, 270 * d2r)
    close_path(ctx)

    if ffill
        set_source_rgba(ctx, color_fill)
        fstroke ? fill_preserve(ctx) : fill(ctx)
    end

    if fstroke
        set_source_rgba(ctx, color_stroke)
        line_width = abs(user_to_device_distance!(ctx, [line_width,0.0])[1])
        set_line_width(ctx, line_width)
        stroke(ctx)
    end

    restore(ctx)
end
function render_car(
    ctx           :: CairoContext,
    x             :: Real, # center of vehicle [m]
    y             :: Real, # center of vehicle [m]
    yaw           :: Real, # counterclockwise [rad]
    color_fill    :: Colorant,
    color_stroke  :: Colorant = color_fill;

    color_arrow   :: Colorant = RGB(1.0,1.0,1.0),
    car_length    :: Float64 = 4.6, # [m]
    car_width     :: Float64 = 2.0, # [m]
    corner_radius :: Float64 = 0.3, # [m]
    corner_aspect :: Float64 = 1.0, # [m]
    arrow_width   :: Float64 = 0.3, # [% car width]
    arrow_length  :: Float64 = 0.6, # [% car length]
    arrow_headlen :: Float64 = 0.5, # [% arrow len that head takes up]
    line_width    :: Float64 = 0.1, # [m]
    )

    # renders a car (rounded rectangle w/ arrow) at the given location

    save(ctx)

    translate(ctx, x, y)
    rotate(ctx, yaw)

    render_round_rect(ctx, 0, 0, car_length, car_width, corner_aspect, corner_radius, color_fill, true, true, color_stroke, line_width)

    # render the arrow
    wid = car_width*arrow_width
    len = car_length*arrow_length
    hed = len*arrow_headlen

    new_sub_path(ctx)
    move_to(ctx, 0+len/2,     0      )
    line_to(ctx, 0+len/2-hed, 0+hed/2)
    line_to(ctx, 0+len/2-hed, 0+wid/2)
    line_to(ctx, 0-len/2,     0+wid/2)
    line_to(ctx, 0-len/2,     0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-hed/2)
    close_path(ctx)

    set_source_rgba(ctx, color_arrow)
    fill(ctx)

    restore(ctx)
end
function render_vehicle(
    ctx           :: CairoContext,
    x             :: Real, # x-pos of the center of the vehicle
    y             :: Real, # y-pos of the center of the vehicle
    yaw           :: Real, # heading angle [rad]
    length        :: Real, # vehicle length
    width         :: Real, # vehicle width
    color_fill    :: Colorant,
    color_stroke  :: Colorant = color_fill;

    color_arrow   :: Colorant = RGB(1.0,1.0,1.0),
    corner_radius :: Float64 = 0.5,
    corner_aspect :: Float64 = 1.0,
    arrow_width   :: Float64 = 0.3, # [% car width]
    arrow_length  :: Float64 = 0.6, # [% car length]
    arrow_headlen :: Float64 = 0.5, # [% arrow len that head takes up]
    line_width    :: Float64 = 0.3,
    )

    # renders a car (rounded rectangle w/ arrow) at the given location
    # (x,y) are in meters and yaw is the radians, counter-clockwise from pos x axis

    save(ctx)

    # translate(ctx, x - 0.5length*cos(yaw), y - 0.5length*sin(yaw))
    translate(ctx, x, y)
    rotate(ctx, yaw)

    render_round_rect(ctx, 0, 0, length, width, corner_aspect, corner_radius, color_fill, true, true, color_stroke, line_width)

    # render the arrow
    wid = width*arrow_width
    len = length*arrow_length
    hed = min(len*arrow_headlen, width)

    new_sub_path(ctx)
    move_to(ctx, 0+len/2,     0      )
    line_to(ctx, 0+len/2-hed, 0+hed/2)
    line_to(ctx, 0+len/2-hed, 0+wid/2)
    line_to(ctx, 0-len/2,     0+wid/2)
    line_to(ctx, 0-len/2,     0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-hed/2)
    close_path(ctx)

    set_source_rgba(ctx, color_arrow)
    fill(ctx)

    restore(ctx)
end

# aggregate
function render_point_trail{T<:Real}(
    ctx           :: CairoContext,
    pts           :: Matrix{T}, # 2×n
    color         :: Colorant,
    circle_radius :: Real = 0.25 )

    save(ctx)

    for i = 1 : size(pts,2)
        arc(ctx, pts[1,i], pts[2,i], circle_radius, 0, 2pi)
        set_line_width(ctx, 1.0)
        set_source_rgba(ctx, color)
        stroke(ctx)
    end

    restore(ctx)
end
function render_line{T<:Real}(
    ctx        :: CairoContext,
    pts        :: Matrix{T}, # 2×n
    color      :: Colorant,
    line_width :: Real = 1.0,
    line_cap   :: Integer=Cairo.CAIRO_LINE_CAP_ROUND, # CAIRO_LINE_CAP_BUTT, CAIRO_LINE_CAP_ROUND, CAIRO_LINE_CAP_SQUARE
    )

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, line_cap)

    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    stroke(ctx)
    restore(ctx)
end
function render_fill_region{T<:Real}(
    ctx        :: CairoContext,
    pts        :: Matrix{T}, # 2×n
    color      :: Colorant,
    )

    save(ctx)
    set_source_rgba(ctx,color)
    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    close_path(ctx)
    fill(ctx)
    restore(ctx)

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,1.0)
    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    close_path(ctx)
    stroke(ctx)
    restore(ctx)
end
function render_line_segment(
    ctx        :: CairoContext,
    x1         :: Float64,
    y1         :: Float64,
    x2         :: Float64,
    y2         :: Float64,
    color      :: Colorant,
    line_width :: Real = 1.0
    )

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)

    move_to(ctx, x1, y1)
    line_to(ctx, x2, y2)
    stroke(ctx)
    restore(ctx)
end
function render_dashed_line{T<:Real}(
    ctx          :: CairoContext,
    pts          :: Matrix{T}, # 2×n
    color        :: Colorant,
    line_width_in   :: Real = 1.0,
    dash_length_in  :: Real = 1.0,
    dash_spacing_in :: Real = 1.0,
    dash_offset_in  :: Real = 0.0
    )

    line_width   = user_to_device_distance!(ctx, [line_width_in,  0])[1]
    dash_length  = user_to_device_distance!(ctx, [dash_length_in, 0])[1]
    dash_spacing = user_to_device_distance!(ctx, [dash_spacing_in,0])[1]
    dash_offset  = user_to_device_distance!(ctx, [dash_offset_in, 0])[1]
    dashes = [dash_length, dash_spacing]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)
    set_dash(ctx, dashes, dash_offset)

    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    stroke(ctx)
    restore(ctx)
end
function render_dashed_arc(
    ctx          :: CairoContext,
    x            :: Real,
    y            :: Real,
    radius       :: Real,
    start_ang    :: Real,
    end_ang      :: Real,
    color_fill   :: Colorant,
    color_stroke :: Colorant   = color_fill,
    line_width_in   :: Real = 1.0,
    dash_length_in  :: Real = 1.0,
    dash_spacing_in :: Real = 1.0,
    dash_offset_in  :: Real = 0.0
    )

    line_width   = user_to_device_distance!(ctx, [line_width_in,  0])[1]
    dash_length  = user_to_device_distance!(ctx, [dash_length_in, 0])[1]
    dash_spacing = user_to_device_distance!(ctx, [dash_spacing_in,0])[1]
    dash_offset  = user_to_device_distance!(ctx, [dash_offset_in, 0])[1]
    dashes = [dash_length, dash_spacing]

    save(ctx)
    translate(ctx, x, y)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_source_rgba(ctx, color_fill)
    fill(ctx)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_source_rgba(ctx, color_stroke)
    set_line_width(ctx, line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)
    set_dash(ctx, dashes, dash_offset)
    stroke(ctx)

    restore(ctx)
end
function render_arrow{T<:Real}(
    ctx               :: CairoContext,
    pts               :: Matrix{T}, # 2×n
    color             :: Colorant,
    line_width        :: Real,
    arrowhead_len     :: Real;
    ARROW_WIDTH_RATIO :: AbstractFloat = 0.8,
    ARROW_ALPHA       :: AbstractFloat = 0.1pi,
    ARROW_BETA        :: AbstractFloat = 0.8pi
    )

    @assert(size(pts,2) > 1)

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx, color)
    set_line_width(ctx, line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)

    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    stroke(ctx)

    # orientation of the arrowhead
    theta = atan2(pts[2,end]-pts[2,end-1], pts[1,end]-pts[1,end-1])

    arrowhead_width = arrowhead_len * ARROW_WIDTH_RATIO
    whatev =    pi - ARROW_BETA
    delta = 0.5pi - whatev
    epsilon =  pi - ARROW_BETA - ARROW_ALPHA
    len_prime = 0.5arrowhead_width*sin(delta + epsilon)/sin(ARROW_ALPHA) - arrowhead_len

    R = [ cos(theta) -sin(theta);
          sin(theta)  cos(theta) ]

    # render the arrowhead
    O = pts[:,end] # origin, center of the arrowhead
    A = O + R * [ arrowhead_len/2 0]'
    B = O + R * [-arrowhead_len/2 0]'
    C = O + R * [-arrowhead_len/2-len_prime  arrowhead_width/2]'
    D = O + R * [-arrowhead_len/2-len_prime -arrowhead_width/2]'

    new_sub_path(ctx)
    move_to(ctx, A[1], A[2])
    line_to(ctx, C[1], C[2])
    line_to(ctx, B[1], B[2])
    line_to(ctx, D[1], D[2])
    close_path(ctx)
    fill(ctx)

    restore(ctx)
end

function render_colormesh{T<:Real, S<:Real, U<:Real}(
    ctx::CairoContext,
    C::Matrix{T}, # n×m matrix of 0->1 values
    X::Vector{S}, # n+1 vector of x bin boundaries
    Y::Vector{U} # m+1 vector of y bin boundaries
    )

    n,m = size(C)
    @assert(length(X) == n+1)
    @assert(length(Y) == n+1)

    save(ctx)

    x₂ = X[1]
    for i = 1 : n
        x₁, x₂ = x₂, X[i+1]
        y₂ = Y[1]
        for j = 1 : m
            y₁, y₂ = y₂, Y[j+1]

            c = C[i,j]

            new_sub_path(ctx)
            move_to(ctx, x₁, y₁)
            line_to(ctx, x₂, y₁)
            line_to(ctx, x₂, y₂)
            line_to(ctx, x₁, y₂)
            close_path(ctx)

            set_source_rgba(ctx, c, c, c, 1.0)
            fill(ctx)
        end
    end

    restore(ctx)
end
function render_colormesh{T<:Real, S<:Real, U<:Real}(
    ctx::CairoContext,
    C::Matrix{T}, # n×m matrix of 0->1 values
    X::Vector{S}, # n+1 vector of x bin boundaries
    Y::Vector{U}, # m+1 vector of y bin boundaries
    color₀::Colorant, # color for c = 0
    color₁::Colorant, # color for c = 1
    )

    n,m = size(C)
    @assert(length(X) == n+1)
    @assert(length(Y) == m+1)

    save(ctx)

    x₂ = X[1]
    for i = 1 : n
        x₁, x₂ = x₂, X[i+1]
        y₂ = Y[1]
        for j = 1 : m
            y₁, y₂ = y₂, Y[j+1]

            set_source_rgba(ctx, color₀, color₁, C[i,j])

            new_sub_path(ctx)
            move_to(ctx, x₁, y₁)
            line_to(ctx, x₂, y₁)
            line_to(ctx, x₂, y₂)
            line_to(ctx, x₁, y₂)
            close_path(ctx)

            fill(ctx)
        end
    end

    restore(ctx)
end

# ----------------------

function add_instruction!(rm::RenderModel, f::Function, arr::Tuple; incameraframe::Bool=true )

    #=
    Add an instruction to the rendermodel

    INPUT:
        rendermodel   - the RenderModel we are adding the instruction to
        f             - the function to be called, the first argument must be a CairoContext
        arr           - tuple of input arguments to f, skipping the CairoContext
        incameraframe - we render in the camera frame by default.
                        To render in the canvas frame (common with text) set this to false

    ex: add_instruction!(rendermodel, render_text, ("hello world", 10, 20, 15, [1.0,1.0,1.0]))
    =#

    push!(rm.instruction_set, (f, arr, incameraframe))
    rm
end

camera_move!(rm::RenderModel, dx::Real, dy::Real) = rm.camera_center = rm.camera_center + VecE2(dx, dy)
camera_move!(rm::RenderModel, Δ::VecE2) = rm.camera_center = rm.camera_center + Δ
camera_move_pix!(rm::RenderModel, dx::Real, dy::Real) = rm.camera_center = rm.camera_center + VecE2(dx/rm.camera_zoom, dy/rm.camera_zoom)
camera_move_pix!(rm::RenderModel, Δ::VecE2) = rm.camera_center = rm.camera_center + VecE2(Δ.x/rm.camera_zoom, Δ.y/rm.camera_zoom)

camera_rotate!(rm::RenderModel, θ::Real) = rm.camera_rotation += θ # [radians]
camera_setrotation!(rm::RenderModel, rad::Real) = rm.camera_rotation = rad
camera_zoom!(rm::RenderModel, factor::Real) = rm.camera_zoom *= factor
camera_setzoom!(rm::RenderModel, zoom::Real) = rm.camera_zoom = zoom
camera_set_x!(rm::RenderModel, x::Real) = rm.camera_center = VecE2(x, rm.camera_center.y)
camera_set_y!(rm::RenderModel, y::Real) = rm.camera_center = VecE2(rm.camera_center.x, y)
camera_set_pos!(rm::RenderModel, x::Real, y::Real) = rm.camera_center = VecE2(x, y)
camera_set_pos!(rm::RenderModel, pos::AbstractVec) = rm.camera_center = VecE2(pos.x, pos.y)
set_background_color!(rm::RenderModel, color::Colorant) = rm.background_color = convert(RGB{U8}, color)

function camera_reset!(rm::RenderModel)
    rm.camera_center = VecE2(0.0,0.0)
    rm.camera_zoom = 1.0
    rm.camera_rotation = 0.0
    rm
end
function camera_set!(rm::RenderModel, x::Real, y::Real, zoom::Real)
    rm.camera_center = VecE2(x,y)
    rm.camera_zoom = zoom
    rm
end
function camera_set!(rm::RenderModel, pt::AbstractVec, zoom::Real)
    rm.camera_center = convert(VecE2, pt)
    rm.camera_zoom = zoom
    rm
end
function clear_setup!(rm::RenderModel)
    empty!(rm.instruction_set)
    camera_reset!(rm)
    rm
end
function camera_fit_to_content!(
    rendermodel    :: RenderModel,
    canvas_width   :: Integer,
    canvas_height  :: Integer;
    percent_border :: Real = 0.1 # amount of extra border we add
    )

    #=
    Positions the camera so that all content is visible within its field of view
    This will always set the camera rotation to zero
    An extra border can be added as well
    =#

    rendermodel.camera_rotation = 0.0

    if isempty(rendermodel.instruction_set)
        return
    end

    # determine render bounds
    xmax = -Inf; xmin = Inf
    ymax = -Inf; ymin = Inf

    for tup in rendermodel.instruction_set
        f = tup[1]
        in_camera_frame = tup[3]
        if !in_camera_frame
            continue
        end

        (x,y,flag) = (0,0,false)
        if f == render_circle || f == render_round_rect
            (x,y,flag) = (tup[2][1],tup[2][2],true)
        elseif f == render_text
            (x,y,flag) = (tup[2][2],tup[2][3],true)
        elseif f == render_point_trail || f == render_line ||
               f == render_dashed_line
            for xi in tup[2][1][1,:]
                xmax = maximum([xmax, xi])
                xmin = minimum([xmin, xi])
            end
            for yi in tup[2][1][2,:]
                ymax = maximum([ymax, yi])
                ymin = minimum([ymin, yi])
            end
        end

        if flag
            xmax = max(xmax, x)
            xmin = min(xmin, x)
            ymax = max(ymax, y)
            ymin = min(ymin, y)
        end
    end

    if isinf(xmin) || isinf(ymin)
        return
    end

    if xmax < xmin
        xmax = xmin + 1.0
    end
    if ymax < ymin
        ymax = ymin + 1.0
    end

    # compute zoom to fit
    world_width = xmax - xmin
    world_height = ymax - ymin
    canvas_aspect = canvas_width / canvas_height
    world_aspect = world_width / world_height

    if world_aspect > canvas_aspect
        # expand height to fit
        half_diff =  (world_width * canvas_aspect - world_height) / 2
        world_height = world_width * canvas_aspect # [m]
        ymax += half_diff
        ymin -= half_diff
    else
        # expand width to fit
        half_diff = (canvas_aspect * world_height - world_width) / 2
        world_width = canvas_aspect * world_height
        xmax += half_diff
        xmin -= half_diff
    end

    rendermodel.camera_center = VecE2(xmin + world_width/2, ymin + world_height/2) # [m]
    rendermodel.camera_zoom   = (canvas_width*(1-percent_border)) / world_width # [pix / m]

    rendermodel
end

function render(rendermodel::RenderModel, ctx::CairoContext, canvas_width::Integer, canvas_height::Integer)

    # fill with background color
    set_source_rgba( ctx, rendermodel.background_color)
    paint(ctx)

    # render text if no other instructions
    if isempty(rendermodel.instruction_set)
        text_color = RGB(1.0 - convert(Float64, red(rendermodel.background_color)),
                         1.0 - convert(Float64, green(rendermodel.background_color)),
                         1.0 - convert(Float64, blue(rendermodel.background_color)))
        render_text(ctx, "This screen left intentionally blank", canvas_width/2, canvas_height/2, 40, text_color, true)
        return
    end

    # reset the transform
    reset_transform(ctx)
    translate(ctx, canvas_width/2, canvas_height/2)                              # translate to image center
    scale(ctx, rendermodel.camera_zoom, -rendermodel.camera_zoom )               # [pix -> m]
    rotate(ctx, rendermodel.camera_rotation)
    translate(ctx, -rendermodel.camera_center.x, -rendermodel.camera_center.y) # translate to camera location

    # execute all instructions
    for tup in rendermodel.instruction_set
        func = tup[1]
        content = tup[2]
        incameraframe = tup[3]

        if !incameraframe
            save(ctx)
            reset_transform(ctx)
            func(ctx, content...)
            restore(ctx)
        else

            if func == render_text
                # deal with the inverted y-axis issue for text rendered
                mat = get_matrix(ctx)
                mat2 = [mat.xx mat.xy mat.x0;
                        mat.yx mat.yy mat.y0]
                pos = mat2*[content[2] content[3] 1.0]'
                content = tuple(content[1], pos..., content[4:end]...)

                save(ctx)
                reset_transform(ctx)
                render_text(ctx, content... )
                restore(ctx)
            else
                # just use the function normally
                func(ctx, content...)
            end
        end
    end

    ctx
end

function Cairo.set_source_rgba(ctx::CairoContext, color::Color)

    r = convert(Float64, red(color))
    g = convert(Float64, green(color))
    b = convert(Float64, blue(color))

    set_source_rgba(ctx, r, g, b, 1.0)
end
function Cairo.set_source_rgba(ctx::CairoContext, color::TransparentColor)

    r = convert(Float64, red(color))
    g = convert(Float64, green(color))
    b = convert(Float64, blue(color))
    a = convert(Float64, alpha(color))

    set_source_rgba(ctx, r, g, b, a)
end
function Cairo.set_source_rgba(ctx::CairoContext, color₀::Colorant, color₁::Colorant, t::Real)

    r₀ = convert(Float64, red(color₀))
    g₀ = convert(Float64, green(color₀))
    b₀ = convert(Float64, blue(color₀))
    a₀ = convert(Float64, alpha(color₀))

    r₁ = convert(Float64, red(color₁))
    g₁ = convert(Float64, green(color₁))
    b₁ = convert(Float64, blue(color₁))
    a₁ = convert(Float64, alpha(color₁))

    r = r₀ + (r₁ - r₀)*t
    g = g₀ + (g₁ - g₀)*t
    b = b₀ + (b₁ - b₀)*t
    a = a₀ + (a₁ - a₀)*t
    set_source_rgba(ctx, r, g, b, a)
end

function get_surface_and_context(canvas_width::Int, canvas_height::Int)
    s = CairoRGBSurface(canvas_width, canvas_height)
    ctx = creategc(s)
    (s, ctx)
end