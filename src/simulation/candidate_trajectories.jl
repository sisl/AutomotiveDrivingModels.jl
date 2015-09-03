export
    PolynomialFactoredTrajectory,
    ExtractedTrajdef,
    TrajDefLink,
    TrajDef,

    get_quartic_coefficients,
    get_quintic_coefficients,

    get_num_pdset_frames,
    create_scenario_pdset,

    extract_trajdef,
    extract_trajdefs,
    copy_trajdef_start

immutable PolynomialFactoredTrajectory
    # NOTE(tim): t=0 is the start of the trajectory in the polynomials
    #            t=τ is the end of the trajectory

    s::Polynomial # along the centerline
    d::Polynomial # perpendicular to the centerline, (left)
end

function translate(traj::PolynomialFactoredTrajectory, Δs::Float64, Δd::Float64)

    PolynomialFactoredTrajectory(translate(traj.s, Δs), translate(traj.d, Δd))
end
function translate_so_is_at_loc_at_time(traj::PolynomialFactoredTrajectory, s::Float64, d::Float64, t::Float64)

    s_t = p₁(traj.s, t)
    d_t = p₁(traj.d, t)
    Δs = s - s_t
    Δd = d - d_t
    Δs = NaN # NOTE(tim): XXXX
    Δd = NaN
    translate(traj, Δs, Δd)
end

function _get_M1(t::Float64, t²::Float64=t*t)
    [1.0   t  t²;
     0.0 1.0 2t;
     0.0 0.0 2.0]
end
function _get_M2_quartic(t::Float64, t²::Float64=t*t, t³::Float64=t²*t, t⁴::Float64=t³*t, t⁵::Float64=t⁴*t)
    [ t³  t⁴   t⁵;
     3t² 4t³  5t⁴;
     6t 12t² 20t³]
end
function _get_M2_quintic(t::Float64, t²::Float64=t*t, t³::Float64=t²*t, t⁴::Float64=t³*t)
    [-1.0  t³  t⁴
      0.0 3t² 4t³;
      0.0 6t 12t²]
end
function _get_M1_and_M2_quartic(t::Float64)
    t² = t*t
    t³ = t²*t
    t⁴ = t³*t

    M₁ = _get_M1(t, t²)
    M₂ = _get_M2_quartic(t, t², t³, t⁴)
    (M₁, M₂)
end
function _get_M1_and_M2_quintic(t::Float64)
    t² = t*t
    t³ = t²*t
    t⁴ = t³*t

    M₁ = _get_M1(t, t²)
    M₂ = _get_M2_quintic(t, t², t³, t⁴)
    (M₁, M₂)
end

function get_quintic_coefficients(
    x1::Float64, # starting state
    v1::Float64, # starting 1st derivative
    a1::Float64, # starting 2nd derivative
    x2::Float64, # end state
    v2::Float64, # end 1st derivative
    a2::Float64, # end 2nd derivative
    τ::Float64, # end time
    )

    @assert(τ > 0.0)

    ξ₀ =  [x1, v1, a1]
    M₁₀ = _get_M1(0.0)

    ξ₂ =  [x2, v2, a2]
    M₁₂, M₂₂ = _get_M1_and_M2_quartic(τ)

    c₀₁₂ = M₁₀ \ ξ₀
    c₃₄₅ = M₂₂ \ (ξ₂ - M₁₂*c₀₁₂)

    Quintic(c₀₁₂[1], c₀₁₂[2], c₀₁₂[3], c₃₄₅[1], c₃₄₅[2], c₃₄₅[3])
end
function get_quartic_coefficients(
    x1::Float64, # starting state
    v1::Float64, # starting 1st derivative
    a1::Float64, # starting 2nd derivative
    v2::Float64, # end 1st derivative
    a2::Float64, # end 2nd derivative
    τ::Float64, # end time
    )

    @assert(τ > 0.0)

    ξ₀ =  [x1, v1, a1]
    M₁₀ = _get_M1(0.0)

    ξ₂ =  [0.0, v2, a2]
    M₁₂, M₂₂ = _get_M1_and_M2_quintic(τ)

    c₀₁₂ = M₁₀ \ ξ₀
    retval = M₂₂ \ (ξ₂ - M₁₂*c₀₁₂) # returns vector [ξ₁, c₃, c₄]

    Quartic(c₀₁₂[1], c₀₁₂[2], c₀₁₂[3], retval[2], retval[3])
end

####################################################

# abstract TradDefLink
# immutable TrajDefLinkTargetPosition <: TradDefLink

#     n_pdset_frames::Int # duration of the link [pdset frames]
#     lanetag::LaneTag # the lane that this is relative to at the start of the segment

#     d::Float64
#     ddot::Float64 # should be always 0.0
#     dddot::Float64 # should be always 0.0

#     s::Float64
#     sdot::Float64
#     sddot::Float64

#     function TrajDefLinkTargetPosition(
#         n_pdset_frames::Int,
#         lanetag::LaneTag,
#         d::Float64,
#         ddot::Float64,
#         dddot::Float64,
#         s::Float64,
#         sdot::Float64,
#         sddot::Float64,
#         )

#         new(n_pdset_frames, lanetag, d, ddot, dddot, s, sdot, sddot)
#     end
# end
immutable TrajDefLink #TargetSpeed  <: TradDefLink

    n_pdset_frames::Int # duration of link [pdset frames]
    lanetag::LaneTag # the lane that this is relative to
                     # at the start of the segment
                     # NOTE(tim): we may cross tile boundaries

    d::Float64 # lane offset
    v::Float64 # speed
    a::Float64 # accel
    ϕ::Float64 # speed heading, relative to lane
    ψ::Float64 # accel heading, relative to lane   

    function TrajDefLink(
        n_pdset_frames::Integer,
        lanetag::LaneTag,

        d::Float64,
        v::Float64,
        a::Float64 = 0.0,
        ϕ::Float64 = 0.0,
        ψ::Float64 = 0.0
        )

        new(n_pdset_frames, lanetag, d, v, a, ϕ, ψ)
    end
end

type TrajDef
    lanetag::LaneTag # initial lanetag
    extind::Float64 # initial state along lanetag

    d::Float64 # initial lane offset
    v::Float64 # initial speed
    a::Float64 # initial accel
    ϕ::Float64 # initial speed heading, relative to lane
    ψ::Float64 # initial accel heading, relative to lane

    links::Vector{TrajDefLink} # list link definitions, in order

    function TrajDef(
        lanetag::LaneTag,
        extind::Float64,

        d::Float64,
        v::Float64,
        a::Float64 = 0.0,
        ϕ::Float64 = 0.0,
        ψ::Float64 = 0.0,

        links::Vector{TrajDefLink} = TrajDefLink[]
        )

        @assert(extind ≥ 1.0)
        new(lanetag, extind, d, v, a, ϕ, ψ, links)
    end
    function TrajDef(
        sn::StreetNetwork,
        inertial::VecE2,

        v::Float64,
        ϕ::Float64 = 0.0,
        a::Float64 = 0.0,
        ψ::Float64 = 0.0,

        links::Vector{TrajDefLink} = TrajDefLink[]
        )

        proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
        @assert(proj.successful)

        extind = proj.extind
        lanetag = LaneTag(proj.tile, proj.laneid)
        dcl = pt_to_frenet_xyy(proj.curvept, inertial.x, inertial.y, 0.0)[2]

        new(lanetag, extind, dcl, v, a, ϕ, ψ, links)
    end
    function TrajDef(
        pdset::PrimaryDataset,
        sn::StreetNetwork,
        carid::Integer,
        validfind::Integer,
        links::Vector{TrajDefLink} = TrajDefLink[]
        )

        start_carind = carid2ind(pdset, carid, validfind)
        start_velFx = get(pdset, :velFx, start_carind, validfind)
        start_velFy = get(pdset, :velFy, start_carind, validfind)
        start_speed = sqrt(start_velFx*start_velFx + start_velFy*start_velFy)

        inertial = get_inertial(pdset, start_carind, validfind)
        proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
        @assert(proj.successful)

        extind = proj.extind
        lanetag = LaneTag(proj.tile, proj.laneid)
        s, dcl, ϕ = pt_to_frenet_xyy(proj.curvept, inertial.x, inertial.y, inertial.θ)

        if validfind > 1
            past_validfind = validfind-1
            past_carind = carid2ind(pdset, carid, past_validfind)

            past_velFx = get(pdset, :velFx, past_carind, past_validfind)
            past_velFy = get(pdset, :velFy, past_carind, past_validfind)
            past_speed = sqrt(past_velFx*past_velFx + past_velFy*past_velFy)

            start_time = gete(pdset, :time, validfind)
            past_time = gete(pdset, :time, past_validfind)

            ψ = atan2(start_velFy-past_velFy, start_velFx-past_velFx) - ϕ
            start_accel = (start_speed - past_speed) / (start_time - past_time)
        else
            ψ = 0.0
            start_accel = 0.0
        end

        # lane = get_lane(sn, lanetag)
        # footpoint = curve_at(lane.curve, extind)
        # footvec = VecSE2(footpoint.x, footpoint.y, footpoint.θ)
        # inertial2 = footvec + polar(dcl, π/2 + footvec.θ, footvec.θ + ϕ)
        # println(inertial.x, "  ", inertial2.x)
        # println(inertial.y, "  ", inertial2.y)
        # println(inertial.θ, "  ", inertial2.θ)

        new(lanetag, extind, dcl, start_speed, start_accel, ϕ, ψ, links)
    end
end

function get_num_pdset_frames(trajdef::TrajDef)
    nframes = 1
    for link in trajdef.links
        nframes += link.n_pdset_frames
    end
    nframes
end
function Base.push!(trajdef::TrajDef, link::TrajDefLink)
    push!(trajdef.links, link)
    trajdef
end

function _set_vehicle_other_nocheck!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    carid::Integer,
    validfind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    footpoint::CurvePt, # NOTE(tim): only need this for curvature
    extind::Float64,
    lanetag::LaneTag,
    dcl::Float64,
    ϕ::Float64,
    )

    if !idinframe(pdset, carid, validfind)
        add_car_to_validfind!(pdset, carid, validfind)
    end
    carind = carid2ind(pdset, carid, validfind)

    lane_index  = lanetag.lane

    setc!(pdset, :posGx,     carind, validfind, inertial.x)
    setc!(pdset, :posGy,     carind, validfind, inertial.y)
    setc!(pdset, :posGyaw,   carind, validfind, inertial.θ)

    setc!(pdset, :posFyaw,   carind, validfind, ϕ)

    setc!(pdset, :velFx,     carind, validfind, speed*cos(ϕ))
    setc!(pdset, :velFy,     carind, validfind, speed*sin(ϕ))

    setc!(pdset, :lanetag,   carind, validfind, lanetag)
    setc!(pdset, :curvature, carind, validfind, footpoint.k)
    setc!(pdset, :d_cl,      carind, validfind, dcl)

    seg = get_segment(sn, lanetag)
    d_merge = distance_to_lane_merge(seg, lane_index, extind)
    d_split = distance_to_lane_split(seg, lane_index, extind)
    setc!(pdset, :d_merge,   carind, validfind, isinf(d_merge) ? NA : d_merge)
    setc!(pdset, :d_split,   carind, validfind, isinf(d_split) ? NA : d_split)

    nll, nlr = StreetNetworks.num_lanes_on_sides(seg, lane_index, extind)
    @assert(nll ≥ 0)
    @assert(nlr ≥ 0)
    setc!(pdset, :nll,       carind, validfind, nll)
    setc!(pdset, :nlr,       carind, validfind, nlr)

    lane_width_left, lane_width_right = marker_distances(seg, lane_index, extind)
    setc!(pdset, :d_mr,      carind, validfind, (dcl <  lane_width_left)  ?  lane_width_left - dcl  : Inf)
    setc!(pdset, :d_ml,      carind, validfind, (dcl > -lane_width_right) ?  dcl - lane_width_right : Inf)

    setc!(pdset, :id,        carind, validfind, uint32(carid))
    setc!(pdset, :t_inview,  carind, validfind, NA)

    pdset
end
function _set_vehicle_ego_nocheck!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    frameind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    footpoint::CurvePt, # NOTE(tim): only need this for curvature
    extind::Float64,
    lanetag::LaneTag,
    dcl::Float64,
    ϕ::Float64,
    )

    lane_index  = lanetag.lane

    sete!(pdset, :posGx,     frameind, inertial.x)
    sete!(pdset, :posGy,     frameind, inertial.y)
    sete!(pdset, :posGyaw,   frameind, inertial.θ)

    sete!(pdset, :posFyaw,   frameind, ϕ)

    sete!(pdset, :velFx,     frameind, speed*cos(ϕ))
    sete!(pdset, :velFy,     frameind, speed*sin(ϕ))

    sete!(pdset, :lanetag,   frameind, lanetag)
    sete!(pdset, :curvature, frameind, footpoint.k)
    sete!(pdset, :d_cl,      frameind, dcl)

    seg = get_segment(sn, lanetag)
    d_merge = distance_to_lane_merge(seg, lane_index, extind)
    d_split = distance_to_lane_split(seg, lane_index, extind)
    sete!(pdset, :d_merge,   frameind, isinf(d_merge) ? NA : d_merge)
    sete!(pdset, :d_split,   frameind, isinf(d_split) ? NA : d_split)

    nll, nlr = StreetNetworks.num_lanes_on_sides(seg, lane_index, extind)
    @assert(nll ≥ 0)
    @assert(nlr ≥ 0)
    sete!(pdset, :nll,       frameind, nll)
    sete!(pdset, :nlr,       frameind, nlr)

    lane_width_left, lane_width_right = marker_distances(seg, lane_index, extind)
    sete!(pdset, :d_mr,      frameind, (dcl <  lane_width_left)  ?  lane_width_left - dcl  : Inf)
    sete!(pdset, :d_ml,      frameind, (dcl > -lane_width_right) ?  dcl - lane_width_right : Inf)

    pdset
end

function _set_vehicle_other!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    carid::Integer,
    validfind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    )

    proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
    @assert(proj.successful)

    laneid = proj.laneid
    footpoint = proj.curvept
    extind = proj.extind
    lanetag = LaneTag(sn, proj.laneid)
    s, dcl, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)

    _set_vehicle_other_nocheck!(pdset, sn, carid, validfind, inertial, speed,
                                footpoint, extind, lanetag, dcl, ϕ)
end
function _set_vehicle_other!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    carid::Integer,
    validfind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    footpoint::CurvePt,
    extind::Float64,
    lanetag::LaneTag,
    dcl::Float64,
    ϕ::Float64,
    )

    proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
    @assert(proj.successful)

    if proj.laneid == convert(LaneID, lanetag)
        _set_vehicle_other_nocheck!(pdset, sn, carid, validfind, inertial, speed,
                                    footpoint, extind, lanetag, dcl, ϕ)
    else
        footpoint = proj.curvept
        extind = proj.extind
        lanetag = LaneTag(proj.tile, proj.laneid)
        s, dcl, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)    

        _set_vehicle_other_nocheck!(pdset, sn, carid, validfind, inertial, speed,
                                    footpoint, extind, lanetag, dcl, ϕ)
    end

    pdset
end
function _set_vehicle_ego!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    frameind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    )

    proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
    @assert(proj.successful)

    laneid = proj.laneid
    footpoint = proj.curvept
    extind = proj.extind
    lanetag = LaneTag(sn, proj.laneid)
    s, dcl, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)

    _set_vehicle_ego_nocheck!(pdset, sn, frameind, inertial, speed,
                              footpoint, extind, lanetag, dcl, ϕ)
end
function _set_vehicle_ego!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    frameind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    footpoint::CurvePt,
    extind::Float64,
    lanetag::LaneTag,
    dcl::Float64,
    ϕ::Float64,
    )

    proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
    @assert(proj.successful)

    if proj.laneid == convert(LaneID, lanetag)
        _set_vehicle_ego_nocheck!(pdset, sn, frameind, inertial, speed,
                                  footpoint, extind, lanetag, dcl, ϕ)
    else
        footpoint = proj.curvept
        extind = proj.extind
        lanetag = LaneTag(proj.tile, proj.laneid)
        s, dcl, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)    

        _set_vehicle_ego_nocheck!(pdset, sn, frameind, inertial, speed,
                                  footpoint, extind, lanetag, dcl, ϕ)
    end

    pdset
end
function _set_vehicle!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    carid::Integer,
    frameind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    footpoint::CurvePt,
    extind::Float64,
    lanetag::LaneTag,
    dcl::Float64,
    ϕ::Float64,
    )

    if carid != CARID_EGO
        validfind = frameind2validfind(pdset, frameind)
        _set_vehicle_other!(pdset, sn, carid, validfind, inertial, speed, 
                            footpoint, extind, lanetag, dcl, ϕ)
    else
        _set_vehicle_ego!(pdset, sn, frameind, inertial, speed,
                          footpoint, extind, lanetag, dcl, ϕ)
    end
end

function _add_trajlink_to_pdset!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    carid::Integer,
    sdot₁::Float64,
    sddot₁::Float64,
    d₁::Float64,
    ddot₁::Float64,
    dddot₁::Float64,
    link::TrajDefLink,
    lane::StreetLane,
    extind_start::Float64, # extind we were previously at, for guessing
    sec_per_frame::Float64,
    inertial::VecSE2,
    footpoint::CurvePt,
    frameind::Int,
    )

    τ = link.n_pdset_frames * sec_per_frame

    d₂ = link.d # final lateral offset
    ddot₂ = link.v * sin(link.ϕ) # final lateral velocity
    dddot₂ = link.a * sin(link.ψ) # final lateral accel

    sdot₂ = link.v * cos(link.ϕ) # final longitudinal velocity
    sddot₂ = link.a * cos(link.ψ) # final longitudinal accel

    # Compute a trajectory in which the lateral start and end states are given
    # and where the longitudinal trajectory has unspecified end position
    poly_s = get_quartic_coefficients(0.0, sdot₁, sddot₁, sdot₂, sddot₂, τ)
    poly_d = get_quintic_coefficients(d₁, ddot₁, dddot₁, d₂, ddot₂, dddot₂, τ)

    t = 0.0
    prev_extind = extind_start
    lanetag = LaneTag(sn, lane)
    new_footpoint = footpoint
    footpoint_s_ajdust = 0.0
    for _ in 1 : link.n_pdset_frames
        t += sec_per_frame

        Δs, sdot = p₁₂(poly_s, float(t))
        new_d, ddot = p₁₂(poly_d, float(t))

        new_s = footpoint.s + footpoint_s_ajdust + Δs
        new_ϕ = atan2(ddot, sdot)
        new_extind = closest_point_extind_to_curve_guess(lane.curve, new_s, prev_extind)
        while is_extind_at_curve_end(lane.curve, new_extind)

            new_s -= lane.nodes[end].d_along
            footpoint_s_ajdust -= lane.nodes[end].d_along

            lane = next_lane(sn, lane)
            lanetag = LaneTag(sn, lane)

            new_s -= lane.nodes[2].d_along
            footpoint_s_ajdust -= lane.nodes[2].d_along
            
            new_extind = closest_point_extind_to_curve_guess(lane.curve, new_s, 1.5)
        end

        new_footpoint = curve_at(lane.curve, new_extind) 
        new_footvec = VecSE2(new_footpoint.x, new_footpoint.y, new_footpoint.θ)       
        inertial = new_footvec + polar(new_d, π/2 + new_footvec.θ, new_footvec.θ + new_ϕ)

        frameind += 1
        speed = hypot(sdot, ddot)
        _set_vehicle!(pdset, sn, carid, frameind, inertial, speed,
                      new_footpoint, new_extind, lanetag, new_d, new_ϕ)

        prev_extind = new_extind
    end

    (inertial, new_footpoint, prev_extind, d₂, ddot₂, dddot₂, sdot₂, sddot₂, frameind, lanetag)
end
function Base.insert!(
    pdset::PrimaryDataset,
    sn::StreetNetwork,
    trajdef::TrajDef,
    carid::Integer,
    frameind_start::Integer,
    sec_per_frame::Float64,
    )

    lane = get_lane(sn, trajdef.lanetag)
    footpoint = curve_at(lane.curve, trajdef.extind)
    footvec = VecSE2(footpoint.x, footpoint.y, footpoint.θ)
    inertial = footvec + polar(trajdef.d, π/2 + footvec.θ, footvec.θ + trajdef.ϕ)

    frameind = frameind_start
    _set_vehicle!(pdset, sn, carid, frameind, inertial, trajdef.v, 
                  footpoint, trajdef.extind, trajdef.lanetag, trajdef.d, trajdef.ϕ)

    #######################################
    # check to ensire that inertial is the same as where we are calling set_vehicle


    #######################################
    # Now we select the next link, project our current location to the new lane tag
    # and move forward!

    a = trajdef.a
    v = trajdef.v
    ψ = trajdef.ψ + footvec.θ # in global coordinates

    link_index = 1
    link = trajdef.links[link_index]
    lanetag = link.lanetag

    lane = get_lane(sn, lanetag)
    curve = lane.curve

    # NOTE: using the same inertial VecSE2
    extind = closest_point_extind_to_curve(curve, inertial.x, inertial.y)
    @assert(!is_extind_at_curve_end(curve, extind))
    footpoint = curve_at(curve, extind)

    s, d₁, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)
    ddot₁ = v * sin(ϕ) # initial lateral velocity
    dddot₁ = a * sin(ψ - footpoint.θ) # initial lateral accel

    sdot₁ = v * cos(ϕ) # initial longitudinal velocity
    sddot₁ = a * cos(ψ - footpoint.θ) # initial longitudinal accel

    (inertial, footpoint, extind, d₂, ddot₂, dddot₂, sdot₂, sddot₂, frameind, lanetag) = _add_trajlink_to_pdset!(
                                                                                            pdset, sn, carid, sdot₁, sddot₁, d₁, ddot₁, dddot₁, link,
                                                                                            lane, extind, sec_per_frame, inertial, footpoint, frameind)

    while link_index < length(trajdef.links)
        link_index += 1

        if trajdef.links[link_index].lanetag == lanetag
            link = trajdef.links[link_index]
            d₁     = d₂
            ddot₁  = ddot₂
            dddot₁ = dddot₂
            sdot₁  = sdot₂
            sddot₁ = sddot₂
        else
            v = hypot(ddot₂, sdot₂)
            a = hypot(dddot₂, sddot₂)

            ϕ = trajdef.links[1].ϕ
            ψ = trajdef.links[1].ψ

            link = trajdef.links[link_index]
            lanetag = link.lanetag

            lane = get_lane(sn, lanetag)
            curve = lane.curve

            n_lane_jumps = 0
            extind = closest_point_extind_to_curve(curve, inertial.x, inertial.y)
            while is_extind_at_curve_end(curve, extind) && n_lane_jumps < 5
                n_lane_jumps += 1
                if extind > 1.0
                    @assert(has_next_lane(sn, lane))
                    lane = next_lane(sn, lane)
                    curve = lane.curve
                    extind = closest_point_extind_to_curve(curve, inertial.x, inertial.y)
                end
            end

            @assert(!is_extind_at_curve_end(curve, extind))
            footpoint = curve_at(curve, extind)

            s, d₁, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)
            ddot₁ = v * sin(ϕ) # initial lateral velocity
            dddot₁ = a * sin(ψ - footpoint.θ) # initial lateral accel

            sdot₁ = v * cos(ϕ) # initial longitudinal velocity
            sddot₁ = a * cos(ψ - footpoint.θ) # initial longitudinal accel
        end

        (inertial, footpoint, extind, d₂, ddot₂, dddot₂, sdot₂, sddot₂, frameind) = _add_trajlink_to_pdset!(
                                                                                        pdset, sn, carid, sdot₁, sddot₁, d₁, ddot₁, dddot₁, link,
                                                                                        lane, extind, sec_per_frame, inertial, footpoint, frameind)
    end

    pdset
end

function create_scenario_pdset(
    sn::StreetNetwork,
    history::Int, # this includes the current frame
    horizon::Int,
    trajdefs::Vector{TrajDef}, # first is assumed to be ego
    sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
    )

    @assert(history > 0)
    @assert(horizon > 0)
    @assert(!isempty(trajdefs))

    nframes = history + horizon
    n_other_vehicles = length(trajdefs) - 1
    pdset = create_empty_pdset(nframes, n_other_vehicles, sec_per_frame=sec_per_frame)

    for (i,trajdef) in enumerate(trajdefs)
        carid = i-1 + CARID_EGO
        insert!(pdset, sn, trajdef, carid, 1, sec_per_frame)
    end
    
    pdset
end

###################

type ExtractedTrajdef
    df::DataFrame # same format as df_ego_primary
    carid::Integer
end

get_num_pdset_frames(extracted::ExtractedTrajdef) = nrow(extracted.df)

function _set_vehicle_nocheck!(
    extracted::ExtractedTrajdef,
    sn::StreetNetwork,
    frameind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    footpoint::CurvePt, # NOTE(tim): only need this for curvature
    extind::Float64,
    lanetag::LaneTag,
    dcl::Float64,
    ϕ::Float64,
    )

    df = extracted.df
    lane_index  = lanetag.lane

    df[frameind,:posGx] = inertial.x
    df[frameind,:posGy] = inertial.y
    df[frameind,:posGyaw] = inertial.θ

    df[frameind,:posFyaw] = ϕ

    df[frameind,:velFx] = speed*cos(ϕ)
    df[frameind,:velFy] = speed*sin(ϕ)

    df[frameind,:lanetag] = lanetag
    df[frameind,:curvature] = footpoint.k
    df[frameind,:d_cl] = dcl

    seg = get_segment(sn, lanetag)
    d_merge = distance_to_lane_merge(seg, lane_index, extind)
    d_split = distance_to_lane_split(seg, lane_index, extind)
    df[frameind,:d_merge] = isinf(d_merge) ? NA : d_merge
    df[frameind,:d_split] = isinf(d_split) ? NA : d_split

    nll, nlr = StreetNetworks.num_lanes_on_sides(seg, lane_index, extind)
    @assert(nll ≥ 0)
    @assert(nlr ≥ 0)
    df[frameind,:nll] = nll
    df[frameind,:nlr] = nlr

    lane_width_left, lane_width_right = marker_distances(seg, lane_index, extind)
    df[frameind,:d_mr] = (dcl <  lane_width_left)  ?  lane_width_left - dcl  : Inf
    df[frameind,:d_ml] = (dcl > -lane_width_right) ?  dcl - lane_width_right : Inf

    extracted
end
function _set_vehicle!(
    extracted::ExtractedTrajdef,
    sn::StreetNetwork,
    frameind::Integer,
    inertial::VecSE2,
    speed::Float64, # [m/s]
    footpoint::CurvePt,
    extind::Float64,
    lanetag::LaneTag,
    dcl::Float64,
    ϕ::Float64,
    )

    proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
    @assert(proj.successful)

    if proj.laneid == convert(LaneID, lanetag)
        _set_vehicle_nocheck!(extracted, sn, frameind, inertial, speed,
                              footpoint, extind, lanetag, dcl, ϕ)
    else
        footpoint = proj.curvept
        extind = proj.extind
        lanetag = LaneTag(proj.tile, proj.laneid)
        s, dcl, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)    

        _set_vehicle_nocheck!(extracted, sn, frameind, inertial, speed,
                              footpoint, extind, lanetag, dcl, ϕ)
    end
end
function _add_trajlink_to_extracted!(
    extracted::ExtractedTrajdef,
    sn::StreetNetwork,
    sdot₁::Float64,
    sddot₁::Float64,
    d₁::Float64,
    ddot₁::Float64,
    dddot₁::Float64,
    link::TrajDefLink,
    lane::StreetLane,
    extind_start::Float64, # extind we were previously at, for guessing
    sec_per_frame::Float64,
    inertial::VecSE2,
    footpoint::CurvePt,
    frameind::Int,
    )

    τ = link.n_pdset_frames * sec_per_frame

    d₂ = link.d # final lateral offset
    ddot₂ = link.v * sin(link.ϕ) # final lateral velocity
    dddot₂ = link.a * sin(link.ψ) # final lateral accel

    sdot₂ = link.v * cos(link.ϕ) # final longitudinal velocity
    sddot₂ = link.a * cos(link.ψ) # final longitudinal accel

    # Compute a trajectory in which the lateral start and end states are given
    # and where the longitudinal trajectory has unspecified end position
    poly_s = get_quartic_coefficients(0.0, sdot₁, sddot₁, sdot₂, sddot₂, τ)
    poly_d = get_quintic_coefficients(d₁, ddot₁, dddot₁, d₂, ddot₂, dddot₂, τ)

    t = 0.0
    prev_extind = extind_start
    lanetag = LaneTag(sn, lane)
    new_footpoint = footpoint
    footpoint_s_ajdust = 0.0
    for _ in 1 : link.n_pdset_frames
        t += sec_per_frame

        Δs, sdot = p₁₂(poly_s, float(t))
        new_d, ddot = p₁₂(poly_d, float(t))

        new_s = footpoint.s + footpoint_s_ajdust + Δs
        new_ϕ = atan2(ddot, sdot)
        new_extind = closest_point_extind_to_curve_guess(lane.curve, new_s, prev_extind)
        while is_extind_at_curve_end(lane.curve, new_extind)

            new_s -= lane.nodes[end].d_along
            footpoint_s_ajdust -= lane.nodes[end].d_along

            lane = next_lane(sn, lane)
            lanetag = LaneTag(sn, lane)

            new_s -= lane.nodes[2].d_along
            footpoint_s_ajdust -= lane.nodes[2].d_along
            
            new_extind = closest_point_extind_to_curve_guess(lane.curve, new_s, 1.5)
        end

        new_footpoint = curve_at(lane.curve, new_extind) 
        new_footvec = VecSE2(new_footpoint.x, new_footpoint.y, new_footpoint.θ)       
        inertial = new_footvec + Vec.polar(new_d, π/2 + new_footvec.θ, new_footvec.θ + new_ϕ)

        frameind += 1
        speed = hypot(sdot, ddot)
        _set_vehicle!(extracted, sn, frameind, inertial, speed,
                      new_footpoint, new_extind, lanetag, new_d, new_ϕ)

        prev_extind = new_extind
    end

    (inertial, new_footpoint, prev_extind, d₂, ddot₂, dddot₂, sdot₂, sddot₂, frameind, lanetag)
end

function extract_trajdef(
    sn::StreetNetwork,
    trajdef::TrajDef,
    carid::Integer,
    frameind_start::Integer,
    sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
    )

    #=
    produces a DataFrame with the same form as df_ego_primary
    with the trajdef contents
    =#

    nframes = get_num_pdset_frames(trajdef)
    df = Trajdata._create_df_ego_primary(nframes)
    extracted = ExtractedTrajdef(df, carid)
    for i = 1 : nrow(df)
        df[i,:frame] = frameind_start + i - 1
        df[i,:time] = sec_per_frame*i
    end

    lane = get_lane(sn, trajdef.lanetag)
    footpoint = curve_at(lane.curve, trajdef.extind)
    footvec = VecSE2(footpoint.x, footpoint.y, footpoint.θ)
    inertial = footvec + Vec.polar(trajdef.d, π/2 + footvec.θ, footvec.θ + trajdef.ϕ)

    frameind = 1
    _set_vehicle!(extracted, sn, frameind, inertial, trajdef.v, 
                  footpoint, trajdef.extind, trajdef.lanetag, trajdef.d, trajdef.ϕ)

    #######################################
    # Now we select the next link, project our current location to the new lane tag
    # and move forward!

    a = trajdef.a
    v = trajdef.v
    ψ = trajdef.ψ + footvec.θ # in global coordinates

    link_index = 1
    link = trajdef.links[link_index]
    lanetag = link.lanetag

    lane = get_lane(sn, lanetag)
    curve = lane.curve

    # NOTE: using the same inertial VecSE2
    n_lane_jumps = 0
    extind = closest_point_extind_to_curve(curve, inertial.x, inertial.y)
    while is_extind_at_curve_end(curve, extind) && n_lane_jumps < 5
        n_lane_jumps += 1
        if extind > 1.0
            @assert(has_next_lane(sn, lane))
            lane = next_lane(sn, lane)
            curve = lane.curve
            extind = closest_point_extind_to_curve(curve, inertial.x, inertial.y)
        end
    end

    footpoint = curve_at(curve, extind)

    s, d₁, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)
    ddot₁ = v * sin(ϕ) # initial lateral velocity
    dddot₁ = a * sin(ψ - footpoint.θ) # initial lateral accel

    sdot₁ = v * cos(ϕ) # initial longitudinal velocity
    sddot₁ = a * cos(ψ - footpoint.θ) # initial longitudinal accel

    (inertial, footpoint, extind, d₂, ddot₂, dddot₂, sdot₂, sddot₂, frameind, lanetag) = _add_trajlink_to_extracted!(
                                                                                            extracted, sn, sdot₁, sddot₁, d₁, ddot₁, dddot₁, link,
                                                                                            lane, extind, sec_per_frame, inertial, footpoint, frameind)

    while link_index < length(trajdef.links)
        link_index += 1

        if trajdef.links[link_index].lanetag == lanetag
            link = trajdef.links[link_index]
            d₁     = d₂
            ddot₁  = ddot₂
            dddot₁ = dddot₂
            sdot₁  = sdot₂
            sddot₁ = sddot₂
        else
            v = hypot(ddot₂, sdot₂)
            a = hypot(dddot₂, sddot₂)

            ϕ = trajdef.links[1].ϕ
            ψ = trajdef.links[1].ψ

            link = trajdef.links[link_index]
            lanetag = link.lanetag

            lane = get_lane(sn, lanetag)
            curve = lane.curve

            n_lane_jumps = 0
            extind = closest_point_extind_to_curve(curve, inertial.x, inertial.y)
            while is_extind_at_curve_end(curve, extind) && n_lane_jumps < 5
                n_lane_jumps += 1
                if extind > 1.0
                    @assert(has_next_lane(sn, lane))
                    lane = next_lane(sn, lane)
                    curve = lane.curve
                    extind = closest_point_extind_to_curve(curve, inertial.x, inertial.y)
                end
            end

            @assert(!is_extind_at_curve_end(curve, extind))
            footpoint = curve_at(curve, extind)

            s, d₁, ϕ = pt_to_frenet_xyy(footpoint, inertial.x, inertial.y, inertial.θ)
            ddot₁ = v * sin(ϕ) # initial lateral velocity
            dddot₁ = a * sin(ψ - footpoint.θ) # initial lateral accel

            sdot₁ = v * cos(ϕ) # initial longitudinal velocity
            sddot₁ = a * cos(ψ - footpoint.θ) # initial longitudinal accel
        end

        (inertial, footpoint, extind, d₂, ddot₂, dddot₂, sdot₂, sddot₂, frameind) = _add_trajlink_to_extracted!(
                                                                                        extracted, sn, sdot₁, sddot₁, d₁, ddot₁, dddot₁, link,
                                                                                        lane, extind, sec_per_frame, inertial, footpoint, frameind)
    end

    extracted
end
function extract_trajdefs(
    basics::FeatureExtractBasicsPdSet,
    trajectories::Vector{Vector{TrajDefLink}},
    active_carid::Integer,
    validfind::Integer,
    sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
    )

    extracted_trajdefs = Array(ExtractedTrajdef, length(trajectories))
    trajdef = TrajDef(basics.pdset, basics.sn, active_carid, validfind)
    for (i,links) in enumerate(trajectories)
        new_trajdef = copy_trajdef_start(trajdef)
        append!(new_trajdef.links, links)
        frameind_start = validfind2frameind(basics.pdset, validfind)
        extracted_trajdefs[i] = extract_trajdef(basics.sn, new_trajdef, active_carid, frameind_start, sec_per_frame)
    end
    extracted_trajdefs
end

function copy_trajdef_start(trajdef::TrajDef)
    TrajDef(trajdef.lanetag,
            trajdef.extind,
            trajdef.d,
            trajdef.v,
            trajdef.a,
            trajdef.ϕ,
            trajdef.ψ,
            TrajDefLink[]
            )
end

function Base.insert!(
    pdset::PrimaryDataset,
    extracted::ExtractedTrajdef
    )
    
    # NOTE(tim): this assumes pdset already contains the appropriate frameinds
    #            and validfinds

    source = extracted.df

    if extracted.carid == CARID_EGO
        for sym in (:posGx, :posGy, :posGyaw, :posFyaw, :velFx, :velFy, :lanetag,
                    :curvature, :d_cl, :d_merge, :d_split, :nll, :nlr, :d_mr, :d_ml)
            for (i,frameind) in enumerate(source[:frame])
                pdset.df_ego_primary[frameind,sym] = source[i,sym]
            end
        end
    else
        for (i,frameind) in enumerate(source[:frame])

            validfind = frameind2validfind(pdset, frameind)
            
            carind = carid2ind_or_negative_two_otherwise(pdset, extracted.carid, validfind)
            if carind == -2
                add_car_to_validfind!(pdset, carid, validfind)
                carind = carid2ind(pdset, extracted.carid, validfind)
            end
            
            baseindex = pdset.df_other_ncol_per_entry * carind   

            for sym in (:posGx, :posGy, :posGyaw, :posFyaw, :velFx, :velFy, :lanetag,
                        :curvature, :d_cl, :d_merge, :d_split, :nll, :nlr, :d_mr, :d_ml,
                        :id, :t_inview)

                col = pdset.df_other_column_map[sym] + baseindex
                pdset.df_other_primary[frameind,col] = source[i,sym]
            end
        end
    end 

    pdset   
end
function Base.insert!(
    pdset::PrimaryDataset,
    extracted::ExtractedTrajdef,
    frameind_start::Integer,
    frameind_end::Integer
    )
    
    # NOTE(tim): this assumes pdset already contains the appropriate frameinds
    #            and validfinds

    source = extracted.df

    frameind_max = nframeinds(pdset)
    if frameind_max < frameind_start
        warn("Base.insert! - no frames copied")
        return pdset
    end
    frameind_end = min(frameind_max, frameind_end)

    extracted_row_start = frameind_start - source[1,:frame]::Int + 1
    extracted_row_end   = frameind_end + extracted_row_start - frameind_start

    if extracted.carid == CARID_EGO
        for sym in (:posGx, :posGy, :posGyaw, :posFyaw, :velFx, :velFy, :lanetag,
                    :curvature, :d_cl, :d_merge, :d_split, :nll, :nlr, :d_mr, :d_ml)
            for (extracted_row, frameind) in zip(extracted_row_start:extracted_row_end, frameind_start:frameind_end)
                pdset.df_ego_primary[frameind,sym] = source[extracted_row,sym]
            end
        end
    else
        # NOTE(tim): this assumes that validfinds are continuous in this sequence
        validfind_start = frameind2validfind(pdset, frameind_start)
        validfind_end = validfind_start + extracted_row_start - frameind_start
        for (extracted_row, validfind) in zip(extracted_row_start:extracted_row_end, validfind_start:validfind_end)
            
            carind = carid2ind_or_negative_two_otherwise(pdset, extracted.carid, validfind)
            if carind == -2
                add_car_to_validfind!(pdset, carid, validfind)
                carind = carid2ind(pdset, extracted.carid, validfind)
            end
            
            baseindex = pdset.df_other_ncol_per_entry * carind   

            for sym in (:posGx, :posGy, :posGyaw, :posFyaw, :velFx, :velFy, :lanetag,
                        :curvature, :d_cl, :d_merge, :d_split, :nll, :nlr, :d_mr, :d_ml,
                        :id, :t_inview)

                col = pdset.df_other_column_map[sym] + baseindex
                pdset.df_other_primary[frameind,col] = source[i,sym]
            end
        end
    end 

    pdset   
end
