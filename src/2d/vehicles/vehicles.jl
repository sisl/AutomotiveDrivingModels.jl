immutable VehicleState
    posG::VecSE2 # global
    posF::Frenet # lane-relative frame
    v::Float64

    VehicleState() = new(VecSE2(), NULL_FRENET, NaN)
    VehicleState(posG::VecSE2, v::Float64) = new(posG, NULL_FRENET, v)
    VehicleState(posG::VecSE2, posF::Frenet, v::Float64) = new(posG, posF, v)
    VehicleState(posG::VecSE2, roadway::Roadway, v::Float64) = new(posG, Frenet(posG, roadway), v)
    VehicleState(posG::VecSE2, lane::Lane, roadway::Roadway, v::Float64) = new(posG, Frenet(posG, roadway), v)
    VehicleState(posF::Frenet, roadway::Roadway, v::Float64) = new(get_posG(posF, roadway), posF, v)
end
Base.show(io::IO, s::VehicleState) = print(io, "VehicleState(", s.posG, ", ", s.posF, ", ", @sprintf("%.3f", s.v), ")")
function Base.write(io::IO, ::MIME"text/plain", s::VehicleState)
    @printf(io, "%.16e %.16e %.16e", s.posG.x, s.posG.y, s.posG.θ)
    @printf(io, " %d %.16e %d %d", s.posF.roadind.ind.i, s.posF.roadind.ind.t, s.posF.roadind.tag.segment, s.posF.roadind.tag.lane)
    @printf(io, " %.16e %.16e %.16e", s.posF.s, s.posF.t, s.posF.ϕ)
    @printf(io, " %.16e", s.v)
end
function Base.read(io::IO, ::MIME"text/plain", ::Type{VehicleState})
    tokens = split(strip(readline(io)), ' ')
    i = 0
    posG = VecSE2(parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]))
    roadind = RoadIndex(CurveIndex(parse(Int, tokens[i+=1]), parse(Float64, tokens[i+=1])),
                        LaneTag(parse(Int, tokens[i+=1]), parse(Int, tokens[i+=1])))
    posF = Frenet(roadind, parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]), parse(Float64, tokens[i+=1]))
    v = parse(Float64, tokens[i+=1])
    return VehicleState(posG, posF, v)
end

function Vec.lerp(a::VehicleState, b::VehicleState, t::Float64, roadway::Roadway)
    posG = lerp(a.posG, b.posG, t)
    v = lerp(a.v, b.v, t)
    VehicleState(posG, roadway, v)
end

get_vel_s(s::VehicleState) = s.v * cos(s.posF.ϕ) # velocity along the lane
get_vel_t(s::VehicleState) = s.v * sin(s.posF.ϕ) # velocity ⟂ to lane

function move_along(vehstate::VehicleState, roadway::Roadway, Δs::Float64;
    ϕ₂::Float64=vehstate.posF.ϕ, t₂::Float64=vehstate.posF.t, v₂::Float64=vehstate.v
    )

    roadind = move_along(vehstate.posF.roadind, roadway, Δs)
    try
        footpoint = roadway[roadind]
    catch
        println(roadind)
    end
    footpoint = roadway[roadind]
    posG = convert(VecE2, footpoint.pos) + polar(t₂, footpoint.pos.θ + π/2)
    posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ₂)
    VehicleState(posG, roadway, v₂)
end

typealias Vehicle Entity{VehicleState,VehicleDef,Int}

Base.show(io::IO, v::Vehicle) = print(io, "Vehicle(", v.id, ", ", v.state, ", ", v.def, ")")

get_center(veh::Vehicle) = veh.state.posG
get_footpoint(veh::Vehicle) = veh.state.posG + polar(veh.state.posF.t, veh.state.posG.θ-veh.state.posF.ϕ-π/2)
get_front(veh::Vehicle) = veh.state.posG + polar(veh.def.length/2, veh.state.posG.θ)
get_rear(veh::Vehicle) = veh.state.posG - polar(veh.def.length/2, veh.state.posG.θ)

function get_lane_width(veh::Vehicle, roadway::Roadway)

    lane = roadway[veh.state.posF.roadind.tag]

    if n_lanes_left(lane, roadway) > 0
        footpoint = get_footpoint(veh)
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        return -proj(footpoint, lane_left, roadway).curveproj.t
    else
        return lane.width
    end
end

function get_markerdist_left(veh::Vehicle, roadway::Roadway)
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 - t
end
function get_markerdist_right(veh::Vehicle, roadway::Roadway)
    t = veh.state.posF.t
    lane_width = get_lane_width(veh, roadway)
    return lane_width/2 + t
end

function propagate(veh::Vehicle, action::LaneFollowingAccel, roadway::Roadway, ΔT::Float64)

    a_lon = action.a

    ds = veh.state.v

    ΔT² = ΔT*ΔT
    Δs = ds*ΔT + 0.5*a_lon*ΔT²

    v₂ = ds + a_lon*ΔT

    roadind = move_along(veh.state.posF.roadind, roadway, Δs)
    posG = roadway[roadind].pos
    VehicleState(posG, roadway, v₂)
end