immutable Frenet
    roadind::RoadIndex
    s::Float64 # distance along lane
    t::Float64 # lane offset, positive is to left
    ϕ::Float64 # lane relative heading
end
const NULL_FRENET = Frenet(NULL_ROADINDEX, NaN, NaN, NaN)
function Base.isapprox(a::Frenet, b::Frenet;
    rtol::Real=cbrt(eps(Float64)),
    atol::Real=sqrt(eps(Float64))
    )

    a.roadind.tag == b.roadind.tag &&
    a.roadind.ind.i == b.roadind.ind.i &&
    isapprox(a.roadind.ind.t, b.roadind.ind.t, atol=atol, rtol=rtol) &&
    isapprox(a.s, b.s, atol=atol, rtol=rtol) &&
    isapprox(a.t, b.t, atol=atol, rtol=rtol) &&
    isapprox(a.ϕ, b.ϕ, atol=atol, rtol=rtol)
end

immutable VehicleState
    posG::VecSE2 # global
    posF::Frenet # lane-relative frame
    v::Float64

    VehicleState() = new(VecSE2(), NULL_FRENET, NaN)
    VehicleState(posG::VecSE2, v::Float64) = new(posG, NULL_FRENET, v)
    VehicleState(posG::VecSE2, posF::Frenet, v::Float64) = new(posG, posF, v)
end

baremodule AgentClass
    const CAR        = 1
    const MOTORCYCLE = 2
    const TRUCK      = 3
end

type Vehicle
    id::Int
    class::Int # ∈ AgentClass
    length::Float64
    width::Float64
    state::VehicleState

    Vehicle() = new(0, AgentClass.Car, NaN, NaN, VehicleState())
    function Vehicle(
        id::Int,
        class::Int,
        length::Float64,
        width::Float64,
        state::VehicleState=VehicleState()
        )
        new(id, class, length, width, state)
    end
end

get_vel_s(s::VehicleState) = s.v * cos(s.posF.ϕ) # velocity along the lane
get_vel_t(s::VehicleState) = s.v * sin(s.posF.ϕ) # velocity ⟂ to lane

get_footpoint(veh::Vehicle) = veh.state.posG + polar(veh.state.posF.t, veh.state.posG.θ-veh.state.posF.ϕ-π/2)
get_center(veh::Vehicle) = veh.state.posG + polar(veh.length/2, veh.state.posG.θ+π)

# function get_headway_dist_between(veh_rear::Vehicle, veh_fore::Vehicle)

#     # distance from the front of the rear vehicle to the rear of the front vehicle

#     active_laneid = veh_rear.state.posF.laneid
#     if veh_fore.state.posF.laneid != active_laneid
#         NaN
#     else
#         s1 = veh_rear.state.posF.s
#         s2 = veh_fore.state.posF.s
#         s2 - s1 - veh_fore.length
#     end
# end
# function get_headway_time_between(veh_rear::Vehicle, veh_fore::Vehicle)

#     active_laneid = veh_rear.state.posF.laneid

#     if veh_fore.state.posF.laneid != active_laneid
#         NaN
#     else
#         s1 = veh_rear.state.posF.s
#         s2 = veh_fore.state.posF.s
#         Δs = s2 - s1 - veh_fore.length
#         Δs/veh_rear.state.v
#     end
# end