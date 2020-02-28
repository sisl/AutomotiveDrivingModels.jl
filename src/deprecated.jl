@deprecate get_vel_s velf(state).s
@deprecate get_vel_t velf(state).t

@deprecate propagate(veh, state, roadway, Δt) propagate(veh, state, a, roadway, Δt)

function simulate!(
    scene::Frame{E},
    roadway::R,
    models::Dict{I,M},
    nticks::Int64,
    timestep::Float64;
    rng::AbstractRNG = Random.GLOBAL_RNG,
    callbacks = nothing
) where {E<:Entity,A,R,I,M<:DriverModel}
    Base.depwarn(
"`simulate!` without specifying a pre-allocated data structure for `scenes` is now deprecated.\n
You probably want to use the `simulate` function instead.\n
Alternatively, you can provide a pre-allocated data structure via the `scenes=` keyword",
        :simulate_no_prealloc
    )
    return simulate(scene, roadway, models, nticks, timestep, rng=rng, callbacks=callbacks)
end

"""
    # TODO: this should be removed, but where to document the function then?
    run_callback(callback::Any, scenes::Vector{F}, roadway::R, models::Dict{I,M}, tick::Int) where {F,I,R,M<:DriverModel}    
    run_callback(callback::Any, rec::EntityQueueRecord{S,D,I}, roadway::R, models::Dict{I,M}, tick::Int) where {S,D,I,R,M<:DriverModel}
run callback and return whether simlation should terminate
A new method should be implemented when defining a new callback object.
"""
function run_callback(callback, scenes, actions::Union{Nothing, Vector{Frame{A}}}, roadway, models, tick) where {A<:EntityAction}
    Base.depwarn(
"Using a deprecated version of `run_callback`. Since v0.7.10, user-defined callback functions should also take an `actions` argument.
 If you have implemented `run_callback` with an actions argument, make sure the method signature is more specific than this one.\n
Check the section `Simulation > Callbacks` in the package documentation for more information
 on how to define callback functions.\n
Your function call is being forwarded to `run_callback` without actions argument.",
        :run_callback_no_actions
    )
    return run_callback(callback, scenes, roadway, models, tick)
end

# 1D state

state_1d_dep_msgs = """
    State1D, Vehicle1D, and Scene1D are deprecated, use VehicleState, Vehicle, and Scene instead.
"""

"""
    State1D

A data type to represent one dimensional states

# Fields 
    - `s::Float64` position 
    - `v::Float64` speed [m/s]
"""
struct State1D
    s::Float64 # position
    v::Float64 # speed [m/s]
end

function vel(s::State1D)
    Base.depwarn(state_1d_dep_msgs, :vel)
    return s.v
end

function Base.write(io::IO, ::MIME"text/plain", s::State1D)
    Base.depwarn(state_1d_dep_msgs, :write)
    @printf(io, "%.16e %.16e", s.s, s.v)
end
function Base.read(io::IO, ::MIME"text/plain", ::Type{State1D})
    Base.depwarn(state_1d_dep_msgs, :read)
    i = 0
    tokens = split(strip(readline(io)), ' ')
    s = parse(Float64, tokens[i+=1])
    v = parse(Float64, tokens[i+=1])
    return State1D(s,v)
end

"""
    Vehicle1D
A specific instance of the Entity type defined in Records.jl to represent vehicles in 1d environments.
"""
const Vehicle1D = Entity{State1D, VehicleDef, Int64}

"""
    Scene1D

A specific instance of the Frame type defined in Records.jl to represent a list of vehicles in 1d environments.

# constructors
    Scene1D(n::Int=100)
    Scene1D(arr::Vector{Vehicle1D})
"""
const Scene1D = Frame{Vehicle1D}

function Scene1D(n::Int=100)
    Base.depwarn(state_1d_dep_msgs, :Scene1D)
    Frame(Vehicle1D, n)
end

function Scene1D(arr::Vector{Vehicle1D})
    Base.depwarn(state_1d_dep_msgs, :Scene1D)
    Frame{Vehicle1D}(arr, length(arr))
end

function get_center(veh::Vehicle1D) 
    Base.depwarn(state_1d_dep_msgs, :get_center)
    veh.state.s 
end
function get_footpoint(veh::Vehicle1D) 
    Base.depwarn(state_1d_dep_msgs, :get_footpoint)
    veh.state.s 
end
function get_front(veh::Vehicle1D) 
    Base.depwarn(state_1d_dep_msgs, :get_front)
    veh.state.s + length(veh.def)/2 
end
function get_rear(veh::Vehicle1D) 
    Base.depwarn(state_1d_dep_msgs, :get_rear)
    veh.state.s - length(veh.def)/2 
end


function get_headway(veh_rear::Vehicle1D, veh_fore::Vehicle1D, roadway::StraightRoadway)
    Base.depwarn(state_1d_dep_msgs, :get_headway)
    return get_headway(get_front(veh_rear), get_rear(veh_fore), roadway)
end
function get_neighbor_fore(scene::Frame{Entity{State1D, D, I}}, vehicle_index::I, roadway::StraightRoadway) where {D, I}
    Base.depwarn(state_1d_dep_msgs, :get_neighbor_fore)
    ego = scene[vehicle_index]
    best_ind = nothing
    best_gap = Inf
    for (i,veh) in enumerate(scene)
        if i != vehicle_index
            Δs = get_headway(ego, veh, roadway)
            if Δs < best_gap
                best_gap, best_ind = Δs, i
            end
        end
    end
    return NeighborLongitudinalResult(best_ind, best_gap)
end
function get_neighbor_rear(scene::Frame{Entity{State1D, D, I}}, vehicle_index::I, roadway::StraightRoadway) where {D, I}
    Base.depwarn(state_1d_dep_msgs, :get_neighbor_rear)
    ego = scene[vehicle_index]
    best_ind = nothing
    best_gap = Inf
    for (i,veh) in enumerate(scene)
        if i != vehicle_index
            Δs = get_headway(veh, ego, roadway)
            if Δs < best_gap
                best_gap, best_ind = Δs, i
            end
        end
    end
    return NeighborLongitudinalResult(best_ind, best_gap)
end

function propagate(veh::Vehicle1D, action::LaneFollowingAccel, roadway::StraightRoadway, Δt::Float64)
    Base.depwarn(state_1d_dep_msgs, :propagate)
    a = action.a
    s, v = veh.state.s, veh.state.v

    s′ = s + v*Δt + a*Δt*Δt/2
    v′ = max(v + a*Δt, 0.)  # no negative velocities

    s′ = mod_position_to_roadway(s′, roadway)

    return State1D(s′, v′)
end


function observe!(model::LaneFollowingDriver, scene::Frame{Entity{State1D, D, I}}, roadway::StraightRoadway, egoid::I) where {D, I}
    Base.depwarn(state_1d_dep_msgs, :observe!)
    vehicle_index = findfirst(egoid, scene)

    fore_res = get_neighbor_fore(scene, vehicle_index, roadway)

    v_ego = vel(scene[vehicle_index].state)
    v_oth = vel(scene[fore_res.ind].state)
    headway = fore_res.Δs

    track_longitudinal!(model, v_ego, v_oth, headway)

    return model
end
