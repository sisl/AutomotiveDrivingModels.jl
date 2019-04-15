"""
    CrossingPhase
Crossing phases for SidewalkPedestrianModel.

For a crossing pedestrian, phases go: -2, -1, 0, 1
For a non-crossing pedestrian that pauses at the crosswalk, phases go: -2, -1, 1
For a non-crossing pedestrian that does not pause at the crosswalk, phases go: -2, 1

Model based on Feliciani et al (2017) - A simulation model for non-signalized pedestrian
crosswalks based on evidence from on field observation.
"""
struct CrossingPhase
    phase::Int # -2, -1, 0, 1
end

const APPROACHING = -2 # Keep walking at constant speed
const APPRAISING = -1 # Decelerate until reach curbside
const CROSSING = 0 # Accelerate until desired speed
const LEAVING = 1 # Keep walking at constant speed

"""
    SidewalkPedestrianModel

Walks along the sidewalk until approaching the crosswalk. Waits for the cars to pass, then crosses.

# Fields
- `timestep::Float64`
- `phase::Int = APPROACHING`
- `ttc_threshold::Float64 = clamp(rand(Normal(4.0, 2.5)), 1.0, Inf)`
- `crosswalk::Lane = Lane()`
- `sw_origin::Lane = Lane()`
- `sw_dest::Lane = Lane()`
- `a::PedestrianLatLonAccel = PedestrianLatLonAccel(0.0, 0.0, sw_origin)` makes you turn, left/right
- `σ::Float64 = NaN` optional stdev on top of the model, set to zero or NaN for deterministic
- `v_des_approach::Float64 = clamp(rand(Normal(1.28, 0.18)), 0.0, Inf)` Based on Feliciani et al. results
- `v_des_appraise::Float64 = clamp(rand(Normal(0.94, 0.21)), 0.0, Inf)`
- `v_des_cross::Float64 = clamp(rand(Normal(1.35, 0.18)), 0.0, Inf)`
- `ped_accel::Float64 = 0.30`
- `ped_decel::Float64 = -0.50`
"""
@with_kw mutable struct SidewalkPedestrianModel <: DriverModel{PedestrianLatLonAccel}
    timestep::Float64
    phase::Int = APPROACHING

    ttc_threshold::Float64 = clamp(rand(Normal(4.0, 2.5)), 1.0, Inf)
    crosswalk::Lane = Lane()
    sw_origin::Lane = Lane()
    sw_dest::Lane = Lane()
    # conflict_lanes::Vector{Lane} = Lane[]

    # Values from Feliciani
    a::PedestrianLatLonAccel = PedestrianLatLonAccel(0.0, 0.0, sw_origin) # makes you turn, left/right
    σ::Float64 = NaN # optional stdev on top of the model, set to zero or NaN for deterministic behavior
    v_des_approach::Float64 = clamp(rand(Normal(1.28, 0.18)), 0.0, Inf) # Based on Feliciani et al. results
    v_des_appraise::Float64 = clamp(rand(Normal(0.94, 0.21)), 0.0, Inf)
    v_des_cross::Float64 = clamp(rand(Normal(1.35, 0.18)), 0.0, Inf)

    ped_accel::Float64 = 0.30
    ped_decel::Float64 = -0.50

    # For graphing
    pos_x = Float64[]
    pos_y = Float64[]
    velocities = Float64[]
    phases = Int[]
end

AutomotiveDrivingModels.get_name(model::SidewalkPedestrianModel) = "SidewalkPedestrianModel"
Base.rand(model::SidewalkPedestrianModel) = model.a

function AutomotiveDrivingModels.observe!(model::SidewalkPedestrianModel, scene::Scene, roadway::Roadway, egoid::Int)
    ped = scene[findfirst(egoid, scene)]

    push!(model.pos_x, ped.state.posG.x)
    push!(model.pos_y, ped.state.posG.y)
    push!(model.velocities, ped.state.v)
    push!(model.phases, model.phase)

    if model.phase == APPROACHING
        update_approaching(ped, model, roadway, model.crosswalk, model.sw_origin)
    elseif model.phase == APPRAISING
        update_appraising(ped, scene, model, roadway, model.crosswalk, model.sw_origin)
    elseif model.phase == CROSSING
        update_crossing(ped, scene, model, roadway, model.crosswalk)
    elseif model.phase == LEAVING
        update_leaving(ped, scene, model, roadway, model.sw_dest)
    else
        # Model phase is out of bounds
    end
end

function update_approaching(ped::Vehicle,
                            model::SidewalkPedestrianModel,
                            roadway::Roadway,
                            crosswalk::Lane,
                            sw_origin::Lane
                            )
    # Keep walking at constant speed until close to crosswalk
    # If waiting to cross, go to appraising. Else, go to leaving phase
    Δs = get_distance_to_lane(ped, crosswalk, roadway)
    if Δs < 3.0
        model.phase = APPRAISING
    end
end

function update_appraising(ped::Vehicle,
                           scene::Scene,
                           model::SidewalkPedestrianModel,
                           roadway::Roadway,
                           crosswalk::Lane,
                           sw_origin::Lane,
                           )

    Δs = get_distance_to_lane(ped, crosswalk, roadway)
    if Δs < 0.1
        set_inst_vel(ped, model, sw_origin, 0.0)

        for veh in scene
            if veh.def.class == AgentClass.PEDESTRIAN
                continue
            end

            # Check vehicle distance to the crosswalk
            posG = veh.state.posG
            cw_posF = Frenet(posG, model.crosswalk, roadway)
            lane = get_lane(roadway, veh)
            Δs = get_distance_to_lane(veh, model.crosswalk, roadway)
            ttc = Δs/veh.state.v

            if 0.0 < ttc < model.ttc_threshold
                return # Exit if any car is not within threshold
            end
        end
        model.phase = CROSSING
    else
        # Decelerate to appraise speed
        set_accel(ped, model, model.v_des_appraise, sw_origin)
    end
end


function update_crossing(ped::Vehicle,
                         scene::Scene,
                         model::SidewalkPedestrianModel,
                         roadway::Roadway,
                         crosswalk::Lane
                         )
    # Accelerate until desired speed
    set_accel(ped, model, model.v_des_cross, crosswalk)
    # If ped is at end of crosswalk, transition to leaving phase.
    Δs = get_distance_to_lane(ped, model.sw_dest, roadway)

    if Δs < 0.2
        set_inst_vel(ped, model, model.crosswalk, 0.0)
        model.phase = LEAVING
    end
end


function update_leaving(ped::Vehicle,
                        scene::Scene,
                        model::SidewalkPedestrianModel,
                        roadway::Roadway,
                        sw_dest::Lane
                        )
    # Keep walking at constant speed
    set_accel(ped, model, model.v_des_approach, model.sw_dest)
end

function set_inst_vel(ped::Vehicle, model::SidewalkPedestrianModel, lane_des::Lane, v_des::Float64)
    model.a = PedestrianLatLonAccel(0.0, calc_a(ped.state.v, v_des, model.timestep), lane_des)
end

function calc_a(v_init::Float64, v::Float64, Δt::Float64)
    return (v - v_init)/Δt
end

function set_accel(ped::Vehicle, model::SidewalkPedestrianModel, v_des::Float64, lane_des::Lane)
    if ped.state.v < v_des
        model.a = PedestrianLatLonAccel(0.0, model.ped_accel, lane_des)
    elseif ped.state.v > v_des
        model.a = PedestrianLatLonAccel(0.0, model.ped_decel, lane_des)
    else
        model.a = PedestrianLatLonAccel(0.0, 0.0, lane_des)
    end
end


# Edited from AutomotivePOMDPs.jl - projects collision point onto the lane.
# Should work in any orientation.
function get_distance_to_lane(veh::Vehicle, lane_des::Lane, roadway::Roadway)
    vehlane = get_lane(roadway, veh)
    lane_segment = LineSegment(lane_des.curve[1].pos, lane_des.curve[end].pos)
    collision_point = intersect(Ray(veh.state.posG), lane_segment)
    lane_des_proj = Frenet(VecSE2(collision_point), vehlane, roadway)
    Δs = lane_des_proj.s - veh.state.posF.s
    return Δs
end

function get_lane(roadway::Roadway, vehicle::Vehicle)
    lane_tag = vehicle.state.posF.roadind.tag
    return roadway[lane_tag]
end
