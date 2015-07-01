
function calc_collision_risk_monte_carlo!{B<:AbstractVehicleBehavior}(
    simlog       :: Matrix{Float64}, # initialized appropriately
    behaviors    :: Vector{B},
    road         :: StraightRoadway,
    frameind     :: Int, # starting index within simlog
    nsimulations :: Int=100,
    params       :: SimParams = SimParams(),
    )

    # NOTE(tim): Vehicles with deterministic trajectories should have those trajectories in simlog
    #            Those vehicles should be using VEHICLE_BEHAVIOR_NONE

    runid::Int = rand(Int)
    nframes=get_nframes(simlog)

    ncollisions = 0
    for i = 1 : nsimulations
        simulate!(simlog, behaviors, road, frameind, params, runid+i)
        ncollisions += intersects(simlog, startframe=frameind+1, endframe=nframes)
    end
    ncollisions / nsimulations
end