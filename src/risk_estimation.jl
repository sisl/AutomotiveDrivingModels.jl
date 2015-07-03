export
    calc_collision_risk_monte_carlo!,
    calc_future_grid_counts!


type Scenario
    
end

function calc_collision_risk_monte_carlo!{B<:AbstractVehicleBehavior}(
    simlog       :: Matrix{Float64}, # initialized appropriately
    behaviors    :: Vector{B},
    road         :: StraightRoadway,
    history      :: Int, # starting index within simlog
    nsimulations :: Int=100,
    params       :: SimParams = SimParams();
    endframe     :: Int = get_nframes(simlog)
    )

    # NOTE(tim): Vehicles with deterministic trajectories should have those trajectories in simlog
    #            Those vehicles should be using VEHICLE_BEHAVIOR_NONE

    runid::Int = rand(Int)
    

    ncollisions = 0
    for i = 1 : nsimulations
        simulate!(simlog, behaviors, road, history, params, runid=runid+i)
        ncollisions += intersects(simlog, startframe=history+1, endframe=endframe)
    end
    ncollisions / nsimulations
end

function calc_future_grid_counts!{B<:AbstractVehicleBehavior}(
    gridcounts::Vector{Matrix{Float64}}, # NOTE(tim): one for [horizon] states in future, does not include start state
    histobin_params::ParamsHistobin,
    simlog::Matrix{Float64}, # initialized appropriately
    behaviors::Vector{B},
    road::StraightRoadway,
    history::Int, # starting index within simlog
    carind::Int,
    nsimulations::Int=100,
    params::SimParams = SimParams(),
    )

    runid::Int = rand(Int)

    for gridcount in gridcounts
        fill!(gridcount, 0.0)
    end

    logindexbase = calc_logindexbase(carind)
    x₀ = simlog[history, logindexbase+LOG_COL_X]
    y₀ = simlog[history, logindexbase+LOG_COL_Y]

    for i = 1 : nsimulations
        simulate!(simlog, behaviors, road, history, params, runid=runid+i)
        
        for (j,gridcount) in enumerate(gridcounts)
            frameind = history + j

            Δx = simlog[frameind, logindexbase+LOG_COL_X] - x₀
            Δy = simlog[frameind, logindexbase+LOG_COL_Y] - y₀

            bin_x = encode(histobin_params.discx, Δx)
            bin_y = encode(histobin_params.discy, Δy)

            gridcount[bin_x, bin_y] += 1.0
        end
    end

    gridcounts
end