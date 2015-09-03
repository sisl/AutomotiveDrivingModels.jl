export

    allocate_gridcounts!,
    calc_start_points,

    calc_collision_risk_monte_carlo!,
    calc_future_grid_counts!,
    calc_collision_risk_and_future_grid_counts!


function calc_start_points(
    pdset::PrimaryDataset,
    behavior_pairs::Vector{(AbstractVehicleBehavior,Int)},
    validfind_start::Integer,
    start_points::Vector{VecE2}=Array(VecE2, length(behavior_pairs)),
    )

    n_cars_to_eval = length(behavior_pairs)

    for i in 1 : n_cars_to_eval
        carid = behavior_pairs[i][2]
        carind_start = carid2ind(pdset, carid, validfind_start)
        x = get(pdset, :posGx, carind_start, validfind_start)
        y = get(pdset, :posGy, carind_start, validfind_start)
        start_points[i] = VecE2(x,y)
    end

    start_points
end

function calc_collision_risk_monte_carlo!(
    basics          :: FeatureExtractBasicsPdSet,
    behavior_pairs  :: Vector{(AbstractVehicleBehavior,Int)}, # (behavior, carid)
    validfind_start :: Int,
    validfind_end   :: Int;

    nsimulations    :: Integer=100,
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int = 2
    )

    # NOTE(tim): Vehicles with deterministic trajectories should have those trajectories in basics.pdset
    #            Those vehicles should be using VEHICLE_BEHAVIOR_NONE
    
    basics.runid = rand(Int)
    ncollisions = 0
    for i = 1 : nsimulations
        
        basics.runid += 1
        ncollisions += simulate_but_terminate_if_collision!(basics, behavior_pairs, validfind_start, validfind_end,
                                pdset_frames_per_sim_frame=pdset_frames_per_sim_frame,
                                n_euler_steps=n_euler_steps)
    end
    ncollisions / nsimulations
end

function calc_future_grid_counts!(
    gridcounts      :: Vector{Matrix{Float64}},
    histobin_params :: ParamsHistobin,
    basics          :: FeatureExtractBasicsPdSet,
    carid           :: Integer,
    behavior_pairs  :: Vector{(AbstractVehicleBehavior,Int)}, # (behavior, carid)
    validfind_start :: Int,
    validfind_end   :: Int;

    nsimulations    :: Integer=100,
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int = 2
    )

    #=
    Compute the future gridcounts for the given carid
    =#

    for gridcount in gridcounts
        fill!(gridcount, 0.0)
    end

    carind_start = carid2ind(basics.pdset, carid, validfind_start)
    x₀ = get(basics.pdset, :posGx, carind_start, validfind_start)
    y₀ = get(basics.pdset, :posGy, carind_start, validfind_start)

    basics.runid = rand(Int)
    for i = 1 : nsimulations
        
        basics.runid += 1
        simulate!(basics, behavior_pairs, validfind_start, validfind_end,
                    pdset_frames_per_sim_frame=pdset_frames_per_sim_frame,
                    n_euler_steps=n_euler_steps)

        for (j,gridcount) in enumerate(gridcounts)
            validfind_final = validfind_start + j



            carind = carid2ind(basics.pdset, carid, validfind_final)
            x₁ = get(basics.pdset, :posGx, carind, validfind_final)
            y₁ = get(basics.pdset, :posGy, carind, validfind_final)

            Δs, Δd = frenet_distance_between_points(basics.sn, x₀, y₀, x₁, y₁)

            bin_s = encode(histobin_params.discx, Δs)
            bin_d = encode(histobin_params.discy, Δd)

            gridcount[bin_s, bin_d] += 1.0
        end
    end

    gridcounts
end
function calc_future_grid_counts!(
    gridcounts      :: Vector{Vector{Matrix{Float64}}},
    histobin_params :: ParamsHistobin,
    basics          :: FeatureExtractBasicsPdSet,
    behavior_pairs  :: Vector{(AbstractVehicleBehavior,Int)}, # (behavior, carid)
    validfind_start :: Int,
    validfind_end   :: Int;

    nsimulations    :: Integer=100,
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int=2,
    start_points    :: Vector{VecE2}=calc_start_points(basics.pdset, behavior_pairs, validfind_start)
    )

    #=
    Compute the future gridcounts for the given carids in behavior_pairs
    =#

    n_cars_to_eval = length(behavior_pairs)
    for i in 1 : n_cars_to_eval
        for gridcount in gridcounts[i]
            fill!(gridcount, 0.0)
        end
    end
    
    basics.runid = rand(Int)
    for i = 1 : nsimulations
        
        basics.runid += 1
        simulate!(basics, behavior_pairs, validfind_start, validfind_end,
                  pdset_frames_per_sim_frame=pdset_frames_per_sim_frame,
                  n_euler_steps=n_euler_steps)

        for k in 1 : n_cars_to_eval

            carid = behavior_pairs[k][2]
            x₀ = start_points[k].x
            y₀ = start_points[k].y

            for (j,gridcount) in enumerate(gridcounts[k])
                validfind_final = validfind_start + j

                carind = carid2ind(basics.pdset, carid, validfind_final)
                x₁ = get(basics.pdset, :posGx, carind, validfind_final)
                y₁ = get(basics.pdset, :posGy, carind, validfind_final)

                Δs, Δd = frenet_distance_between_points(basics.sn, x₀, y₀, x₁, y₁)

                bin_s = encode(histobin_params.discx, Δs)
                bin_d = encode(histobin_params.discy, Δd)

                gridcount[bin_s, bin_d] += 1.0
            end
        end
    end

    gridcounts
end

function calc_collision_risk_and_future_grid_counts!(
    gridcounts      :: Vector{Matrix{Float64}},
    histobin_params :: ParamsHistobin,
    basics          :: FeatureExtractBasicsPdSet,
    carid           :: Integer,
    behavior_pairs  :: Vector{(AbstractVehicleBehavior,Int)}, # (behavior, carid)
    validfind_start :: Int,
    validfind_end   :: Int;

    nsimulations    :: Integer=100,
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int = 2
    )

    #=
    Compute the future gridcounts for the given carid
    =#

    for gridcount in gridcounts
        fill!(gridcount, 0.0)
    end

    carind_start = carid2ind(basics.pdset, carid, validfind_start)
    x₀ = get(basics.pdset, :posGx, carind_start, validfind_start)
    y₀ = get(basics.pdset, :posGy, carind_start, validfind_start)

    basics.runid = rand(Int)
    ncollisions = 0
    for i = 1 : nsimulations
        
        basics.runid += 1
        simulate!(basics, behavior_pairs, validfind_start, validfind_end,
                    pdset_frames_per_sim_frame=pdset_frames_per_sim_frame,
                    n_euler_steps=n_euler_steps)

        ncollisions += has_intersection(basics.pdset, validfind_start, validfind_end)

        for (j,gridcount) in enumerate(gridcounts)
            validfind_final = validfind_start + j



            carind = carid2ind(basics.pdset, carid, validfind_final)
            x₁ = get(basics.pdset, :posGx, carind, validfind_final)
            y₁ = get(basics.pdset, :posGy, carind, validfind_final)

            Δs, Δd = frenet_distance_between_points(basics.sn, x₀, y₀, x₁, y₁)

            bin_s = encode(histobin_params.discx, Δs)
            bin_d = encode(histobin_params.discy, Δd)

            gridcount[bin_s, bin_d] += 1.0
        end
    end

    ncollisions / nsimulations
end
function calc_collision_risk_and_future_grid_counts!(
    gridcounts      :: Vector{Vector{Matrix{Float64}}},
    histobin_params :: ParamsHistobin,
    basics          :: FeatureExtractBasicsPdSet,
    behavior_pairs  :: Vector{(AbstractVehicleBehavior,Int)}, # (behavior, carid)
    validfind_start :: Int,
    validfind_end   :: Int;

    nsimulations    :: Integer=100,
    pdset_frames_per_sim_frame::Int=N_FRAMES_PER_SIM_FRAME,
    n_euler_steps   :: Int=2,
    start_points    :: Vector{VecE2}=calc_start_points(basics.pdset, behavior_pairs, validfind_start),

    carA::Vehicle = Vehicle(),
    carB::Vehicle = Vehicle(),
    cornersCarA::FourCorners=FourCorners(),
    cornersCarB::FourCorners=FourCorners(),
    )

    #=
    Compute the future gridcounts for the given carids in behavior_pairs
    =#

    n_cars_to_eval = length(behavior_pairs)
    for i in 1 : n_cars_to_eval
        for gridcount in gridcounts[i]
            fill!(gridcount, 0.0)
        end
    end
    
    basics.runid = rand(Int)
    ncollisions = 0
    for i = 1 : nsimulations
        
        basics.runid += 1
        simulate!(basics, behavior_pairs, validfind_start, validfind_end,
                  pdset_frames_per_sim_frame=pdset_frames_per_sim_frame,
                  n_euler_steps=n_euler_steps)

        ncollisions += has_intersection(basics.pdset, validfind_start, validfind_end,
                                        carA, carB, cornersCarA, cornersCarB)

        for k in 1 : n_cars_to_eval

            carid = behavior_pairs[k][2]
            x₀ = start_points[k].x
            y₀ = start_points[k].y

            for (j,gridcount) in enumerate(gridcounts[k])
                validfind_final = validfind_start + j

                carind = carid2ind(basics.pdset, carid, validfind_final)
                x₁ = get(basics.pdset, :posGx, carind, validfind_final)
                y₁ = get(basics.pdset, :posGy, carind, validfind_final)

                Δs, Δd = frenet_distance_between_points(basics.sn, x₀, y₀, x₁, y₁)

                bin_s = encode(histobin_params.discx, Δs)
                bin_d = encode(histobin_params.discy, Δd)

                gridcount[bin_s, bin_d] += 1.0
            end
        end
    end

    ncollisions / nsimulations
end

function allocate_gridcounts!(horizon::Integer, histobin_params::ParamsHistobin)

    nbinsx = nlabels(histobin_params.discx)
    nbinsy = nlabels(histobin_params.discy)

    gridcounts = Array(Matrix{Float64}, horizon) # only frames past start, noninclusive
    for i = 1 : length(gridcounts)
        gridcounts[i] = Array(Float64, nbinsx, nbinsy)
    end
    gridcounts
end