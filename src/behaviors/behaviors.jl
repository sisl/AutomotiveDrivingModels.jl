#= #################################################
                 VEHICLE BEHAVIOR
=# #################################################

# define behavior for a given vehicle in the scene

export  AbstractVehicleBehavior,

        select_action,
        calc_action_loglikelihood,

        init_log!,
        init_logs!,
        train,

        VehicleBehaviorNone,
        VEHICLE_BEHAVIOR_NONE
        

abstract AbstractVehicleBehavior

function calc_action_loglikelihood(
    basics::FeatureExtractBasicsPdSet,
    behavior::AbstractVehicleBehavior,
    carind::Int,
    validfind::Int
    )

    action_lat = Features._get(FUTUREDESIREDANGLE_250MS, basics, carind, validfind)::Float64
    action_lon = Features._get(FUTUREACCELERATION_250MS, basics, carind, validfind)::Float64

    calc_action_loglikelihood(basics, behavior, carind, validfind, action_lat, action_lon)
end

train{B<:AbstractVehicleBehavior}(::Type{B}, trainingframes::DataFrame; args...) = error("train not implemented for $B")

###############################################################

function init_log!{B<:AbstractVehicleBehavior}(
    simlog     :: Matrix{Float64},
    behaviors  :: Vector{B},
    traces     :: Vector{VehicleTrace},
    startframe :: Int
    )
    
    num_cars = get_ncars(simlog)
    @assert(num_cars == length(behaviors))
    @assert(num_cars == length(traces))

    for carind = 1 : num_cars
        behavior = behaviors[carind]
        trace = traces[carind]
        init_log!(simlog, carind, behavior, trace, startframe)
    end

    simlog
end
init_log!(simlog::Matrix{Float64}, carind::Int, ::AbstractVehicleBehavior, trace::VehicleTrace, startframe::Int) =
    fill_log_with_trace_partial!(simlog, trace, carind, startframe)

function init_logs!{B<:AbstractVehicleBehavior}(
    simlogs::Vector{Matrix{Float64}},
    tracesets::Vector{Vector{VehicleTrace}},
    behaviors::Vector{B},
    history::Int
    )
    
    @assert(length(simlogs) == length(tracesets))
    for (simlog, traces) in zip(simlogs, tracesets)
        init_log!(simlog, behaviors[1:get_ncars(simlog)], traces, history)
    end
    simlogs
end

###############################################################

# The vehicle's actions are pre-determined
# No update is performed on the simlog for this vehicle
type VehicleBehaviorNone <: AbstractVehicleBehavior end
VEHICLE_BEHAVIOR_NONE = VehicleBehaviorNone()

init_log!(simlog::Matrix{Float64}, carind::Int, ::VehicleBehaviorNone, trace::VehicleTrace, startframe::Int) =
    fill_log_with_trace_complete!(simlog, trace, carind, startframe)