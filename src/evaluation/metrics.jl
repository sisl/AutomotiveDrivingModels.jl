export
    MetricExtractor,
    FrameMetricExtractor,
    TraceMetricExtractor,

    get_symbol,
    get_score,
    reset!,
    extract!,

    RootWeightedSquareError

abstract MetricExtractor
abstract FrameMetricExtractor <: MetricExtractor
abstract TraceMetricExtractor <: MetricExtractor

########################################
#        RootWeightedSquareError       #
########################################

type RootWeightedSquareError{F<:AbstractFeature} <: TraceMetricExtractor
    f::F
    horizon::Float64 # [s]
    running_sum::Float64
    n_obs::Int
end
RootWeightedSquareError{F<:AbstractFeature}(f::F, horizon::Float64) = RootWeightedSquareError(f, horizon, 0.0, 0)

get_symbol(m::RootWeightedSquareError) = symbol(@sprintf("RWSE_%s_%d_%02d", string(symbol(m.f)), floor(Int, m.horizon), floor(Int, 100*rem(m.horizon, 1.0))))
get_score(m::RootWeightedSquareError) = sqrt(m.running_sum / m.n_obs)
function reset!(metric::RootWeightedSquareError)
    metric.running_sum = 0.0
    metric.n_obs = 0
    metric
end
function extract!(
    metric::RootWeightedSquareError,
    rec_orig::SceneRecord, # the records are exactly as long as the simulation (ie, contain everything)
    rec_sim::SceneRecord,
    roadway::Roadway,
    egoid::Int,
    )

    # TODO: how to handle missing values???

    pastframe = 1-length(rec) + clamp(round(Int, metric.horizon/rec_orig.timestep), 0, length(rec_orig)-1)

    # pull true value
    vehicle_index = get_index_of_first_vehicle_with_id(rec_orig, egoid, pastframe)
    v_true = convert(Float64, get(metric.f, rec_orig, roadway, vehicle_index, pastframe))

    # pull sim value
    vehicle_index = get_index_of_first_vehicle_with_id(rec_sim, egoid, pastframe)
    v_montecarlo = convert(Float64, get(metric.F, rec_sim, roadway, vehicle_index, pastframe))

    Δ = v_true - v_montecarlo
    metric.running_sum += Δ*Δ
    metric.n_obs += 1

    metric
end