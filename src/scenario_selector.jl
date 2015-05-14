export ScenarioSelector
export select_encounter_model, get_scenario_symbol
export EM_ID_UNKNOWN, EM_ID_FREEFLOW, EM_ID_CARFOLLOW, EM_ID_LANECHANGE

import Base: get

const EM_ID_UNKNOWN    = 0
const EM_ID_FREEFLOW   = 1
const EM_ID_CARFOLLOW  = 2
const EM_ID_LANECHANGE = 3

immutable ScenarioSelector
    freeflow   :: EM
    carfollow  :: EM
    lanechange :: EM
end

function select_encounter_model(
    scenarioselector :: ScenarioSelector,
    log              :: Matrix{Float64},
    road             :: StraightRoadway,
    carind           :: Int,
    frameind         :: Int,
    sec_per_frame    :: Float64
    )

    # TODO(tim): allow for lanechange

    vFy = get(VELFY, log, road, sec_per_frame, carind, frameind)::Float64
    if abs(vFy) > 0.15
        return EM_ID_CARFOLLOW
    end

    has_front = bool(get(HAS_FRONT, log, road, sec_per_frame, carind, frameind)::Float64)
    if !has_front
        return EM_ID_FREEFLOW
    end

    # ΔT = get(TIMEGAP_X_FRONT, log, road, sec_per_frame, carind, frameind)::Float64
    # if ΔT > 3.0
    #     return EM_ID_FREEFLOW
    # end

    # ΔV = get(V_X_FRONT, log, road, sec_per_frame, carind, frameind)::Float64
    # if ΔV > 0.5
    #     return EM_ID_FREEFLOW
    # end

    return EM_ID_CARFOLLOW
end
function get(ss::ScenarioSelector, id::Int)
    if id == EM_ID_FREEFLOW
        ss.freeflow
    elseif id == EM_ID_CARFOLLOW
        ss.carfollow
    elseif id == EM_ID_LANECHANGE
        ss.lanechange
    else
        warn("unknown id, returning freeflow model")
        ss.freeflow
    end
end
function get_scenario_symbol(ss::ScenarioSelector, em::EM)
    if em === ss.freeflow
        return :freeflow
    elseif em === ss.carfollow
        return :carfollow
    elseif em === ss.lanechange
        return :lanechange
    end
    warn("unknown em")
    return :unknown
end