export
    Scenario

type Scenario
    name::AbstractString
    sn::StreetNetwork
    history::Int
    sec_per_frame::Float64
    trajdefs::Vector{TrajDef} # first is assumed to be ego
end

function CommonTypes.get_horizon(scenario::Scenario)
    horizon = -1
    for trajdef in scenario.trajdefs
        trajdef_horizon = get_num_pdset_frames(trajdef)-scenario.history
        if trajdef_horizon > horizon
            horizon = trajdef_horizon
        end
    end
    horizon
end

function AutomotiveDrivingModels.create_scenario_pdset(scenario::Scenario)
    create_scenario_pdset(
        scenario.sn,
        scenario.history,
        get_horizon(scenario),
        scenario.trajdefs,
        scenario.sec_per_frame
        )
end
