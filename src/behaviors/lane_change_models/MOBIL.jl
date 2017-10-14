"""
    MOBIL
See Treiber & Kesting, 'Modeling Lane-Changing Decisions with MOBIL'
"""
@with_kw mutable struct MOBIL <: LaneChangeModel

    dir::Int = DIR_MIDDLE
    mlon::LaneFollowingDriver = IntelligentDriverModel()
    safe_decel::Float64 = 2.0 # safe deceleration (positive value) [m/s²]
    politeness::Float64 = 0.35 # politeness factor (suggested p ∈ [0.2,0.5])
    advantage_threshold::Float64 = 0.1# Δaₜₕ
end
get_name(::MOBIL) = "MOBIL"
function set_desired_speed!(model::MOBIL, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    model
end
Base.rand(model::MOBIL) = LaneChangeChoice(model.dir)