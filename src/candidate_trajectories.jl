

function calc_candidate_trajectory_bezier_followlane!(
    simlog::Matrix{Float64},
    road::StraightRoadway,
    carind::Int,
    frameind_start::Int,
    horizon::Int,
    lane_index::Int,
    speed_end::Float64,
    accel_end::Float64,
    params::SimParams = SimParams()
    )

    baseind = calc_logindexbase(carind)

    closest_centerline = get_lanecenter(road, lane_index)

    s_start = simlog[frameind_start, baseind+LOG_COL_X]
    d_start = simlog[frameind_start, baseind+LOG_COL_Y] - closest_centerline
    v_start = simlog[frameind_start, baseind+LOG_COL_V]
    ϕ_start = simlog[frameind_start, baseind+LOG_COL_ϕ]

    sdot = v_start * cos(ϕ_start)
    ddot = v_start * sin(ϕ_start)

    sddot = 0.0 # TODO(tim): fix this
    dddot = 0.0 # TODO(tim): fix this
    d_end, ddot_end, dddot_end = 0.0, 0.0, 0.0

    τ = horizon * params.sec_per_frame

    q_lat = get_quartic_coefficients(d_start, ddot, dddot, d_end, ddot_end, dddot_end, τ)
    q_lon = get_quintic_coefficients(s_start, sdot, sddot, speed_end, accel_end, τ)

    for framejump in 1 : horizon
        frameind = frameind_start + framejump
        t = framejump * params.sec_per_frame

        s = p₁(q_lon, t::Float64)
        d = p₁(q_lat, t::Float64)
        v = hypot(p₂(q_lon, t::Float64), p₂(q_lat, t::Float64))
        ϕ = 0.0 # TODO(tim): fix this

        simlog[frameind, baseind + LOG_COL_X] = s
        simlog[frameind, baseind + LOG_COL_Y] = d + closest_centerline
        simlog[frameind, baseind + LOG_COL_ϕ] = ϕ
        simlog[frameind, baseind + LOG_COL_V] = v
    end

    simlog
end

abstract Polynomial
immutable Quintic <: Polynomial
    x₁::Float64
    x₂::Float64
    x₃::Float64
    x₄::Float64
    x₅::Float64
end
immutable Quartic <: Polynomial
    x₁::Float64
    x₂::Float64
    x₃::Float64
    x₄::Float64
    x₅::Float64
    x₆::Float64
end

p₁(q::Quintic, t::Float64) = q.x₁ + t*(q.x₂ + t*(q.x₃ + t*(q.x₄ + t*q.x₅))) # eval poly
p₂(q::Quintic, t::Float64) = q.x₂ + t*(2q.x₃ + t*(3q.x₄ + t*4q.x₅)) # first derivative
p₃(q::Quintic, t::Float64) = 2q.x₃ + t*(6q.x₄ + t*12q.x₅) # second derivative
p₄(q::Quintic, t::Float64) = 6q.x₄ + t*24q.x₅ # third derivative

p₁(q::Quartic, t::Float64) = q.x₁ + t*(q.x₂ + t*(q.x₃ + t*(q.x₄ + t*(q.x₅ + t*q.x₆)))) # eval poly
p₂(q::Quartic, t::Float64) = q.x₂ + t*(2q.x₃ + t*(3q.x₄ + t*(4q.x₅ + t*5q.x₆))) # first derivative
p₃(q::Quartic, t::Float64) = 2q.x₃ + t*(6q.x₄ + t*(12q.x₅ + t*20q.x₆)) # second derivative
p₄(q::Quartic, t::Float64) = 6q.x₄ + t*(24q.x₅ + t*60q.x₆) # third derivative

function _get_M1(t::Float64)
    [1.0 t t*t;
     0.0 1.0 2t;
     0.0 0.0 2.0]
end
function _get_M2(t::Float64)
    t2 = t*t
    t3 = t2*t
    t4 = t3*t

    [t3 t4 t4*t;
     3t2 4t3 5t4;
     6t 12t2 20t3]
end
function _get_M2_quintic(t::Float64)
    t2 = t*t
    t3 = t2*t

    [-1.0 t3 t3*t
      0.0 3t2 4t3;
      0.0 6t 12t2]
end

function get_quartic_coefficients(
    x1::Float64, # starting state
    v1::Float64, # starting 1st derivative
    a1::Float64, # starting 2nd derivative
    x2::Float64, # end state
    v2::Float64, # end 1st derivative
    a2::Float64, # end 2nd derivative
    τ::Float64, # end time
    )

    @assert(τ > 0.0)

    M₁₀ = _get_M1(0.0)
    # M₂₀ = _get_M2(0.0)
    ξ₀ =  [x1, v1, a1]

    M₁₂ = _get_M1(τ)
    M₂₂ = _get_M2(τ)
    ξ₂ =  [x2, v2, a2]

    c₀₁₂ = M₁₀ \ ξ₀
    c₃₄₅ = M₂₂ \ (ξ₂ - M₁₂*c₀₁₂)

    Quartic(c₀₁₂[1], c₀₁₂[2], c₀₁₂[3], c₃₄₅[1], c₃₄₅[2], c₃₄₅[3])
end
function get_quintic_coefficients(
    x1::Float64, # starting state
    v1::Float64, # starting 1st derivative
    a1::Float64, # starting 2nd derivative
    v2::Float64, # end 1st derivative
    a2::Float64, # end 2nd derivative
    τ::Float64, # end time
    )

    @assert(τ > 0.0)

    M₁₀ = _get_M1(0.0)
    # M₂₀ = _get_M2(0.0)
    ξ₀ =  [x1, v1, a1]

    M₁₂ = _get_M1(τ)
    M₂₂ = _get_M2_quintic(τ)
    ξ₂ =  [0.0, v2, a2]

    c₀₁₂ = M₁₀ \ ξ₀
    retval = M₂₂ \ (ξ₂ - M₁₂*c₀₁₂) # returns vector [ξ₁, c₃, c₄]

    Quintic(c₀₁₂[1], c₀₁₂[2], c₀₁₂[3], retval[2], retval[3])
end

start_state_lat = [3.0, 0.0, 0.0]
end_state_lat = [1.0, -0.5, 0.0]
start_state_lon = [0.0, 1.0, 0.0]
end_state_lon = [NaN, 1.0, 0.0]
τ = 2.5

q_lat = get_quartic_coefficients(start_state_lat..., end_state_lat..., τ)
q_lon = get_quintic_coefficients(start_state_lon..., end_state_lon[2:3]..., τ)


println("start: ", start_state)
println("end: ", end_state)
println("τ: ", τ)
println(q)

println("p₁: ", p₁(q, 0.0))
println("p₂: ", p₂(q, 0.0))
println("p₃: ", p₃(q, 0.0))

println("p₁: ", p₁(q, 1.0))
println("p₂: ", p₂(q, 1.0))
println("p₃: ", p₃(q, 1.0))