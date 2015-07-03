# export
#     PolynomialFactoredTrajectory,

#     Polynomial,
#     Quartic,
#     Quintic,

#     set_trajectory_relative_to_lane!,
#     calc_candidate_trajectory_bezier_followlane,

#     get_quartic_coefficients,
#     get_quintic_coefficients,
#     p₁, p₂, p₃, p₄, p₁₂, p₁₂₃

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

p₁₂(p::Polynomial, t::Float64) = (p₁(p,t), p₂(p,t))
p₁₂₃(p::Polynomial, t::Float64) = (p₁(p,t), p₂(p,t), p₃(p,t))

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

# =========================

type PolynomialFactoredTrajectory
    # NOTE(tim): t=0 is the start of the trajectory,
    #            t=τ is the end of the trajectory
    s::Polynomial # along the centerline
    d::Polynomial # perpendicular to the centerline, (left)
end

function set_trajectory_relative_to_lane!(
    simlog::Matrix{Float64},
    road::StraightRoadway,
    carind::Int,
    frameind_start::Int,
    horizon::Int,
    lane_index::Int,
    traj::PolynomialFactoredTrajectory,
    sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
    )

    # NOTE: this does NOT set the initial position

    closest_centerline_y = get_lanecenter(road, lane_index)

    baseind = calc_logindexbase(carind)

    t = 0.0
    frameind = frameind_start
    for framejump in 1 : horizon
        frameind += 1
        t += sec_per_frame

        s, sdot = p₁₂(traj.s, t)
        d, ddot = p₁₂(traj.d, t)
        v = hypot(ddot, sdot)
        ϕ = atan2(ddot, sdot)

        simlog[frameind, baseind + LOG_COL_X] = s
        simlog[frameind, baseind + LOG_COL_Y] = d + closest_centerline_y
        simlog[frameind, baseind + LOG_COL_ϕ] = ϕ
        simlog[frameind, baseind + LOG_COL_V] = v
    end

    simlog
end

function calc_candidate_trajectory_bezier_followlane(
    simlog::Matrix{Float64},
    road::StraightRoadway,
    carind::Int,
    frameind_start::Int,
    horizon::Int,
    lane_index::Int,
    sdot₂::Float64,
    sddot₂::Float64,
    sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME;
    d₂::Float64=0.0, # final lateral offset
    ddot₂::Float64=0.0, # final lateral velocity
    dddot₂::Float64=0.0, # final lateral acceleration
    )

    baseind = calc_logindexbase(carind)
    s₁ = simlog[frameind_start, baseind+LOG_COL_X]
    d₁ = simlog[frameind_start, baseind+LOG_COL_Y] - get_lanecenter(road, lane_index)
    v₁ = simlog[frameind_start, baseind+LOG_COL_V]
    ϕ₁ = simlog[frameind_start, baseind+LOG_COL_ϕ]

    sdot₁ = v₁ * cos(ϕ₁)
    ddot₁ = v₁ * sin(ϕ₁)

    if frameind_start > 1
        v₀ = simlog[frameind_start-1, baseind+LOG_COL_V]
        ϕ₀ = simlog[frameind_start-1, baseind+LOG_COL_V]
        sdot₀ = v₀ * cos(ϕ₀)
        ddot₀ = v₀ * sin(ϕ₀)
        sddot₁ = (sdot₁ - sdot₀) / sec_per_frame
        dddot₁ = (ddot₁ - ddot₀) / sec_per_frame
    else
        sddot₁ = 0.0
        dddot₁ = 0.0
    end

    τ = horizon * sec_per_frame

    calc_candidate_trajectory_bezier_followlane(d₁, ddot₁, dddot₁, sdot₁, sddot₁, τ, 
                                                d₂=d₂, ddot₂=ddot₂, dddot₂=dddot₂,
                                                s₁=s₁, sdot₂=sdot₂, sddot₂=sddot₂)
end
function calc_candidate_trajectory_bezier_followlane(
    d₁::Float64, # initial lateral offset
    ddot₁::Float64, # initial lateral velocity
    dddot₁::Float64, # initial lateral acceleration
    sdot₁::Float64, # initial longitudinal speed
    sddot₁::Float64, # initial longitudinal acceleration
    τ::Float64; # trajectory duration [sec]

    d₂::Float64=0.0, # final lateral offset
    ddot₂::Float64=0.0, # final lateral velocity
    dddot₂::Float64=0.0, # final lateral acceleration
    sdot₂::Float64=sdot₁, # final longitudinal velocity
    sddot₂::Float64=0.0, # final longitudinal accel
    s₁::Float64=0.0, # initial longitudinal position
    )

    # Compute a trajectory in which the lateral start and end states are fully known
    # and where the longitudinal trajectory has unspecified end position

    s = get_quintic_coefficients(s₁, sdot₁, sddot₁, sdot₂, sddot₂, τ)
    d = get_quartic_coefficients(d₁, ddot₁, dddot₁, d₂, ddot₂, dddot₂, τ)

    PolynomialFactoredTrajectory(s, d)
end