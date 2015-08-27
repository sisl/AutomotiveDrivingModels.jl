export
    PolynomialFactoredTrajectory,
    IntegratorState,

    get_quartic_coefficients,
    get_quintic_coefficients,

    # calc_candidate_trajectory_bezier_followlane,
    # set_trajectory_relative_to_lane!,
    # vehicletrace!,
    # vehicletrace

immutable PolynomialFactoredTrajectory
    # NOTE(tim): t=0 is the start of the trajectory in the polynomials
    #            t=τ is the end of the trajectory

    s::Polynomial # along the centerline
    d::Polynomial # perpendicular to the centerline, (left)
end
immutable IntegratorState
    x::Float64 # value
    v::Float64 # 1st derivative
    a::Float64 # 2nd derivative

    IntegratorState(x::Float64=0.0,v::Float64=0.0,a::Float64=0.0) = new(x,v,a)
end

function translate(traj::PolynomialFactoredTrajectory, Δs::Float64, Δd::Float64)

    PolynomialFactoredTrajectory(translate(traj.s, Δs), translate(traj.d, Δd))
end
function translate_so_is_at_loc_at_time(traj::PolynomialFactoredTrajectory, s::Float64, d::Float64, t::Float64)

    s_t = p₁(traj.s, t)
    d_t = p₁(traj.d, t)
    Δs = s - s_t
    Δd = d - d_t
    Δs = NaN # NOTE(tim): XXXX
    Δd = NaN
    translate(traj, Δs, Δd)
end

function _get_M1(t::Float64, t²::Float64=t*t)
    [1.0   t  t²;
     0.0 1.0 2t;
     0.0 0.0 2.0]
end
function _get_M2_quartic(t::Float64, t²::Float64=t*t, t³::Float64=t²*t, t⁴::Float64=t³*t, t⁵::Float64=t⁴*t)
    [ t³  t⁴   t⁵;
     3t² 4t³  5t⁴;
     6t 12t² 20t³]
end
function _get_M2_quintic(t::Float64, t²::Float64=t*t, t³::Float64=t²*t, t⁴::Float64=t³*t)
    [-1.0  t³  t⁴
      0.0 3t² 4t³;
      0.0 6t 12t²]
end
function _get_M1_and_M2_quartic(t::Float64)
    t² = t*t
    t³ = t²*t
    t⁴ = t³*t

    M₁ = _get_M1(t, t²)
    M₂ = _get_M2_quartic(t, t², t³, t⁴)
    (M₁, M₂)
end
function _get_M1_and_M2_quintic(t::Float64)
    t² = t*t
    t³ = t²*t
    t⁴ = t³*t

    M₁ = _get_M1(t, t²)
    M₂ = _get_M2_quintic(t, t², t³, t⁴)
    (M₁, M₂)
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

    ξ₀ =  [x1, v1, a1]
    M₁₀ = _get_M1(0.0)

    ξ₂ =  [x2, v2, a2]
    M₁₂, M₂₂ = _get_M1_and_M2_quartic(τ)

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

    ξ₀ =  [x1, v1, a1]
    M₁₀ = _get_M1(0.0)

    ξ₂ =  [0.0, v2, a2]
    M₁₂, M₂₂ = _get_M1_and_M2_quintic(τ)

    c₀₁₂ = M₁₀ \ ξ₀
    retval = M₂₂ \ (ξ₂ - M₁₂*c₀₁₂) # returns vector [ξ₁, c₃, c₄]

    Quintic(c₀₁₂[1], c₀₁₂[2], c₀₁₂[3], retval[2], retval[3])
end

# function calc_candidate_trajectory_bezier_followlane(
#     trace::VehicleTrace,
#     road::StraightRoadway,
#     frameind_start::Int,
#     horizon::Int,
#     lane_index::Int,
#     sdot₂::Float64,
#     sddot₂::Float64,
#     sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME;
#     d₂::Float64=0.0, # final lateral offset
#     ddot₂::Float64=0.0, # final lateral velocity
#     dddot₂::Float64=0.0, # final lateral acceleration
#     )

#     s₁ = trace.log[frameind_start, LOG_COL_X]
#     d₁ = trace.log[frameind_start, LOG_COL_Y] - get_lanecenter(road, lane_index)
#     v₁ = trace.log[frameind_start, LOG_COL_V]
#     ϕ₁ = trace.log[frameind_start, LOG_COL_ϕ]

#     sdot₁ = v₁ * cos(ϕ₁)
#     ddot₁ = v₁ * sin(ϕ₁)

#     if frameind_start > 1
#         v₀ = trace.log[frameind_start-1, LOG_COL_V]
#         ϕ₀ = trace.log[frameind_start-1, LOG_COL_ϕ]
#         sdot₀ = v₀ * cos(ϕ₀)
#         ddot₀ = v₀ * sin(ϕ₀)
#         sddot₁ = (sdot₁ - sdot₀) / sec_per_frame
#         dddot₁ = (ddot₁ - ddot₀) / sec_per_frame
#     else
#         sddot₁ = 0.0
#         dddot₁ = 0.0
#     end

#     τ = horizon * sec_per_frame

#     calc_candidate_trajectory_bezier_followlane(d₁, ddot₁, dddot₁, sdot₁, sddot₁, τ, 
#                                                 d₂=d₂, ddot₂=ddot₂, dddot₂=dddot₂,
#                                                 s₁=s₁, sdot₂=sdot₂, sddot₂=sddot₂)
# end
# function calc_candidate_trajectory_bezier_followlane(
#     simlog::Matrix{Float64},
#     road::StraightRoadway,
#     carind::Int,
#     frameind_start::Int,
#     horizon::Int,
#     lane_index::Int,
#     sdot₂::Float64,
#     sddot₂::Float64,
#     sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME;
#     d₂::Float64=0.0, # final lateral offset
#     ddot₂::Float64=0.0, # final lateral velocity
#     dddot₂::Float64=0.0, # final lateral acceleration
#     )

#     baseind = calc_logindexbase(carind)
#     s₁ = simlog[frameind_start, baseind+LOG_COL_X]
#     d₁ = simlog[frameind_start, baseind+LOG_COL_Y] - get_lanecenter(road, lane_index)
#     v₁ = simlog[frameind_start, baseind+LOG_COL_V]
#     ϕ₁ = simlog[frameind_start, baseind+LOG_COL_ϕ]

#     sdot₁ = v₁ * cos(ϕ₁)
#     ddot₁ = v₁ * sin(ϕ₁)

#     if frameind_start > 1
#         v₀ = simlog[frameind_start-1, baseind+LOG_COL_V]
#         ϕ₀ = simlog[frameind_start-1, baseind+LOG_COL_V]
#         sdot₀ = v₀ * cos(ϕ₀)
#         ddot₀ = v₀ * sin(ϕ₀)
#         sddot₁ = (sdot₁ - sdot₀) / sec_per_frame
#         dddot₁ = (ddot₁ - ddot₀) / sec_per_frame
#     else
#         sddot₁ = 0.0
#         dddot₁ = 0.0
#     end

#     τ = horizon * sec_per_frame

#     calc_candidate_trajectory_bezier_followlane(d₁, ddot₁, dddot₁, sdot₁, sddot₁, τ, 
#                                                 d₂=d₂, ddot₂=ddot₂, dddot₂=dddot₂,
#                                                 s₁=s₁, sdot₂=sdot₂, sddot₂=sddot₂)
# end
# function calc_candidate_trajectory_bezier_followlane(
#     ξd₁::IntegratorState, # initial lateral state
#     ξs₁::IntegratorState, # initial longitudinal state
#     ξd₂::IntegratorState, # final lateral state
#     ξs₂::IntegratorState, # final longitudinal state (position ignored)
#     τ::Float64; # trajectory duration [sec]
#     )

#    calc_candidate_trajectory_bezier_followlane(
#         ξd₁.x, ξd₁.v, ξd₁.a, ξs₁.v, ξs₁.a, τ,
#         d₂=ξd₂.x, ddot₂=ξd₂.v, dddot₂=ξd₂.a, sdot₂=ξs₂.v, sddot₂=ξs₂.a, s₁=ξs₁.x
#    )
# end
# function calc_candidate_trajectory_bezier_followlane(
#     d₁::Float64, # initial lateral offset
#     ddot₁::Float64, # initial lateral velocity
#     dddot₁::Float64, # initial lateral acceleration
#     sdot₁::Float64, # initial longitudinal speed
#     sddot₁::Float64, # initial longitudinal acceleration
#     τ::Float64; # trajectory duration [sec]

#     d₂::Float64=0.0, # final lateral offset
#     ddot₂::Float64=0.0, # final lateral velocity
#     dddot₂::Float64=0.0, # final lateral acceleration
#     sdot₂::Float64=sdot₁, # final longitudinal velocity
#     sddot₂::Float64=0.0, # final longitudinal accel
#     s₁::Float64=0.0, # initial longitudinal position
#     )

#     # Compute a trajectory in which the lateral start and end states are fully known
#     # and where the longitudinal trajectory has unspecified end position

#     s = get_quintic_coefficients(s₁, sdot₁, sddot₁, sdot₂, sddot₂, τ)
#     d = get_quartic_coefficients(d₁, ddot₁, dddot₁, d₂, ddot₂, dddot₂, τ)

#     PolynomialFactoredTrajectory(s, d)
# end

# function _set_frame!(
#     trace::VehicleTrace,
#     frameind::Int,
#     s::Float64,
#     sdot::Float64,
#     sddot::Float64,
#     d::Float64,
#     ddot::Float64,
#     dddot::Float64
#     )
    
#     v = hypot(ddot, sdot)
#     a = hypot(dddot, sddot)
#     ϕ = atan2(ddot, sdot)
#     ω = atan2(dddot, sddot)

#     trace.log[frameind, LOG_COL_X] = s
#     trace.log[frameind, LOG_COL_Y] = d
#     trace.log[frameind, LOG_COL_ϕ] = ϕ
#     trace.log[frameind, LOG_COL_V] = v
#     trace.log[frameind, LOG_COL_A] = a
#     trace.log[frameind, LOG_COL_T] = ω
#     trace.log[frameind, LOG_COL_BIN_LAT] = NaN
#     trace.log[frameind, LOG_COL_BIN_LON] = NaN
#     trace.log[frameind, LOG_COL_LAT_BEFORE_SMOOTHING] = NaN
#     trace.log[frameind, LOG_COL_LON_BEFORE_SMOOTHING] = NaN
#     trace.log[frameind, LOG_COL_logprobweight_A] = NaN
#     trace.log[frameind, LOG_COL_logprobweight_T] = NaN

#     trace
# end
# function _set_frame!(
#     trace::VehicleTrace,
#     frameind::Int,
#     ξs::IntegratorState,
#     ξd::IntegratorState
#     )
    
#     _set_frame!(trace, frameind, ξs.x, ξs.v, ξs.a, ξd.x, ξd.v, ξd.a)
# end
# function _set_frame!(
#     trace::VehicleTrace,
#     frameind::Int,
#     traj::PolynomialFactoredTrajectory,
#     t::Float64,
#     closest_centerline_y::Float64=0.0
#     )
    
#     s, sdot, sddot = p₁₂₃(traj.s, t)
#     d, ddot, dddot = p₁₂₃(traj.d, t)

#     _set_frame!(trace, frameind, s, sdot, sddot, d, ddot, dddot)
# end

# function set_trajectory_relative_to_lane!(
#     simlog::Matrix{Float64},
#     road::StraightRoadway,
#     carind::Int,
#     frameind_start::Int,
#     horizon::Int,
#     lane_index::Int,
#     traj::PolynomialFactoredTrajectory,
#     sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
#     )

#     # NOTE: this does NOT set the initial position

#     closest_centerline_y = get_lanecenter(road, lane_index)

#     baseind = calc_logindexbase(carind)

#     t = 0.0
#     frameind = frameind_start
#     for framejump in 1 : horizon
#         frameind += 1
#         t += sec_per_frame

#         s, sdot = p₁₂(traj.s, t)
#         d, ddot = p₁₂(traj.d, t)
#         v = hypot(ddot, sdot)
#         ϕ = atan2(ddot, sdot)

#         simlog[frameind, baseind + LOG_COL_X] = s
#         simlog[frameind, baseind + LOG_COL_Y] = d + closest_centerline_y
#         simlog[frameind, baseind + LOG_COL_ϕ] = ϕ
#         simlog[frameind, baseind + LOG_COL_V] = v
#     end

#     simlog
# end
# function vehicletrace!(
#     trace::VehicleTrace, # initialized with sufficiently large simlog and set trace_history
#     frameind_start::Int, # index in trace.log corresponding to t=0 in the polynomial is [this frame WILL be overwritten]
#     horizon::Int, # how many frames to go forward [ie, horizon=2 means 3 frames will be written]
#     road::StraightRoadway,
#     lane_index::Int,
#     traj::PolynomialFactoredTrajectory,
#     sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
#     )
    
#     closest_centerline_y = get_lanecenter(road, lane_index)

#     t = -sec_per_frame
#     frameind = frameind_start-1
#     for framejump in 0 : horizon
#         frameind += 1
#         t += sec_per_frame
#         _set_frame!(trace, frameind, traj, t, closest_centerline_y)
#     end

#     trace
# end
# function vehicletrace(
#     trace_history::Int, # what to set the trace history to
#     frameind_start::Int, # where t=0 in the polynomial is [this frame WILL be overwritten]
#     horizon::Int, # how many frames to go forward [ie, horizon=2 means 3 frames will be written]
#     road::StraightRoadway,
#     lane_index::Int,
#     traj::PolynomialFactoredTrajectory,
#     sec_per_frame::Float64=DEFAULT_SEC_PER_FRAME
#     )

#     @assert(trace_history - frameind_start ≥ 0)
#     nframes = trace_history - frameind_start + horizon + 1
#     trace = VehicleTrace(nframes, trace_history)
#     vehicletrace!(trace, frameind_start, horizon, road, lane_index, traj, sec_per_frame)
# end
# function vehicletrace(
#     trace_history::Int,
#     states_lat::Vector{IntegratorState}, # n points
#     states_lon::Vector{IntegratorState}, # n points
#     durations::Vector{Int}, # n-1 segment durations [frames]
#     lane_indeces::Vector{Int},
#     road::StraightRoadway,
#     sec_per_frame::Float64
#     )

#     #=
#     (ξ_lat,ξ_lon) → (ξ_lat,ξ_lon) → (ξ_lat,ξ_lon) ...
#     with a duration for each segment given in terms of frames
#     and a Δt / frame declared
#     =#

#     n = length(states_lat)
#     @assert(length(states_lon) == n)
#     @assert(length(durations) == n-1)
#     @assert(length(lane_indeces) == n-1)

#     nframes_total = sum(durations)+1
#     trace = VehicleTrace(nframes_total, trace_history)

#     t = NaN
#     frameind = 1
#     ξ_lat₂, ξ_lon₂ = states_lat[1], states_lon[1]
#     _set_frame!(trace, frameind, ξ_lat₂, ξ_lon₂)

#     for i = 2 : n
#         ξ_lat₁, ξ_lon₁ = ξ_lat₂, ξ_lon₂
#         ξ_lat₂, ξ_lon₂ = states_lat[i], states_lon[i]
#         duration = durations[i-1]
#         lane_index = lane_indeces[i-1]

#         closest_centerline_y = get_lanecenter(road, lane_index)


#         τ = duration * sec_per_frame
#         traj = calc_candidate_trajectory_bezier_followlane(
#                        ξ_lat₁, ξ_lon₁, ξ_lat₂, ξ_lon₂, τ)

#         t = 0.0
#         for duration_index in 1 : duration
#             frameind += 1
#             t += sec_per_frame
#             _set_frame!(trace, frameind, traj, t, closest_centerline_y)
#         end

#         ξ_lat₂ = IntegratorState(p₁₂₃(traj.d, t)...)
#         ξ_lon₂ = IntegratorState(p₁₂₃(traj.s, t)...)
#     end

#     trace
# end



