"""
	uncertain_IDM <: LaneFollowingDriver

The Intelligent Driver Model. A rule based driving model that is governed by parameter
settings. The output is an longitudinal acceleration.

Here, we have extended IDM to incorporate two sources of uncertain behavior. If a standard deviation parameter is
specified, then the output is a longitudinal acceleration sampled from a normal distribution
around the non-errorable IDM output. Also, if sensor noise standar deviation parameter is provided, then the headway
measurement suffers by 0 mean Gaussian noise corruption.

# Fields
- `a::Float64 = NaN` the predicted acceleration i.e. the output of the model
- `σ::Float64 = NaN` allows errorable IDM, optional stdev on top of the model, set to zero or NaN for deterministic behavior
- `sigma_sensor::Float64 = NaN` allows sensor noise corruption in the measurement of distance to leader
- `k_spd::Float64 = 1.0` proportional constant for speed tracking when in freeflow [s⁻¹]
- `δ::Float64 = 4.0` acceleration exponent
- `T::Float64  = 1.5` desired time headway [s]
- `v_des::Float64 = 29.0` desired speed [m/s]
- `s_min::Float64 = 5.0` minimum acceptable gap [m]
- `a_max::Float64 = 3.0` maximum acceleration ability [m/s²]
- `d_cmf::Float64 = 2.0` comfortable deceleration [m/s²] (positive)
- `d_max::Float64 = 9.0` maximum deceleration [m/s²] (positive)
"""
@with_kw mutable struct uncertain_IDM <: LaneFollowingDriver
    a::Float64 = NaN # predicted acceleration
    σ::Float64 = NaN # optional stdev on top of the model, set to zero or NaN for deterministic behavior
    sigma_sensor::Float64 = NaN # Optional sensor noise for headway measurement corruption
    k_spd::Float64 = 1.0 # proportional constant for speed tracking when in freeflow [s⁻¹]

    δ::Float64 = 4.0 # acceleration exponent [-]
    T::Float64  = 1.5 # desired time headway [s]
    v_des::Float64 = 29.0 # desired speed [m/s]
    s_min::Float64 = 5.0 # minimum acceptable gap [m]
    a_max::Float64 = 3.0 # maximum acceleration ability [m/s²]
    d_cmf::Float64 = 2.0 # comfortable deceleration [m/s²] (positive)
    d_max::Float64 = 9.0 # maximum deceleration [m/s²] (positive)
end
get_name(::uncertain_IDM) = "uncertain_IDM"
function set_desired_speed!(model::uncertain_IDM, v_des::Float64)
    model.v_des = v_des
    model
end

function observe!(model::uncertain_IDM, scene::Scene, roadway::Roadway, egoid::Int)
# print("uncertain_IDM calls the observe! method \n")
    vehicle_index = findfirst(egoid, scene)

    fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())

    v_ego = scene[vehicle_index].state.v
    v_oth = NaN
    headway = NaN

    if fore.ind != nothing
        v_oth = scene[fore.ind].state.v
        headway = fore.Δs
    end

    # Add sensor noise to corrupt headway measurement
    if !(isnan(model.sigma_sensor))
	#print("Adding sensor noise to headway measurement \n")
	headway = rand(Normal(headway,model.sigma_sensor))
    end
    track_longitudinal!(model, v_ego, v_oth, headway)

    return model
end


function track_longitudinal!(model::uncertain_IDM, v_ego::Float64, v_oth::Float64, headway::Float64)
#print("uncertain_IDM says: track_longitudinal! called \n")
    if !isnan(v_oth)
        @assert !isnan(headway)
        if headway < 0.0

            @debug("IntelligentDriverModel Warning: IDM received a negative headway $headway"*
                  ", a collision may have occured.")
            model.a = -model.d_max
        else

            Δv = v_oth - v_ego

            s_des = model.s_min + v_ego*model.T - v_ego*Δv / (2*sqrt(model.a_max*model.d_cmf))

            v_ratio = model.v_des > 0.0 ? (v_ego/model.v_des) : 1.0

            model.a = model.a_max * (1.0 - v_ratio^model.δ - (s_des/headway)^2)

        end
    else
        Δv = model.v_des - v_ego
        model.a = Δv*model.k_spd # predicted accel to match target speed
#print("IDM says: No vehicle in front. Accelerate to heart's content, a = $(model.a)\n")
    end

    @assert !isnan(model.a)

    model.a = clamp(model.a, -model.d_max, model.a_max)

    return model
end

reset_hidden_state!(model::uncertain_IDM) = model

function Base.rand(model::uncertain_IDM)
    if isnan(model.σ) || model.σ ≤ 0.0
        LaneFollowingAccel(model.a)
    else
        LaneFollowingAccel(rand(Normal(model.a, model.σ)))
    end
end
function Distributions.pdf(model::uncertain_IDM, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a.a)
    end
end
function Distributions.logpdf(model::uncertain_IDM, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a.a)
    end
end
