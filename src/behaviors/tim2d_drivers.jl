export Tim2DDriver

type Tim2DDriver <: DriverModel{LatLonAccel, IntegratedContinuous}

    rec::SceneRecord
    action_context::IntegratedContinuous

    v_des::Float64 # desired speed
    σ_lat::Float64 # constant standard deviation (optional, set to NaN or 0 if not needed)
    σ_lon::Float64 # constant standard deviation (optional, set to NaN or 0 if not needed)
    a_lat::Float64
    a_lon::Float64

    state::Symbol # unknown, freeflow, following, lanechange_left, lanechange_right

    # tuneable parameters
    k_lon::Float64
    k_lat_p::Float64
    k_lat_d::Float64

    threshold_lane_change_gap_fore::Float64
    threshold_lane_change_gap_rear::Float64
    k_follow::Float64 # controls how close we are willing to get to other cars

    function Tim2DDriver(
        action_context::IntegratedContinuous;

        rec::SceneRecord = SceneRecord(2, action_context.Δt),

        σ_lat::Float64 = NaN,
        σ_lon::Float64 = NaN,
        v_des::Float64 = 29.0,

        state::Symbol = :unknown,

        k_lon::Float64 = 1.0,
        k_lat_p::Float64 = 2.0,
        k_lat_d::Float64 = 1.0,

        threshold_lane_change_gap_fore::Float64 = 5.0,
        threshold_lane_change_gap_rear::Float64 = 5.0,
        k_follow::Float64 = 0.5
        )

        retval = new()

        retval.action_context=action_context
        retval.rec = rec

        retval.v_des = v_des
        retval.σ_lat = σ_lat
        retval.σ_lon = σ_lon
        retval.a_lat = NaN
        retval.a_lon = NaN

        retval.state = state

        retval.k_lon = k_lon
        retval.k_lat_p = k_lat_p
        retval.k_lat_d = k_lat_d

        retval.threshold_lane_change_gap_fore = threshold_lane_change_gap_fore
        retval.threshold_lane_change_gap_rear = threshold_lane_change_gap_rear
        retval.k_follow = k_follow

        retval
    end
end

get_name(::Tim2DDriver) = "Tim2DDriver"
action_context(driver::Tim2DDriver) = driver.action_context


function observe!(driver::Tim2DDriver, scene::Scene, roadway::Roadway, egoid::Int)

    rec = driver.rec
    update!(rec, scene)
    vehicle_index = get_index_of_first_vehicle_with_id(rec, egoid)
    @assert(vehicle_index != 0)

    veh_ego = scene[vehicle_index]
    t = veh_ego.state.posF.t
    ϕ = veh_ego.state.posF.ϕ
    v = veh_ego.state.v
    len_ego = veh_ego.def.length

    left_lane_exists = convert(Float64, get(N_LANE_LEFT, rec, roadway, vehicle_index)) > 0
    right_lane_exists = convert(Float64, get(N_LANE_RIGHT, rec, roadway, vehicle_index)) > 0
    fore_M = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    fore_L = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
    fore_R = get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
    rear_L = get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())
    rear_R = get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())

    driver.state = :freeflow # by default
    THRESHOLD_FORE = 50.0
    if fore_M.Δs < THRESHOLD_FORE # there is a lead vehicle
        veh_M = scene[fore_M.ind]
        speed_M = veh_M.state.v
        if speed_M ≤ min(driver.v_des, v) # they are driving slower than we want

            speed_ahead = speed_M
            driver.state = :following

            # consider changing to a different lane
            if right_lane_exists &&
               fore_R.Δs > driver.threshold_lane_change_gap_rear + len_ego && # there is space rear
               rear_R.Δs > driver.threshold_lane_change_gap_fore + len_ego && # there is space fore
               (rear_R.ind == 0 || scene[rear_R.ind].state.v ≤ v) && # we are faster than any follower
               (fore_R.ind == 0 || scene[fore_R.ind].state.v > speed_ahead) # we are faster than any leader

                if fore_R.ind != 0
                    speed_ahead = scene[fore_R.ind].state.v
                end
                driver.state = :lanechange_right
            end
            if left_lane_exists &&
               fore_L.Δs > driver.threshold_lane_change_gap_rear + len_ego && # there is space rear
               rear_L.Δs > driver.threshold_lane_change_gap_fore + len_ego && # there is space fore
               (rear_L.ind == 0 || scene[rear_L.ind].state.v ≤ v) && # we are faster than any follower
               (fore_L.ind == 0 || scene[fore_L.ind].state.v > speed_ahead) # we are faster than any leader


                if fore_L.ind != 0
                    speed_ahead = scene[fore_L.ind].state.v
                end
                driver.state = :lanechange_left
            end
        end
    end

    v_des = NaN
    lane_offset = NaN

    if driver.state == :freeflow
        v_des = driver.v_des
        lane_offset = t
    elseif driver.state == :following
        veh_M = scene[fore_M.ind]
        speed_M = veh_M.state.v
        dist_M = fore_M.Δs
        v_des = min(speed_M*(1-exp(-driver.k_follow*dist_M/speed_M - 1)), driver.v_des)
        lane_offset = t
    elseif driver.state == :lanechange_left
        lane = roadway[veh_ego.state.posF.roadind.tag]
        lane_left = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        lane_offset = t - lane.width/2 - lane_left.width/2

        if fore_L.ind != 0
            v_des = min(scene[fore_L.ind].state.v, driver.v_des)
        else
            v_des = driver.v_des
        end
    elseif driver.state == :lanechange_right
        lane = roadway[veh_ego.state.posF.roadind.tag]
        lane_right = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        lane_offset = t + lane.width/2 + lane_right.width/2

        if fore_R.ind != 0
            v_des = min(scene[fore_R.ind].state.v, driver.v_des)
        else
            v_des = driver.v_des
        end
    else
        warning("Tim2DDriver: should not trigger")
        v_des = 0.0
        lane_offset = t
    end

    let
        # compute lane tracking
        kp = driver.k_lat_p
        kd = driver.k_lat_d
        vlat = v*sin(ϕ) # rate of change of lane offset
        driver.a_lat = -lane_offset*kp - vlat*kd
    end

    let
        # compute acceleration based on desired speed
        Δv = v_des - v
        driver.a_lon = Δv*driver.k_lon
    end

    driver
end

function _rand(μ::Float64, σ::Float64)
    if isnan(σ) || driver.σ ≤ 0.0
        μ
    else
        rand(Normal(μ, σ))
    end
end
Base.rand(driver::Tim2DDriver) = LatLonAccel(_rand(driver.a_lat, driver.σ_lat), _rand(driver.a_lon, driver.σ_lon))

function _pdf(μ::Float64, σ::Float64, a::Float64)
    if isnan(σ) || σ ≤ 0.0
        Inf
    else
        pdf(Normal(μ, σ), a)
    end
end
function _logpdf(μ::Float64, σ::Float64, a::Float64)
    if isnan(σ) || σ ≤ 0.0
        Inf
    else
        logpdf(Normal(μ, σ), a)
    end
end

function Distributions.pdf(driver::Tim2DDriver, a::LatLonAccel)
    _pdf(driver.a_lat, driver.σ_lat, a.a_lat) *
    _pdf(driver.a_lon, driver.σ_lon, a.a_lon)
end
function Distributions.logpdf(driver::Tim2DDriver, a::LatLonAccel)
    _logpdf(driver.a_lat, driver.σ_lat, a.a_lat) *
    _logpdf(driver.a_lon, driver.σ_lon, a.a_lon)
end

