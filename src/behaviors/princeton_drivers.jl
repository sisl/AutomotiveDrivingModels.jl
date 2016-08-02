export PrincetonDriver

# "DeepDriving: Learning Affordance for Direct Perception in Autonomous Driving"
type PrincetonDriver <: DriverModel{AccelTurnrate, IntegratedContinuous}
    action_context::IntegratedContinuous
    rec::SceneRecord

    σ_lat::Float64 # constant standard deviation (optional, set to NaN or 0 if not needed)
    σ_lon::Float64 # constant standard deviation (optional, set to NaN or 0 if not needed)

    v_des::Float64 # desired speed

    threshold_lane_change_gap::Float64
    in_lane_system_activation::Float64
    on_marking_system_activation::Float64

    # tuneable parameters
    c::Float64
    d::Float64
    k_speed::Float64
    k_angle_p::Float64
    k_angle_d::Float64

    # features
    angle::Float64        # angle between the car’s heading and the tangent of the road
    # “in lane system”, when driving in the lane:
    toMarking_LL::Float64 # distance to the left lane marking of the left lane
    toMarking_ML::Float64 # distance to the left lane marking of the current lane
    toMarking_MR::Float64 # distance to the right lane marking of the current lane
    toMarking_RR::Float64 # distance to the right lane marking of the right lane
    dist_LL::Float64      # distance to the preceding car in the left lane
    dist_MM::Float64      # distance to the preceding car in the current lane
    dist_RR::Float64      # distance to the preceding car in the right lane
    # "on mark system"
    toMarking_L::Float64  # distance to the left lane marking
    toMarking_M::Float64  # distance to the central lane marking
    toMarking_R::Float64  # distance to the right lane marking
    dist_L::Float64       # distance to the preceding car in the left lane
    dist_R::Float64       # distance to the preceding car in the right lane

    # additional features that probably should have been in the Princeton featureset
    rel_speed_MM::Float64 # relative speed with respect to lead car (speed_ego - speed_lead)
    rel_speed_L::Float64
    rel_speed_R::Float64
    speed_ego::Float64

    function PrincetonDriver(
        action_context::IntegratedContinuous;

        rec::SceneRecord = SceneRecord(1, action_context.Δt),

        σ_lat::Float64 = NaN,
        σ_lon::Float64 = NaN,
        v_des::Float64 = 29.0,

        threshold_lane_change_gap::Float64 = 5.0,
        in_lane_system_activation::Float64 = 1.0,
        on_marking_system_activation::Float64 = 0.5,

        c::Float64=1.0,
        d::Float64=1.0,
        k_speed::Float64=1.0,
        k_angle_p::Float64=1.0,
        k_angle_d::Float64=0.1,
        )

        retval = new()

        retval.action_context=action_context
        retval.rec = rec

        retval.σ_lat = σ_lat
        retval.σ_lon = σ_lon
        retval.v_des = v_des

        retval.threshold_lane_change_gap = threshold_lane_change_gap
        retval.in_lane_system_activation = in_lane_system_activation
        retval.on_marking_system_activation = on_marking_system_activation

        retval.c = c
        retval.d = d
        retval.k_speed = k_speed
        retval.k_angle_p = k_angle_p
        retval.k_angle_d = k_angle_d

        retval
    end
end

get_name(::PrincetonDriver) = "PrincetonDriver"
action_context(driver::PrincetonDriver) = driver.action_context
function clear_features!(driver::PrincetonDriver)
    driver.toMarking_LL = NaN
    driver.toMarking_ML = NaN
    driver.toMarking_MR = NaN
    driver.toMarking_RR = NaN
    driver.dist_LL = NaN
    driver.dist_MM = NaN
    driver.dist_RR = NaN

    driver.toMarking_L = NaN
    driver.toMarking_M = NaN
    driver.toMarking_R = NaN
    driver.dist_L = NaN
    driver.dist_R = NaN

    driver.rel_speed_MM = NaN
    driver.rel_speed_L = NaN
    driver.rel_speed_R = NaN

    driver
end
function observe!(driver::PrincetonDriver, scene::Scene, roadway::Roadway, egoid::Int)

    rec = driver.rec
    update!(rec, scene)
    vehicle_index = get_index_of_first_vehicle_with_id(rec, egoid)
    @assert(vehicle_index != 0)

    clear_features!(driver)
    driver.angle = convert(Float64, get(POSFYAW, rec, roadway, vehicle_index))

    # in lane system
    speed_ego = rec[vehicle_index].state.v
    driver.speed_ego = speed_ego
    posFt = convert(Float64, get(POSFT, rec, roadway, vehicle_index))
    d_ml = convert(Float64, get(MARKERDIST_LEFT, rec, roadway, vehicle_index))
    d_mr = convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, vehicle_index))

     # println("posFt: ", abs(posFt))
    if abs(posFt) < driver.in_lane_system_activation

        neighborfore = get_neighbor_fore_along_lane(get_scene(rec, 0), vehicle_index, roadway)

         # println("neighborfore: ", neighborfore)

        driver.toMarking_LL = convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, vehicle_index))
        driver.toMarking_ML = d_ml
        driver.toMarking_MR = d_mr
        driver.toMarking_RR = convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, vehicle_index))
        driver.dist_LL = convert(Float64, get(DIST_FRONT_LEFT, rec, roadway, vehicle_index))
        driver.dist_MM = convert(Float64, get(DIST_FRONT, rec, roadway, vehicle_index, neighborfore=neighborfore))
        driver.dist_RR = convert(Float64, get(DIST_FRONT_RIGHT, rec, roadway, vehicle_index))

        if neighborfore.ind != 0
            speed_MM = scene[neighborfore.ind].state.v
            driver.rel_speed_MM = speed_ego - speed_MM
        end
    end

    # on marking system
    if min(abs(d_ml), abs(d_mr)) < driver.on_marking_system_activation

        speed_L = speed_R = NaN

        scene = get_scene(rec, 0)
        neighborfore_left = NeighborForeResult(0,Inf)
        neighborfore_right = NeighborForeResult(0,Inf)

        if abs(d_ml) ≤ abs(d_mr)
            # closest to left lane marking
            neighborfore_left = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway)
            neighborfore_right = get_neighbor_fore_along_lane(scene, vehicle_index, roadway)

            driver.toMarking_L = convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, vehicle_index))
            driver.toMarking_M = d_ml
            driver.toMarking_R = d_mr
        else
            # closest to right lane marking
            neighborfore_left = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway)
            neighborfore_right = get_neighbor_fore_along_lane(scene, vehicle_index, roadway)

            driver.toMarking_L = d_ml
            driver.toMarking_M = d_mr
            driver.toMarking_R = convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, vehicle_index))
        end

        driver.dist_L = convert(Float64, get(DIST_FRONT, rec, roadway, vehicle_index, neighborfore=neighborfore_left))
        driver.dist_R = convert(Float64, get(DIST_FRONT, rec, roadway, vehicle_index, neighborfore=neighborfore_right))
        if neighborfore_left.ind != 0
            driver.rel_speed_L = speed_ego - scene[neighborfore_left.ind].state.v
        end
        if neighborfore_right.ind != 0
            driver.rel_speed_R = speed_ego - scene[neighborfore_right.ind].state.v
        end
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
function Base.rand(driver::PrincetonDriver)

    lane_offset = NaN
    lane_width = NaN
    dist_fore = NaN

    in_lane_act = false
    on_marking_act = false

    if !isnan(driver.toMarking_ML)
        # in-lane system

        in_lane_act = true

        dist_fore = driver.dist_MM
         # println("dist_fore: ", dist_fore)

        state = :normal_driving
        if !isnan(driver.rel_speed_MM) && driver.rel_speed_MM > 0.0

            if !isnan(driver.toMarking_LL) && # left lane exists
               (isnan(driver.dist_LL) || driver.dist_LL > driver.threshold_lane_change_gap) # lane changing allowable

               state = :left_lane_change
            elseif !isnan(driver.toMarking_RR) && # left lane exists
                   (isnan(driver.dist_RR) || driver.dist_RR > driver.threshold_lane_change_gap) # lane changing allowable

                state = :right_lane_change
            else
                state = :slow_down
            end
        end

        if state == :left_lane_change
            lane_width = driver.toMarking_LL - driver.toMarking_ML
            lane_offset = lane_width/2 + driver.toMarking_ML
        elseif state == :right_lane_change
            lane_width = driver.toMarking_RR - driver.toMarking_MR
            lane_offset = -(lane_width/2 + driver.toMarking_MR)
        else
            lane_width = driver.toMarking_ML + driver.toMarking_MR
            lane_offset = lane_width/2 - driver.toMarking_MR # positive towards left
        end
    elseif !isnan(driver.toMarking_L) || !isnan(driver.toMarking_R)
        # on mark system

        on_marking_act = true

        state = :normal_driving

        dist_left = isnan(driver.dist_L) ? Inf : driver.dist_L
        dist_right = isnan(driver.dist_R) ? Inf : driver.dist_R
        dist_fore = min(dist_left, dist_right)
         # println("dist_fore2: ", dist_fore)

        if !isinf(dist_fore) && (
               (dist_left ≤ dist_right && !isnan(driver.dist_L) && driver.rel_speed_L > 0.0) ||
               (dist_right ≤ dist_left && !isnan(driver.dist_R) && driver.rel_speed_R > 0.0)
            )

            if !isnan(driver.toMarking_L) && # left lane exists
               (isnan(driver.dist_L) || driver.dist_L > driver.threshold_lane_change_gap) # lane changing allowable

               state = :left_lane_change
            elseif !isnan(driver.toMarking_R) && # left lane exists
                   (isnan(driver.dist_R) || driver.dist_R > driver.threshold_lane_change_gap) # lane changing allowable

                state = :right_lane_change
            else
                state = :slow_down
            end
        else
            # choose between head left and head right
            # just pick the closest one

            if (isnan(driver.toMarking_L) ? Inf : driver.toMarking_L) <
               (isnan(driver.toMarking_R) ? Inf : driver.toMarking_R)

                state = :left_lane_change
            else
                state = :right_lane_change
            end
        end

        if state == :left_lane_change
            lane_width = driver.toMarking_L - driver.toMarking_M
            lane_offset = lane_width/2 + driver.toMarking_M
        elseif state == :right_lane_change
            lane_width = driver.toMarking_R - driver.toMarking_M
            lane_offset = -(lane_width/2 + driver.toMarking_M)
        end
    else
        # not on the road! Drive straight
        return AccelTurnrate(0.0, 0.0)
    end

    # compute steering command
    ϕ = driver.angle
    kp = driver.k_angle_p*3.0
    kd = driver.k_angle_d*3
    # ω = -C*(ϕ - lane_offset/lane_width)

    dt = driver.speed_ego * sin(ϕ) # rate of change of lane offset
    ω = lane_offset*kp - dt*kd

    # if isnan(ω)
    #     println("ω:           ", ω)
    #     println("C:           ", C)
    #     println("ϕ:           ", ϕ)
    #     println("lane_offset: ", lane_offset)
    #     println("lane_width:  ", lane_width)
    #     println("in_lane_act: ", in_lane_act)
    #     println("on_marking_act: ", on_marking_act)
    #     println("")
    #     println("toMarking_L: ", driver.toMarking_L)
    #     println("toMarking_M: ", driver.toMarking_M)
    #     println("toMarking_R: ", driver.toMarking_R)
    #     println("")
    #     println("toMarking_LL: ", driver.toMarking_LL)
    #     println("toMarking_ML: ", driver.toMarking_ML)
    #     println("toMarking_MR: ", driver.toMarking_MR)
    #     println("toMarking_RR: ", driver.toMarking_RR)
    # end


    # compute desired speed
    v_max = driver.v_des
    c = driver.c
    d = driver.d
    if !isnan(dist_fore)

         # println("v_max:     ", v_max)
         # println("c:         ", c)
         # println("d:         ", d)
         # println("dist_fore: ", dist_fore)

        v_des = v_max*(1-exp(-c/v_max*dist_fore-d))
    else
        v_des = v_max
    end

    # compute acceleration based on desired speed
    Δv = v_des - driver.speed_ego
    a = Δv*driver.k_speed

    a = _rand(a, driver.σ_lon)
    ω = _rand(ω, driver.σ_lat)

    @assert(!isnan(a))
    @assert(!isnan(ω))

    # println("Δv: ", Δv)
    # println("a:  ", a)
    # println("ω:  ", ω)
    # println("")

    AccelTurnrate(a, ω)
end

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

function Distributions.pdf(driver::PrincetonDriver, a::AccelTurnrate)
    _pdf(driver.action_mean.a, driver.σ_lon, a.a) *
    _pdf(driver.action_mean.ω, driver.σ_lat, a.ω)
end
function Distributions.logpdf(driver::PrincetonDriver, a::AccelTurnrate)
    _logpdf(driver.action_mean.a, driver.σ_lon, a.a) +
    _logpdf(driver.action_mean.ω, driver.σ_lat, a.ω)
end
