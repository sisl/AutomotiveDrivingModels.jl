export PrincetonDriver

# "DeepDriving: Learning Affordance for Direct Perception in Autonomous Driving"
type PrincetonDriver <: DriverModel{AccelTurnrate, IntegratedContinuous}
    action_context::IntegratedContinuous

    v_des::Float64 # desired speed

    threshold_lane_change_gap::Float64
    in_lane_system_activation::Float64
    on_marking_system_activation::Float64

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
end

get_name(::PrincetonDriver) = "PrincetonDriver"
action_context(driver::PrincetonDriver) = driver.action_context

function observe!(driver::PrincetonDriver, rec::SceneRecord, roadway::Roadway, egoid::Int)

    vehicle_index = get_index_of_first_vehicle_with_id(rec, egoid)

    driver.angle = convert(Float64, get(POSFYAW, rec, roadway, vehicle_index))

    # in lane system
    speed_ego = rec[vehicle_index].state.v
    posFt = convert(Float64, get(POSFT, rec, roadway, vehicle_index))
    d_ml = convert(Float64, get(MARKERDIST_LEFT, rec, roadway, vehicle_index))
    d_mr = convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, vehicle_index))
    if abs(posFt) < driver.in_lane_system_activation

        neighborfore = get_neighbor_fore_along_lane(get_scene(rec, 0), vehicle_index, roadway)

        driver.toMarking_LL = convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, vehicle_index))
        driver.toMarking_ML = d_ml
        driver.toMarking_MR = d_mr
        driver.toMarking_RR = convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, vehicle_index))
        driver.dist_LL = convert(Float64, get(DIST_FRONT_LEFT, rec, roadway, vehicle_index))
        driver.dist_MM = convert(Float64, get(DIST_FRONT, rec, roadway, vehicle_index, neighborfore=neighborfore))
        driver.dist_RR = convert(Float64, get(DIST_FRONT_RIGHT, rec, roadway, vehicle_index))

        speed_MM = convert(Float64, get(SPEED_FRONT, rec, roadway, vehicle_index, neighborfore=neighborfore))
        driver.rel_speed_MM = speed_ego - speed_MM
    end

    # on marking system
    if min(abs(d_ml), abs(d_mr)) < driver.on_marking_system_activation
        if abs(d_ml) ≤ abs(d_mr)
            # closest to left lane marking

            driver.toMarking_L = convert(Float64, get(MARKERDIST_LEFT_LEFT, rec, roadway, vehicle_index))
            driver.toMarking_M = d_ml
            driver.toMarking_R = d_mr
            driver.dist_L = convert(Float64, get(DIST_FRONT_LEFT, rec, roadway, vehicle_index))
            driver.dist_R = convert(Float64, get(DIST_FRONT, rec, roadway, vehicle_index))
            driver.rel_speed_L = speed_ego - speed_L
        else
            # closest to right lane marking

            driver.toMarking_L = d_ml
            driver.toMarking_M = d_mr
            driver.toMarking_R = convert(Float64, get(MARKERDIST_RIGHT_RIGHT, rec, roadway, vehicle_index))
            driver.dist_L = convert(Float64, get(DIST_FRONT, rec, roadway, vehicle_index))
            driver.dist_R = convert(Float64, get(DIST_FRONT_RIGHT, rec, roadway, vehicle_index))
        end
    end

    driver
end
function Base.rand(driver::PrincetonDriver)

    lane_offset = NaN
    lane_width = NaN
    dist_fore = NaN

    if !isnan(toMarking_ML)
        # in-lane system

        dist_fore = driver.dist_MM

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
            lane_offset = -(lane_width/2 + dirver.toMarking_MR)
        else
            lane_width = driver.toMarking_ML + driver.toMarking_MR
            lane_offset = lane_width - driver.toMarking_MR # positive towards left
        end
    else
        # on mark system

        state = :normal_driving

        dist_left = isnan(driver.dist_L) ? Inf : driver.dist_L
        dist_right = isnan(driver.dist_R) ? Inf : driver.dist_R
        dist_fore = min(dist_left, dist_right)
        if !isinf(dist_fore)
            if dist_left ≤ dist_right
                if !isnan(driver.dist_L) && driver.rel_speed_MM > 0.0

                    dist_fore = driver.dist_MM

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
            else
                if !isnan(driver.dist_R) && driver.rel_speed_MM > 0.0

                    dist_fore = driver.dist_MM

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
            end
        end

        if state == :left_lane_change
            lane_width = driver.toMarking_LL - driver.toMarking_ML
            lane_offset = lane_width/2 + driver.toMarking_ML
        elseif state == :right_lane_change
            lane_width = driver.toMarking_RR - driver.toMarking_MR
            lane_offset = -(lane_width/2 + dirver.toMarking_MR)
        else
            lane_width = driver.toMarking_ML + driver.toMarking_MR
            lane_offset = lane_width - driver.toMarking_MR # positive towards left
        end
    end



    # compute steering command
        ϕ = driver.angle
        C = driver.C
        steerCmd = C*(ϕ - lane_offset/lane_width)


    # compute desired speed
        v_max = driver.v_des
        c = driver.c
        d = driver.d
        v(t) = v_max*(1-exp(-c/v_max*dist(t)-d))

    # compute acceleration/brake command based on desired speed

    AccelTurnrate(0.0,0.0) # TODO: this
end
Distributions.pdf(driver::PrincetonDriver) = 0.0
Distributions.logpdf(driver::PrincetonDriver) = -Inf
