export
    TrajdataEditMode,

    remove_offroad_vehicles!,
    remove_unaligned_vehicles!,
    interpolate_to_timestep!

type TrajdataEditMode
    roadway::Roadway
    scenes::Vector{Scene}
    time::Vector{Float64}
end
function Base.push!(tdem::TrajdataEditMode, scene::Scene, Δt::Float64=1.0)
    t = isempty(tdem.time) ? 0.0 : tdem.time[end] + Δt
    push!(tdem.scenes, scene)
    push!(tdem.time, t)
    return tdem
end

function Base.convert(::Type{TrajdataEditMode}, trajdata::Trajdata)
    roadway = trajdata.roadway
    scenes = Scene[get!(Scene(carsinframe(trajdata, frame)), trajdata, frame) for frame in 1 : nframes(trajdata)]
    time = Float64[frame.t for frame in trajdata.frames]
    TrajdataEditMode(roadway, scenes, time)
end
function Base.convert(::Type{Trajdata}, tdem::TrajdataEditMode)
    roadway = tdem.roadway
    vehicles = Dict{Int, VehicleDef}()
    states = Array(TrajdataState, sum(s->length(s), tdem.scenes))
    frames = Array(TrajdataFrame, length(tdem.scenes))

    states_index = 0
    for (frame_index, scene) in enumerate(tdem.scenes)
        lo = states_index + 1

        for veh in scene
            if !haskey(vehicles, veh.id)
                vehicles[veh.id] = veh.def
            else
                @assert(vehicles[veh.id] == veh.def)
            end
            states[states_index+=1] = TrajdataState(veh.id, veh.state)
        end
        frames[frame_index] = TrajdataFrame(lo, states_index, tdem.time[frame_index])
    end

    Trajdata(roadway, vehicles, states, frames)
end

function remove_offroad_vehicles!(tdem::TrajdataEditMode, threshold_lane_lateral_offset::Float64=2.5)
    for scene in tdem.scenes
        i = 0
        while i < length(scene)
            i += 1
            veh = scene[i]
            if abs(veh.state.posF.t) > threshold_lane_lateral_offset
                deleteat!(scene, i)
                i -= 1
            end
        end
    end
    tdem
end
function remove_unaligned_vehicles!(tdem::TrajdataEditMode, threshold_lane_angle::Float64=deg2rad(45))
    for scene in tdem.scenes
        i = 0
        while i < length(scene)
            i += 1
            veh = scene[i]
            if abs(veh.state.posF.ϕ) > threshold_lane_angle
                deleteat!(scene, i)
                i -= 1
            end
        end
    end
    tdem
end

function interpolate_to_timestep!(tdem::TrajdataEditMode, timestep::Float64)
    time = collect(tdem.time[1]:timestep:tdem.time[end])
    scenes = Array(Scene, length(time))
    scenes[1] = tdem.scenes[1]

    frame_old = 1
    for i in 2 : length(time)
        t = time[i]

        # move frame_old to one past the time, if it exists
        while tdem.time[frame_old] < t
            frame_old += 1
        end

        # interpolation value ∈ [0,1]
        γ = (t - tdem.time[frame_old-1]) / (tdem.time[frame_old] - tdem.time[frame_old-1])

        scene_lo = tdem.scenes[frame_old-1]
        scene_hi = tdem.scenes[frame_old]

        # interpolate all vehicles that are in both scenes
        # if a vehicle is not in both scenes, do not inlude it
        scenes[i] = Scene(min(length(scene_lo), length(scene_hi)))
        for veh_lo in scene_lo
            id = veh_lo.id
            veh_index = findfirst(scene_hi, id)
            if veh_index != 0
                veh_hi = scene_hi[veh_index]
                @assert(veh_lo.def == veh_hi.def)
                veh_interp = Vehicle(lerp(veh_lo.state, veh_hi.state, γ, tdem.roadway), veh_lo.def)
                push!(scenes[i], veh_interp)
            end
        end
    end

    tdem.time = time
    tdem.scenes = scenes
    tdem
end

# smooth trajectories

# remove ghost cars