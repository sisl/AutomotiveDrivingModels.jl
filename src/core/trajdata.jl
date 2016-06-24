immutable TrajdataFrame
    lo::Int # index in states of first vehicle in scene
    hi::Int # index in states of last vehicle in scene
    t::Float64 # time
end
immutable TrajdataState
    id::Int # vehicle ID
    state::VehicleState
end
Base.length(frame::TrajdataFrame) = frame.hi - frame.lo + 1 # number of cars in the frame

type Trajdata
    roadway::Roadway
    id::Int                    # id assigned to this trajdata

    vehdefs::Dict{Int, VehicleDef} # vehicle id -> vehdef
    states::Vector{TrajdataState} # list of vehicle states (for each scene)
    frames::Vector{TrajdataFrame} # list of frames
end
Trajdata(roadway::Roadway, trajdata_id::Int=0) = Trajdata(roadway, trajdata_id, Dict{Int, VehicleDef}(), TrajdataState[], TrajdataFrame[])

nframes(trajdata::Trajdata) = length(trajdata.frames)
frame_inbounds(trajdata::Trajdata, frame::Int) = 1 ≤ frame ≤ nframes(trajdata)
carsinframe(trajdata::Trajdata, frame::Int) = length(trajdata.frames[frame])
nth_carid(trajdata::Trajdata, frame::Int, n::Int=1) = trajdata.states[trajdata.frames[frame].lo + n-1].id

function iscarinframe(trajdata::Trajdata, carid::Int, frame::Int)
    frame = trajdata.frames[frame]
    for i in frame.lo : frame.hi
        s = trajdata.states[i]
        if s.id == carid
            return true
        end
    end
    false
end

# function car_df_index(trajdata::Trajdata, carid::Int, frame::Int)
#     #=
#     given frame and carid, find index of car in trajdata
#     Returns 0 if it does not exist
#     =#

#     lo = trajdata.car2start[carid]
#     framestart = trajdata.frames[lo]

#     retval = frame - framestart + lo
#     n_frames = trajdata.n_frames_in_dataset[lo]
#     if retval > lo + n_frames
#         retval = 0
#     end

#     retval
# end
# function get_frame_range(trajdata::Trajdata, carid::Int)
#     lo = trajdata.car2start[carid]
#     framestart = trajdata.frames[lo]
#     n_frames = trajdata.n_frames_in_dataset[lo]
#     frameend = framestart + n_frames - 1

#     framestart:frameend
# end
# function get_vehiclestate(trajdata::Trajdata, carid::Int, frame::Int)

#     dfind = car_df_index(trajdata, carid::Int, frame::Int)
#     trajdata.states[dfind]
# end
# get_vehicle(trajdata::Trajdata, carid::Int) = trajdata.vehicles[carid]
# function get_vehicle(trajdata::Trajdata, carid::Int, frame::Int)

#     veh = trajdata.vehicles[carid]
#     dfind = car_df_index(trajdata, carid, frame)
#     veh.state = trajdata.states[dfind]

#     veh
# end

# function get_turnrate(trajdata::Trajdata, id::Int, frame::Int, frenet::Bool=false)
#     if frame == 1 || !frame_inbounds(trajdata, frame) || !iscarinframe(trajdata, id, frame-1)
#         return 0.0 # no past info, assume zero
#     end

#     if frenet
#         past = get_vehiclestate(trajdata, id, frame-1).posF.ϕ
#         curr = get_vehiclestate(trajdata, id, frame).posF.ϕ
#         (curr - past) / NGSIM_TIMESTEP # [ft/s²]
#     else # global frame
#         past = get_vehiclestate(trajdata, id, frame-1).posG.θ
#         curr = get_vehiclestate(trajdata, id, frame).posG.θ
#         (curr - past) / NGSIM_TIMESTEP # [ft/s²]
#     end
# end
# function get_acceleration(trajdata::Trajdata, id::Int, frame::Int)
#     if frame == 1 || !frame_inbounds(trajdata, frame) || !iscarinframe(trajdata, id, frame-1)
#         return 0.0 # no past info, assume zero
#     end

#     v_past = get_vehiclestate(trajdata, id, frame-1).v
#     v_curr = get_vehiclestate(trajdata, id, frame).v

#     (v_curr - v_past) / NGSIM_TIMESTEP # [ft/s²]
# end
# function get_acceleration_lat(trajdata::Trajdata, id::Int, frame::Int)
#     if frame == 1 || !frame_inbounds(trajdata, frame) || !iscarinframe(trajdata, id, frame-1)
#         return 0.0 # no past info, assume zero
#     end


#     s_past = get_vehiclestate(trajdata, id, frame-1)
#     s_curr = get_vehiclestate(trajdata, id, frame)

#     curve = trajdata.roadway.centerlines[s_curr.posF.laneid]
#     proj = project_to_lane(s_curr.posG, curve)

#     v_past = s_past.v * sin(s_past.posF.ϕ)
#     v_curr = s_curr.v * sin(proj.θ)

#     (v_curr - v_past) / NGSIM_TIMESTEP # [ft/s²]
# end
# function get_acceleration_lon(trajdata::Trajdata, id::Int, frame::Int)
#     if frame == 1 || !frame_inbounds(trajdata, frame) || !iscarinframe(trajdata, id, frame-1)
#         return 0.0 # no past info, assume zero
#     end


#     s_past = get_vehiclestate(trajdata, id, frame-1)
#     s_curr = get_vehiclestate(trajdata, id, frame)

#     curve = trajdata.roadway.centerlines[s_curr.posF.laneid]
#     proj = project_to_lane(s_curr.posG, curve)

#     v_past = s_past.v * cos(s_past.posF.ϕ)
#     v_curr = s_curr.v * cos(proj.θ)

#     (v_curr - v_past) / NGSIM_TIMESTEP # [ft/s²]
# end