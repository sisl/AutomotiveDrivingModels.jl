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

function iscarinframe(trajdata::Trajdata, id::Int, frame::Int)
    frame = trajdata.frames[frame]
    for i in frame.lo : frame.hi
        s = trajdata.states[i]
        if s.id == id
            return true
        end
    end
    false
end

function get_vehiclestate(trajdata::Trajdata, id::Int, frame::Int)
    frame = trajdata.frames[frame]
    for i in frame.lo : frame.hi
        s = trajdata.states[i]
        if s.id == id
            return s.state
        end
    end
    error("vehicle not found for id $id and frame $(frame)!")
end
get_vehicledef(trajdata::Trajdata, id::Int) = trajdata.vehdefs[id]

function get_vehicle!(veh::Vehicle, trajdata::Trajdata, id::Int, frame::Int)
    veh.state = get_vehiclestate(trajdata, id, frame)
    veh.def = get_vehicledef(trajdata, id)
    veh
end
get_vehicle(trajdata::Trajdata, id::Int, frame::Int) = get_vehicle!(Vehicle(), trajdata, id, frame)