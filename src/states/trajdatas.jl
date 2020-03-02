"""
    Trajdata
Trajdata is a specific instance of ListRecord defined in Records.jl. It is a collection of Scenes
"""
const Trajdata = ListRecord{VehicleState, VehicleDef, Int}
Trajdata(timestep::Float64) = ListRecord(timestep, VehicleState, VehicleDef, Int)
function Trajdata(scenes::Vector{EntityFrame{S,D,I}}, timestep::Float64) where {S,D,I}
    trajdata = ListRecord(timestep, S, D, I)
    push!.(Ref(trajdata), scenes)
    return trajdata
end

Base.show(io::IO, trajdata::Trajdata) = @printf(io, "Trajdata(%d frames)", nframes(trajdata))
