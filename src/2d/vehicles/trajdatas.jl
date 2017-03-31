typealias Trajdata ListRecord{VehicleState, VehicleDef, Int}
Trajdata(timestep::Float64) = ListRecord(timestep, VehicleState, VehicleDef, Int)
Base.show(io::IO, trajdata::Trajdata) = @printf(io, "Trajdata(%d frames)", nframes(trajdata))
