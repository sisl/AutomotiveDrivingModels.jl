typealias SceneRecord QueueRecord{Vehicle}
SceneRecord(capacity::Int, timestep::Float64, frame_capacity::Int=100) = QueueRecord(Vehicle, capacity, timestep, frame_capacity)
Base.show(io::IO, rec::SceneRecord) = print(io, "SceneRecord(nscenes=", nframes(rec), ")")