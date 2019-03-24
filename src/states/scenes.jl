const Scene = Frame{Vehicle}
Scene(n::Int=100) = Frame(Vehicle, n)
Scene(arr::Vector{Vehicle}) = Frame{Vehicle}(arr, length(arr))

Base.show(io::IO, scene::Scene) = print(io, "Scene(with $(length(scene)) cars)")

const SceneRecord = QueueRecord{Vehicle}
SceneRecord(capacity::Int, timestep::Float64, frame_capacity::Int=100) = QueueRecord(Vehicle, capacity, timestep, frame_capacity)
Base.show(io::IO, rec::SceneRecord) = print(io, "SceneRecord(nscenes=", nframes(rec), ")")