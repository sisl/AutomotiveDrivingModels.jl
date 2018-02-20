const Trajdata = ListRecord{VehicleState, VehicleDef, Int}

Trajdata(timestep::Float64) = ListRecord(timestep, VehicleState, VehicleDef, Int)


Base.show(io::IO, trajdata::Trajdata) = @printf(io, "Trajdata(%d frames)", nframes(trajdata))


function Trajdata{S,D,I}(timestep::Float64, rec::Vector{EntityFrame{S,D,I}})
    frames = Array{RecordFrame}(length(rec))
    states = Array{RecordState{S,I}}(nstates(rec))
    defs = Dict{I, D}()

    lo = 1
    for (i, frame) in enumerate(rec)

        hi = lo-1
        for entity in frame
            hi += 1
            defs[entity.id] = entity.def
            states[hi] = RecordState{S,I}(entity.state, entity.id)
        end

        frames[i] = RecordFrame(lo, hi)
        lo = hi + 1
    end

    return ListRecord{S,D,I}(timestep, frames, states, defs)
end


function nstates{S,D,I}(rec::Vector{EntityFrame{S,D,I}})
    retval = 0
    for i in 1 : length(rec)
        retval += length(rec[i])
    end
    return retval
end
