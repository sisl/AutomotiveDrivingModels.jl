baremodule AgentClass
    const MOTORCYCLE = 1
    const CAR        = 2
    const TRUCK      = 3
    const PEDESTRIAN = 4
end


"""
    Vehicle definition which contains a class and a bounding box.
"""
immutable VehicleDef
    class::Int # ∈ AgentClass
    length::Float64
    width::Float64
end
function VehicleDef(;
    class::Int=AgentClass.CAR,
    length::Float64=4.0,
    width::Float64=1.8,
    )

    VehicleDef(class, length, width)
end

const NULL_VEHICLEDEF = VehicleDef(AgentClass.CAR, NaN, NaN)

function Base.show(io::IO, d::VehicleDef)
    class = d.class == AgentClass.CAR ? "CAR" :
          d.class == AgentClass.MOTORCYCLE ? "MOTORCYCLE" :
          d.class == AgentClass.TRUCK ? "TRUCK" :
          d.class == AgentClass.PEDESTRIAN ? "PEDESTRIAN" :
          "UNKNOWN"
    @printf(io, "VehicleDef(%s, %.3f, %.3f)", class, d.length, d.width)
end
Base.write(io::IO, ::MIME"text/plain", def::VehicleDef) = @printf(io, "%d %.16e %.16e", def.class, def.length, def.width)
function Base.read(io::IO, ::MIME"text/plain", ::Type{VehicleDef})
    tokens = split(strip(readline(io)), ' ')
    class = parse(Int, tokens[1])
    length = parse(Float64, tokens[2])
    width = parse(Float64, tokens[3])
    return VehicleDef(class, length, width)
end