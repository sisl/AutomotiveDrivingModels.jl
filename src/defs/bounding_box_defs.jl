"""
    Vehicle definition which contains a class and a bounding box.
"""
struct BoundingBoxDef
    class::Int
    len::Float64
    wid::Float64
end
function BoundingBoxDef(;
    class::Int = AgentClass.CAR, # ∈ AgentClass
    len::Float64 = 4.0,
    wid::Float64 = 1.8,
    )
    return BoundingBoxDef(class, len, wid)
end

const VehicleDef = BoundingBoxDef # for backwards compatibility

const NULL_VEHICLEDEF = BoundingBoxDef(class=AgentClass.CAR, len=NaN, wid=NaN)

function Base.show(io::IO, d::BoundingBoxDef)
    class = d.class == AgentClass.CAR ? "CAR" :
            d.class == AgentClass.MOTORCYCLE ? "MOTORCYCLE" :
            d.class == AgentClass.TRUCK ? "TRUCK" :
            d.class == AgentClass.PEDESTRIAN ? "PEDESTRIAN" :
            "UNKNOWN"
    @printf(io, "BoundingBoxDef(%s, %.3f, %.3f)", class, d.len, d.wid)
end
Base.write(io::IO, ::MIME"text/plain", def::BoundingBoxDef) = @printf(io, "%d %.16e %.16e", def.class, def.len, def.wid)
function Base.read(io::IO, ::MIME"text/plain", ::Type{BoundingBoxDef})
    tokens = split(strip(readline(io)), ' ')
    return BoundingBoxDef(class=parse(Int, tokens[1]),
                          len=parse(Float64, tokens[2]),
                          wid=parse(Float64, tokens[3]))
end