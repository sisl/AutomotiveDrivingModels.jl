immutable RoadType
    s::Float64 # starting location
    typ::String # 'highway', etc.
    speed_limit::Float64 # [m/s]
end


type Road
    name::String
    length::Float64
    id::UInt32 # starting at 0
    types::Vector{RoadType} # in order
    refline::Vector{LaneGeometry}
    sections::Vector{LaneSection}
end