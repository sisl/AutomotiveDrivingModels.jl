@with_kw struct BicycleModel
    def::BoundingBoxDef
    a::Float64 = 1.5 # distance between cg and front axle [m]
    b::Float64 = 1.5 # distance between cg and rear axle [m]
end