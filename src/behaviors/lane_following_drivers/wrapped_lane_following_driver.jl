struct WrappedLaneFollowingDriver{A,L<:LaneFollowingDriver} <: DriverModel{A}
    submodel::L
end
get_name(model::WrappedLaneFollowingDriver) = "WrappedLaneFollowingDriver(" * get_name(model.submodel) * ")"

function track_longitudinal!(model::WrappedLaneFollowingDriver, v_ego::Float64, v_oth::Float64, headway::Float64)
    track_longitudinal!(model.submodel, v_ego, v_oth, headway)
    return model
end

Base.rand(model::WrappedLaneFollowingDriver) = convert(A, rand(model.submodel))
Distributions.pdf{A}(model::WrappedLaneFollowingDriver{A}, a::A) = pdf(model.submodel, convert(Accel, a))
Distributions.logpdf{A}(model::WrappedLaneFollowingDriver{A}, a::A) = logpdf(model.submodel, convert(Accel, a))