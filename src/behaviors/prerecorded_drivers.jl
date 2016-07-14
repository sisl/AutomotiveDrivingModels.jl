# type PrerecordedDriver <: DriverModel{NextState}
#     trajdata::Trajdata # log we pull from
#     time_start::Float64   # starting time [s]
#     t_current::Float64
#     nextstate::NextState
# end

# get_name(::PrerecordedDriver) = "PrerecordedDriver"

# function reset_hidden_state!(model::PrerecordedDriver)
#     model.t_current = model.time_start
#     model
# end
# function observe!(model::PrerecordedDriver, scene::Scene, roadway::Roadway, egoid::Int)
#     i = searchsortedfirst(model.trajdata.frames, model.t_current, by=frame->frame.t) # index of first frame ≥ t_current

#     if i == 1
#         i_lo,  = 1
#     elseif i ≤ length(nframes(model.trajdata.frames))

#     else # t_current > all times in model.trajdata.frames

#     end

#     model
# end

# Base.rand(model::PrerecordedDriver) = model.nextstate
# Distributions.pdf(model::PrerecordedDriver, a::NextState) = a == model.nextstate ? Inf : 0.0
# Distributions.logpdf(model::PrerecordedDriver, a::NextState) = a == model.nextstate ? Inf : -Inf