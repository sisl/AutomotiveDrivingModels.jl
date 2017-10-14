# export PrerecordedDriver

# type PrerecordedDriver <: DriverModel{NextState}
#     trajdata::Trajdata # log we pull from
#     time_start::Float64   # starting time [s]
#     t_current::Float64
#     timestep::Float64
#     nextstate::VehicleState
# end
# function PrerecordedDriver(trajdata::Trajdata, frame_start::Int, egoid::Int, timestep::Float64=get_timestep(trajdata))
#     time_start = get_time(trajdata, frame_start)
#     t_current = NaN
#     nextstate = get_state(trajdata, egoid, frame_start)
#     PrerecordedDriver(trajdata, time_start, t_current, timestep, nextstate)
# end

# get_name(::PrerecordedDriver) = "PrerecordedDriver"
# function reset_hidden_state!(model::PrerecordedDriver)
#     model.t_current = model.time_start
#     model
# end
# function observe!(model::PrerecordedDriver, scene::Scene, roadway::Roadway, egoid::Int)
#     model.t_current += model.timestep

#     i = searchsortedfirst(model.trajdata.frames, model.t_current, by=frame_or_time->isa(frame_or_time, TrajdataFrame) ? frame_or_time.t : frame_or_time) # index of first frame ≥ t_current

#     if i == 1
#         lo, hi = 1, 2
#     elseif i ≤ nframes(model.trajdata)
#         lo, hi = i-1, i
#     else # t_current > all times in model.trajdata.frames
#         lo, hi = i-2, i-1
#     end

#     t = model.t_current
#     t_lo = get_time(model.trajdata, lo)
#     t_hi = get_time(model.trajdata, hi)

#     γ = (t - t_lo) / (t_hi - t_lo)

#     if (γ == 0.0 && !iscarinframe(model.trajdata, egoid, lo)) ||
#        (γ == 1.0 && !iscarinframe(model.trajdata, egoid, hi)) ||
#        (!iscarinframe(model.trajdata, egoid, lo) || !iscarinframe(model.trajdata, egoid, hi))

#         model.nextstate = VehicleState()
#     else
#         state_lo = get_state(model.trajdata, egoid, lo)
#         state_hi = get_state(model.trajdata, egoid, hi)
#         model.nextstate = lerp(state_lo, state_hi, γ, model.trajdata.roadway)
#     end



#     model
# end

# Base.rand(model::PrerecordedDriver) = NextState(model.nextstate)
# Distributions.pdf(model::PrerecordedDriver, a::NextState) = a.s == model.nextstate ? Inf : 0.0
# Distributions.logpdf(model::PrerecordedDriver, a::NextState) = a.s == model.nextstate ? Inf : -Inf