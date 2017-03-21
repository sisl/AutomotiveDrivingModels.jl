export
    get_speeds

"""
    get_speeds(trajdata::Trajdata, carid::Int)
Get all of the speeds, in order, for the given vehicle in the trajdata
"""
function get_speeds(trajdata::Trajdata, carid::Int)
    retval = Float64[]
    for frame in 1 : nframes(trajdata)
        if iscarinframe(trajdata, carid, frame)
            state = get_state(trajdata, carid, frame)
            push!(retval, state.v)
        end
    end
    retval
end