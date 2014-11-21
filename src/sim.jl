

# em       = load_em()
# s        = init_scene()
# road     = Roadway()
# trace    = [s]
# log      = RunLog()

# while !is_done(s)

# 	# ego car
# 	observations = observe(s, ego, features)
# 	actions[ego] = select_action(observations)

# 	for car in s
# 		observations = observe(s, car, features)
# 		emsample = encode(observations, features)
# 		sample!(em, emsample)
# 		actions[car] = action(emsample)
# 	end
# 	propagate!(s, actions)
# 	add_to_trace!(s, actions, trace)
# end

# metrics = compute_metrics()
# save_metrics(metrics)

# # ------------------------------------------

# Scene
# 	- equivalent to a single traffic state in time

# Roadway
# 	- contains roadway structure information

# Trace
# 	- contains scene history

# Log
# 	- full scene recording for playback


# function observe(s::Scene, car::Car, features::Vector{Feature})

# 	# for each indicator feature build Dict{Symbol, value} (value is Int or Float64 or whatever)
# end

# function encode(observations::Dict{Symbol, Any}, features::Vector{Feature})
# 	# take each observation and bin it appropriately for the EM
# 	# returns a Dict{Symbol,Int}
# end

# function sample!(em::EM, conditions::Dict{Symbol, Int})

# 	# run through nodes in topological order, building the instantiation vector as we go
# 	# nodes we already know (hasley(conditions, nodesym)) we use
# 	# nodes we do not know we sample from
# 	# modifies conditions to include new samples symbols
# end

# function action(emsample::Dict{Symbol, Int})
# 	# construct an action from the emsample
# 	# create a dictionary mapping target features to specific values (float64, probably)
# end

# function propagate!(s::Scene, actions::Vector{Dict{Symbol, Any}})
# 	# propagate each car in the scene using its actions
# 	# this should be deterministic
# 	# modify values in-place rather than allocating new values
# end

# function add_to_trace(s::Scene, actions::Vector{Dict{Symbol, Any}}, trace::DataFrame)
# 	# record things to trace
# end