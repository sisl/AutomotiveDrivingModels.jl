using Test
using AutomotiveDrivingModels
using Distributions

@testset "Data structures" begin 
    include("test_records.jl")
end
include("test_roadways.jl")
include("test_agent_definitions.jl")
include("test_states.jl")
include("test_collision_checkers.jl")
include("test_actions.jl")
include("test_features.jl")
include("test_behaviors.jl")
include("test_simulation.jl")
