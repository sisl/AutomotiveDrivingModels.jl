using Test
using AutomotiveDrivingModels
using DataFrames
using Distributions

@testset "Vec" begin 
    include("vec-tests/vec_runtests.jl")
end

@testset "AutomotiveDrivingModels" begin 
    include("test_roadways.jl")
    include("test_agent_definitions.jl")
    include("test_states.jl")
    include("test_collision_checkers.jl")
    include("test_actions.jl")
    include("test_features.jl")
    include("test_behaviors.jl")
    include("test_simulation.jl")
end
