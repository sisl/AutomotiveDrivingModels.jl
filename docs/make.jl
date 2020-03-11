using Documenter, AutomotiveDrivingModels

makedocs(
	 modules = [AutomotiveDrivingModels],
	 format = Documenter.HTML(),
	 sitename="AutomotiveDrivingModels.jl",
	 strict = true,
	 pages=[
		 "Home" => "index.md",
		 "Examples" => [
			     "examples/straight_roadway.md",
				"examples/stadium.md",
				"examples/intersection.md",
				"examples/crosswalk.md",
				"examples/sidewalk.md"
		 ],
		 "Manual" => [
			 "Roadways.md",
			     "actions.md",
				"states.md",
				"agent_definitions.md",
				"behaviors.md",
				"simulation.md",
				"collision_checkers.md",
				"feature_extraction.md"
		 ]
	 ]
	 )

deploydocs(
    repo = "github.com/sisl/AutomotiveDrivingModels.jl.git",
)
