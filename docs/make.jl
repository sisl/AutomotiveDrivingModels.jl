using Documenter
using Literate
using AutomotiveDrivingModels

# generate tutorials and how-to guides using Literate
src = joinpath(@__DIR__, "src")
lit = joinpath(@__DIR__, "lit")
notebooks = joinpath(src, "notebooks")

for (root, _, files) in walkdir(lit), file in files
    splitext(file)[2] == ".jl" || continue
    ipath = joinpath(root, file)
    opath = splitdir(replace(ipath, lit=>src))[1]
    Literate.markdown(ipath, opath, documenter = true)
    Literate.notebook(ipath, notebooks, execute = false)
end

makedocs(
	modules = [AutomotiveDrivingModels],
	format = Documenter.HTML(),
	sitename="AutomotiveDrivingModels.jl",
	pages=[
		"Home" => "index.md",
		"Tutorials" => [
			"tutorials/straight_roadway.md",
			"tutorials/stadium.md",
			"tutorials/intersection.md",
			"tutorials/crosswalk.md",
			"tutorials/sidewalk.md"
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
