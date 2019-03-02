using Documenter, AutomotiveDrivingModels

makedocs(
	 modules = [AutomotiveDrivingModels],
	 format = Documenter.HTML(),
	 sitename="AutomotiveDrivingModels.jl"
	 )

deploydocs(
    repo = "github.com/sisl/AutomotiveDrivingModels.jl.git",
)
