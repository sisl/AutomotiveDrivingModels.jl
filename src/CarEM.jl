# Car Encounter Model

module CarEM

push!(LOAD_PATH, "/home/tim/Documents/code/wheelerworkspace/Bosch/model/")

using BinMaps
using BayesNets
using Features

export EM
export Roadway, PointSE2, Vehicle
export encounter_model
export export_to_text

immutable EM
	BN       :: BayesNet
	features :: Vector{AbstractFeature}
	binmaps  :: Vector{AbstractBinMap}
	istarget :: BitVector # whether a feature is a target or an indicator
end

function encounter_model( 
	BN         :: BayesNet, 
	binmapdict :: Dict{Symbol, AbstractBinMap}, 
	targets    :: Vector{AbstractFeature}, 
	indicators :: Vector{AbstractFeature}
	)

	features = AbstractFeature[symbol2feature(sym) for sym in BN.names]
	binmaps  = AbstractBinMap[binmapdict[sym] for sym in BN.names]
	istarget = falses(length(features))
	for (i,f) in enumerate(features)
		if in(f, targets)
			istarget[i] = true
		elseif !in(f, indicators)
			error("Feature not in targets or indicators")
		end
	end

	EM(BN, features, binmaps, istarget)
end

include("common.jl")
include("io.jl")
include("sim.jl")


end # module