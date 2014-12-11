# Car Encounter Model

module CarEM

push!(LOAD_PATH, "/home/tim/Documents/code/wheelerworkspace/Bosch/model/")

using BinMaps
using BayesNets
using Features
import Base: get

export EM
export Roadway, PointSE2, Vehicle, Scene
export encounter_model, get_targets, get_indicators
export export_to_text
export get, clearmeta
export SEC_PER_FRAME, set_sec_per_frame

immutable EM
	BN       :: BayesNet
	features :: Vector{AbstractFeature}
	binmaps  :: Vector{AbstractBinMap}
	istarget :: BitVector # whether a feature is a target or an indicator
end

function encounter_model{A<:AbstractBinMap, B<:AbstractFeature, C<:AbstractFeature}( 
	BN         :: BayesNet, 
	binmapdict :: Dict{Symbol, A}, 
	targets    :: Vector{B}, 
	indicators :: Vector{C}
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


get_targets(em::EM) = em.features[em.istarget]
get_indicators(em::EM) = em.features[!(em.istarget)]

SEC_PER_FRAME = 0.25 # [sec]
function set_sec_per_frame(Δt::Float64)
	@assert(Δt > 0.0)
	global SEC_PER_FRAME = Δt
end

include("common.jl")
include("io.jl")
include("sim.jl")
include("feature_extract.jl")


end # module