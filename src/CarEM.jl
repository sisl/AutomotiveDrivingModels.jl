# Car Encounter Model
# 	largely based off of EM sampling code by Dr. Mykel Kochenderfer 
# 	and translations by Youngjun Kim
# See the encounter model representation in Dr. Kochenderfer's work on Aircraft Encounter Models

module CarEM

using BinMaps

export EM
export export_to_text

immutable EM
	labels   :: Vector{Symbol}          # node labels
	n        :: Int                     # number of nodes
	G        :: BitMatrix               # adjacency matrix. G[i,j] is true iff [i] is a parent of [j]
	r        :: Vector{Int}             # bincounts. r[i] is the number of discrete values node i can take on
	N        :: Vector{Matrix{Int}}     # sufficient statistics. N[i] is an r[i]×q[i] matrix
	binmaps  :: Vector{AbstractBinMap}  # bin mappers
end

include("io.jl")


end # module
