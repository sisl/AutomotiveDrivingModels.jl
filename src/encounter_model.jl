export EM
export encounter_model, build_bn, get_targets, get_indicators, index, parent_indeces
export get_target_lat, get_target_lon
export variable_bin_counts, counts, edges

immutable EM
    BN       :: BayesNet
    statsvec :: Vector{Matrix{Float64}}
    features :: Vector{AbstractFeature}
    binmaps  :: Vector{AbstractBinMap}
    istarget :: BitVector # whether a feature is a target or an indicator
end
function encounter_model{A<:AbstractBinMap, B<:AbstractFeature, C<:AbstractFeature, R<:Real}( 
    BN         :: BayesNet,
    statsvec   :: Vector{Matrix{R}},
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

    statsvecFloat = convert(Vector{Matrix{Float64}}, statsvec)
    EM(BN, statsvecFloat, features, binmaps, istarget)
end
function build_bn{F<:AbstractFeature, G<:AbstractFeature, R<:Real}(
    targets    :: Vector{F},
    indicators :: Vector{G},
    statsvec   :: Vector{Matrix{R}},
    adj        :: BitMatrix
    )

    names = [symbol(f)::Symbol for f in [targets, indicators]]
    n_nodes = length(names)
    @assert(n_nodes == size(adj, 1) == size(adj, 2))

    BN = BayesNet(names)

    r_arr = Array(Int, n_nodes)
    for (node,node_sym) in enumerate(names)
        stats = statsvec[node]
        r, q = size(stats) # r = num node instantiations, q = num parental instantiations
        states = [1:r]
        BN.domains[node] = DiscreteDomain(states)
        r_arr[node] = r
    end

    for (node,node_sym) in enumerate(names)

        stats = statsvec[node]
        r, q = size(stats)
        states = [1:r]

        stats += 1 # NOTE(tim): adding uniform prior
        probs = stats ./ sum(stats,1)

        # set any parents & populate probability table
        n_parents = sum(adj[:,node])
        if n_parents > 0
            parents = names[adj[:,node]]
            for pa in parents
                addEdge!(BN, pa, node_sym)
            end

            # populate probability table
            assignments = BayesNets.assignment_dicts(BN, parents)
            parameterFunction = BayesNets.discrete_parameter_function(assignments, vec(probs), r)
            setCPD!(BN, node_sym, CPDs.Discrete(states, parameterFunction))
        else
            # no parents
            setCPD!(BN, node_sym, CPDs.Discrete(states, vec(probs)))
        end

    end

    return BN
end

get_targets(em::EM) = em.features[em.istarget]
get_indicators(em::EM) = em.features[!(em.istarget)]
function get_target_lat(em::EM, targets::Vector{AbstractFeature} = get_targets(em))
    for target in targets
        if isa(target, Features.Feature_FutureTurnRate_250ms) ||
           isa(target, Features.Feature_FutureTurnRate_500ms) ||
           isa(target, Features.Feature_FutureDesiredAngle_250ms) ||
           isa(target, Features.Feature_FutureDesiredAngle_500ms)

           return target
        end
    end
    error("Lat target not found in targets: $(targets)")
end
function get_target_lon(em::EM, targets::Vector{AbstractFeature} = get_targets(em))
    for target in targets
        if isa(target, Features.Feature_FutureDesiredSpeed_250ms) ||
           isa(target, Features.Feature_FutureDesiredSpeed_500ms) ||
           isa(target, Features.Feature_FutureAcceleration_250ms) ||
           isa(target, Features.Feature_FutureAcceleration_500ms)

           return target
        end
    end
    error("Lon target not found in targets: $(targets)")
end

index(f::AbstractFeature, em::EM) = em.BN.index[symbol(f)]
index(f::Symbol, em::EM) = em.BN.index[f]
function parent_indeces(varindex::Int, em::EM)
    parent_names = BayesNets.parents(em.BN, em.BN.names[varindex])
    retval = Array(Int, length(parent_names))
    for (i, name) in enumerate(parent_names)
        retval[i] = index(name, em)
    end
    retval
end

function variable_bin_counts(em::EM)
    n_nodes = length(em.statsvec)
    r_arr = Array(Int, n_nodes)
    for i = 1 : n_nodes
        r_arr[i] = size(em.statsvec[i], 1)
    end
    r_arr
end
function counts(em::EM, targetind::Int, parentindeces::Vector{Int}, parentassignments::Vector{Int}, binsizes::Vector{Int})
    
    dims      = tuple([binsizes[parentindeces]]...)
    subs      = tuple(parentassignments...)
    j = sub2ind(dims, subs...)
    em.statsvec[targetind][:,j]
end
function counts(em::EM, targetind::Int, assignments::Dict{Symbol, Int}, binsizes::Vector{Int})
    parentindeces = parent_indeces(targetind, em)
    nparents = length(parentindeces)
    parentassignments = Array(Int, nparents)
    for (i,ind) in enumerate(parentindeces)
        parentassignments[i] = assignments[em.BN.names[ind]]
    end
    counts(em, targetind, parentindeces, parentassignments, binsizes)
end
function counts(em::EM)
    sum(em.statsvec[1])
end

edges(varindex::Int, em::EM) = em.binmaps[varindex].binedges

