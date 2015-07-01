export 
        EM,                   # type representing probabilistic action model

        encounter_model,      # constructor
        build_bn,             # constructs BayesNet from EM
        get_targets,          # get Vector{AbstractFeature} of targets in EM
        get_indicators,       # get Vector{AbstractFeature} of indicators in EM
        get_target_lat,       # get lateral target, AbstractFeature
        get_target_lon,       # get longitudinal target, AbstractFeature
        get_indicators_lat,   # get Vector{AbstractFeature} of parents of lateral target
        get_indicators_lon,   # get Vector{AbstractFeature} of parents of longitudinal target

        index,                # get index of feature or symbol in EM
        parent_indeces,       # get indeces of parents of the given variable in EM
        is_parent,            # whether the given parent->child edge is in the net
        
        variable_bin_counts,  # get Vector{Int} of number of instantiations of each variable
        marginal_probability, # get Vector{Float64} if the marginal probability of the variable
        counts,               # get value related to counts depending on inputs
        edges                 # get bin edges associated with variable

immutable EM
    BN       :: BayesNet
    statsvec :: Vector{Matrix{Float64}}
    features :: Vector{AbstractFeature}
    binmaps  :: Vector{AbstractDiscretizer}
    istarget :: BitVector # whether a feature is a target feature
end
function encounter_model{A<:AbstractDiscretizer, B<:AbstractFeature, C<:AbstractFeature, R<:Real}( 
    BN         :: BayesNet,
    statsvec   :: Vector{Matrix{R}},
    binmapdict :: Dict{Symbol, A}, 
    targets    :: Vector{B}, 
    indicators :: Vector{C}
    )

    features = AbstractFeature[symbol2feature(sym) for sym in BN.names]
    binmaps  = AbstractDiscretizer[binmapdict[sym] for sym in BN.names]
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
function encounter_model{A<:AbstractDiscretizer, B<:AbstractFeature, C<:AbstractFeature, R<:Real}( 
    BN         :: BayesNet,
    statsvec   :: Vector{Matrix{R}},
    binmaps    :: Vector{A}, 
    targets    :: Vector{B}, 
    indicators :: Vector{C}
    )

    features = AbstractFeature[symbol2feature(sym) for sym in BN.names]
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

    bnnames = [symbol(f)::Symbol for f in [targets, indicators]]
    n_nodes = length(bnnames)
    @assert(n_nodes == size(adj, 1) == size(adj, 2))

    BN = BayesNet(bnnames)

    r_arr = Array(Int, n_nodes)
    for (node,node_sym) in enumerate(bnnames)
        stats = statsvec[node]
        r, q = size(stats) # r = num node instantiations, q = num parental instantiations
        states = [1:r]
        BN.domains[node] = DiscreteDomain(states)
        r_arr[node] = r
    end

    for (node,node_sym) in enumerate(bnnames)

        stats = statsvec[node]
        r, q = size(stats)
        states = [1:r]

        stats .+= 1 # NOTE(tim): adding uniform prior
        probs = stats ./ sum(stats,1)

        # set any parents & populate probability table
        n_parents = sum(adj[:,node])
        if n_parents > 0
            bnparents = bnnames[adj[:,node]]
            for pa in bnparents
                addEdge!(BN, pa, node_sym)
            end

            # populate probability table
            assignments = BayesNets.assignment_dicts(BN, bnparents)
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
function get_indicators_lat(em::EM, target_lat::AbstractFeature = get_target_lat(em))
    i = index(target_lat, em)
    indicator_indeces = parent_indeces(i, em)
    em.features[indicator_indeces]
end
function get_indicators_lon(em::EM, target_lon::AbstractFeature = get_target_lon(em))
    i = index(target_lon, em)
    indicator_indeces = parent_indeces(i, em)
    em.features[indicator_indeces]
end
function is_parent(em::EM, parent::Symbol, child::Symbol)
    index_parent = em.BN.index[parent]
    index_child = em.BN.index[child]
    is_parent(em, index_parent, index_child)
end
function is_parent(em::EM, parent::Int, child::Int)
    
    in(parent, in_neighbors(child, em.BN.dag))
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
function marginal_probability(varindex::Int, em::EM)
    binprobs = vec(sum(em.statsvec[varindex], 2))
    binprobs ./= sum(binprobs)
end
function counts(em::EM, targetind::Int, parentindeces::Vector{Int}, parentassignments::Vector{Int}, binsizes::Vector{Int})
    
    dims = tuple([binsizes[parentindeces]]...)
    subs = tuple(parentassignments...)
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