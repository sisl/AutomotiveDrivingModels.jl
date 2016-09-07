using BayesNets
using OnlineStats

export ConditionalLinearGaussianDriver

"""
The ordering of the parental instantiations in discrete networks follows the convention
defined in Decision Making Under Uncertainty.

Suppose a variable has three discrete parents. The first parental instantiation
assigns all parents to their first bin. The second will assign the first
parent (as defined in `parents`) to its second bin and the other parents
to their first bin. The sequence continues until all parents are instantiated
to their last bins.

This is a directly copy from Base.sub2ind but allows for passing a vector instead of separate items

Note that this does NOT check bounds
"""
function sub2ind_vec{T<:Integer, S<:Integer}(dims::AbstractVector{T}, I::AbstractVector{S})
    N = length(dims)
    if N == 0
        return 1
    end
    @assert(N == length(I))

    ex = I[N] - 1
    for i in N-1:-1:1
        if i > N
            ex = (I[i] - 1 + ex)
        else
            ex = (I[i] - 1 + dims[i]*ex)
        end
    end

    ex + 1
end

######################################################################
#
#
#
######################################################################


immutable LinearGaussianStats
    # μ = wᵀx where x[end] = 1
    w::Vector{Float64} # [nparents + 1]
    σ::Float64
end
n_sufficient_statistics(s::LinearGaussianStats) = length(s.w) + 1

type ConditionalLinearGaussianNode
    #=
    Defines a variable in a Conditional Linear Gaussian Bayesian Network

    p(x|pa(x)) ~ Normal(wᵀcont + b) ∀ disc

    There is one linear gaussian for each combination of discrete parents
    =#

    index::Int # index of this node in the assignment list
    stats::Matrix{LinearGaussianStats} # [n_nann_parent_instantiations × n_disc_parent_instantiations]
    parents_disc::Vector{Int} # index, in assignment_disc, of discrete parents
    parents_cont::Vector{Int} # index, in assignment_cont, of continuous parents, may include other nodes's vars
    parents_nann::Vector{Int} # index, in assignment_cont, of nannable parents

    parental_assignments_disc::Vector{Int} # discrete assignment, preallocated array for sub2ind calculation
    parental_assignments_nann::Vector{Int} # discrete assignment, preallocated array for sub2ind calculation
    parental_assignments_cont::Vector{Float64} # continuous assignment (last entry is 1.0) [cont | nann]
    parent_instantiation_counts_disc::Vector{Int} # number of possible instantiations for each discrete parent
    parent_instantiation_counts_nann::Vector{Int} # number of possible instantiations for each nannable parent (all 2's)
end

function n_sufficient_statistics(node::ConditionalLinearGaussianNode)
    retval = 0
    for s in node.stats
        retval += n_sufficient_statistics(s)
    end
    retval
end

function _get_normal(
    node::ConditionalLinearGaussianNode,
    assignment_disc::Vector{Int},    # note: full array into which we must index with node.parents_disc
    assignment_cont::Vector{Float64} # note: full array into which we must index with node.parents_cont
    )

    for (i,v) in enumerate(node.parents_nann)
        node.parental_assignments_nann[i] = isnan(assignment_cont[v]) ? 1 : 2
    end
    q = sub2ind_vec(node.parent_instantiation_counts_nann, node.parental_assignments_nann)

    for (i,v) in enumerate(node.parents_disc)
        node.parental_assignments_disc[i] = assignment_disc[v]
    end
    j = sub2ind_vec(node.parent_instantiation_counts_disc, node.parental_assignments_disc)

    stat = node.stats[q,j]
    μ = stat.w[end]

    i = 0
    for p in node.parents_cont
        i += 1
        μ += stat.w[i] * assignment_cont[p]
    end
    for p in node.parents_nann
        if !isnan(assignment_cont[p])
            i += 1
            μ += stat.w[i] * assignment_cont[p]
        end
    end
    @assert(i == length(stat.w)-1)

    normal = Normal(μ, stat.σ)
end

######################################################################
#
#
#
######################################################################

type LinearGaussianBayesianNetBehavior{A} <: DriverModel{A, IntegratedContinuous}

    C::IntegratedContinuous
    rec::SceneRecord

    target_lat::AbstractFeature
    target_lon::AbstractFeature
    features_disc::Vector{AbstractFeature}
    features_cont::Vector{AbstractFeature}

    sample_lat_first::Bool # used to ensure topological ordering is preserved
    node_lat::ConditionalLinearGaussianNode
    node_lon::ConditionalLinearGaussianNode
    assignment_disc::Vector{Int} # [d]
    assignment_cont::Vector{Float64} # [c + 2], preallocated memory, with cont | lat | lon

    # function LinearGaussianBayesianNetBehavior(
    #     C::IntegratedContinuous,
    #     target_lat::AbstractFeature,
    #     target_lon::AbstractFeature,
    #     features_disc::Vector{AbstractFeature},
    #     features_cont::Vector{AbstractFeature},
    #     clamper_cont::DataClamper,
    #     clamper_act::DataClamper,
    #     sample_lat_first::Bool,
    #     node_lat::ConditionalLinearGaussianNode,
    #     node_lon::ConditionalLinearGaussianNode,
    #     )

    #     retval = new()

    #     retval.target_lat = target_lat
    #     retval.target_lon = target_lon
    #     retval.features_disc = features_disc
    #     retval.features_cont = features_cont

    #     retval.sample_lat_first = sample_lat_first
    #     retval.node_lat = node_lat
    #     retval.node_lon = node_lon
    #     retval.assignment_disc = Array(Int, length(retval.extractor_disc.indicators))
    #     retval.assignment_cont = Array(Float64, length(retval.extractor_cont.indicators)+2)

    #     retval
    # end
end

function Base.print(io::IO, LB::LinearGaussianBayesianNetBehavior)

    symbols_disc = convert(Vector{Symbol}, map(f->symbol(f), LB.features_disc))
    symbols_cont = convert(Vector{Symbol},[map(f->symbol(f), LB.features_cont);
                                           symbol(LB.target_lat); symbol(LB.target_lon)])

    println(io, "Linear Gaussian Bayesian Network Behavior")
    println(io, "\ttargets: ", LB.target_lat, "  ", LB.target_lon)
    println(io, "\tfeatures: ")
    println(io, "\t\tdiscrete:   ", symbols_disc)
    println(io, "\t\tcontinuous: ", symbols_cont)
    println(io, "\tnumber of suffstats for lat: ", n_sufficient_statistics(LB.node_lat))
    println(io, "\tnumber of suffstats for lon: ", n_sufficient_statistics(LB.node_lon))
    println(io, "\tparents lat: ", symbols_disc[LB.node_lat.parents_disc])
    println(io, "\t             ", symbols_cont[LB.node_lat.parents_cont])
    println(io, "\t             ", symbols_cont[LB.node_lat.parents_nann])
    println(io, "\tparents lon: ", symbols_disc[LB.node_lon.parents_disc])
    println(io, "\t             ", symbols_cont[LB.node_lon.parents_cont])
    println(io, "\t             ", symbols_cont[LB.node_lon.parents_nann])
    println(io, "\tsample lat first: ", LB.sample_lat_first)
end

get_name(::LinearGaussianBayesianNetBehavior) = "LB"
action_context(LB::LinearGaussianBayesianNetBehavior) = LB.C

type LB_TrainParams <: AbstractVehicleBehaviorTrainParams

    targets::ModelTargets
    indicators::Vector{AbstractFeature} # list of all potential indicators

    ridge_regression_constant::Float64
    min_σ_lat::Float64 # minimum standard deviation for lateral target
    min_σ_lon::Float64 # minimum standard deviation for longitudinal target

    max_parents::Int # maximum number of parents per node
    verbosity::Int

    function LB_TrainParams(;
        targets::ModelTargets = ModelTargets(Features.FUTUREDESIREDANGLE, Features.FUTUREACCELERATION),
        indicators::Vector{AbstractFeature} = AbstractFeature[],
        ridge_regression_constant::Float64=0.001,
        min_σ_lat::Float64=1e-4,
        min_σ_lon::Float64=1e-5,
        max_parents::Int=4,
        verbosity::Int=0,
        )

        @assert(!in(targets.lat, indicators))
        @assert(!in(targets.lon, indicators))

        retval = new()

        retval.ridge_regression_constant = ridge_regression_constant
        retval.targets = targets
        retval.indicators = indicators
        retval.min_σ_lat = min_σ_lat
        retval.min_σ_lon = min_σ_lon

        retval.max_parents = max_parents
        retval.verbosity = verbosity

        retval
    end
end
function Base.print(io::IO, p::LB_TrainParams)
    println(io, "LB Train Params")
    println(io, "\ttargets: ", p.targets)
    println(io, "\tindicators: ", map(f->symbol(f), p.indicators))
    println(io, "\tridge_regression_constant: ", p.ridge_regression_constant)
    println(io, "\tmin_σ_lat:      ", p.min_σ_lat)
    println(io, "\tmin_σ_lon:      ", p.min_σ_lon)
    println(io, "\tmax_parents:    ", p.max_parents)
end

# type LB_PreallocatedData <: AbstractVehicleBehaviorPreallocatedData

#     Y::Matrix{Float64} # [2 × m] # two targets (lat, lon)
#     X_disc::Matrix{Int} # [d × m]
#     X_cont::Matrix{Float64} # [c × m]
#     X_nann::Matrix{Float64} # [n × m]

#     features_disc  :: Vector{AbstractFeature}
#     features_cont  :: Vector{AbstractFeature}
#     features_nann  :: Vector{AbstractFeature}
#     clamper_cont   :: DataClamper
#     clamper_nann   :: DataClamper
#     clamper_act    :: DataClamper

#     function LB_PreallocatedData(dset::ModelTrainingData2, params::LB_TrainParams)

#         retval = new()

#         targets = params.targets
#         indicators = params.indicators
#         trainingframes = dset.dataframe
#         nframes = nrow(trainingframes)
#         nindicators = length(indicators)

#         X = Array(Float64, nindicators, nframes)
#         Y = Array(Float64, 2, nframes)
#         pull_design_and_target_matrices!(X, Y, trainingframes, targets, indicators)

#         ###########################

#         features_disc_indeces = find(f->isint(f), indicators)
#         features_cont_indeces = find(f->!isint(f) && !couldna(f), indicators)
#         features_nann_indeces = find(f->!isint(f) &&  couldna(f), indicators)

#         retval.Y = Y
#         retval.X_disc = round(Int, X[features_disc_indeces, :]) .+ 1
#         retval.X_cont = X[features_cont_indeces, :]
#         retval.X_nann = X[features_nann_indeces, :]
#         retval.features_disc = indicators[features_disc_indeces]
#         retval.features_cont = indicators[features_cont_indeces]
#         retval.features_nann = indicators[features_nann_indeces]

#         ###########################

#         retval.clamper_cont = DataClamper(retval.X_cont)
#         retval.clamper_nann = DataClamper(retval.X_nann)
#         retval.clamper_act = DataClamper(retval.Y)

#         retval
#     end
# end
# function preallocate_learning_data(
#     dset::ModelTrainingData2,
#     params::LB_TrainParams)

#     LB_PreallocatedData(dset, params)
# end

function _sample_from_node!(LB::LinearGaussianBayesianNetBehavior, node::ConditionalLinearGaussianNode)
    normal = _get_normal(node, LB.assignment_disc, LB.assignment_cont)
    action = rand(normal)
    LB.assignment_cont[node.index] = action
    action
end
function _set_and_process_action!(LB::LinearGaussianBayesianNetBehavior, action_lat::Float64, action_lon::Float64)
    LB.clamper_act.x[1] = action_lat
    LB.clamper_act.x[2] = action_lon
    process!(LB.clamper_act)
    LB.assignment_cont[LB.node_lat.index] = LB.clamper_act.x[1]
    LB.assignment_cont[LB.node_lon.index] = LB.clamper_act.x[2]
    LB
end

function observe!(LB::LinearGaussianBayesianNetBehavior, scene::Scene, roadway::Roadway, egoid::Int)

    vehicle_index = get_index_of_first_vehicle_with_id(scene, egoid)
    update!(LB.rec, scene)

    # observe cont
    for (i,f) in enumerate(LB.features_cont)
        v = get(f, LB.rec, roadway, vehicle_index)
        LB.assignment_cont[i] = is_feature_valid(v) ? v.v : NaN
    end

    # observe disc
    for (i,f) in enumerate(LB.features_disc)
        v = get(f, LB.rec, roadway, vehicle_index)
        # compensate for fact that discrete features (such as booleans) start at 0
        LB.assignment_disc[i] = round(Int, convert(Float64, v)) + 1
    end

    LB
end
function Base.rand{A}(LB::LinearGaussianBayesianNetBehavior{A})


    if LB.sample_lat_first
        _sample_from_node!(LB, LB.node_lat)
        _sample_from_node!(LB, LB.node_lon)
    else
        _sample_from_node!(LB, LB.node_lon)
        _sample_from_node!(LB, LB.node_lat)
    end

    action_lat = LB.assignment_cont[end-1]
    action_lon = LB.assignment_cont[end]

    convert(A, [action_lat, action_lon])
end
function Distributions.pdf{A}(LB::LinearGaussianBayesianNetBehavior{A}, a::A)

    v = convert(Vector{Float64}, a)
    LB.assignment_cont[end-1] = v[1]
    LB.assignment_cont[end] = v[2]

    logl = 1.0
    normal = _get_normal(LB.node_lat, LB.assignment_disc, LB.assignment_cont)
    logl *= pdf(normal, v[1])

    normal = _get_normal(LB.node_lon, LB.assignment_disc, LB.assignment_cont)
    logl *= pdf(normal, v[2])

    logl
end
function Distributions.logpdf{A}(LB::LinearGaussianBayesianNetBehavior{A}, a::A)

    v = convert(Vector{Float64}, a)
    LB.assignment_cont[end-1] = v[1]
    LB.assignment_cont[end] = v[2]

    logl = 0.0
    normal = _get_normal(LB.node_lat, LB.assignment_disc, LB.assignment_cont)
    logl += logpdf(normal, v[1])

    normal = _get_normal(LB.node_lon, LB.assignment_disc, LB.assignment_cont)
    logl += logpdf(normal, v[2])

    logl
end

######################################################################
#
#
#
######################################################################

immutable NodeInTraining
    index::Int # index of this node in Y
    target_as_parent::Int # if != 0, index of other target in Y
    parents_disc::Vector{Int} # index, in X_disc, of discrete parents (always in sorted order)
    parents_cont::Vector{Int} # index, in X_cont, of continuous parents (always in sorted order)
    parents_nann::Vector{Int} # index, in X_nann, of NaNable parents

    NodeInTraining(index::Int) = new(index, 0, Int[], Int[], Int[])
    NodeInTraining(i::Int, t_as_p::Int, pdisc::Vector{Int}, pcont::Vector{Int}, pnann::Vector{Int}) = new(i, t_as_p, pdisc, pcont, pnann)
end
function Base.copy(node::NodeInTraining)
    NodeInTraining(node.index, node.target_as_parent,
                   copy(node.parents_disc), copy(node.parents_cont),
                   copy(node.parents_nann))
end
function set_new_parent_disc(node::NodeInTraining, parent::Int)
    parents_disc = sort!(unique(push!(copy(node.parents_disc), parent)))
    NodeInTraining(node.index, node.target_as_parent, parents_disc, copy(node.parents_cont), copy(node.parents_nann))
end
function set_new_parent_cont(node::NodeInTraining, parent::Int)
    parents_cont = sort!(unique(push!(copy(node.parents_cont), parent)))
    NodeInTraining(node.index, node.target_as_parent, copy(node.parents_disc), parents_cont, copy(node.parents_nann))
end
function set_new_parent_nann(node::NodeInTraining, parent::Int)
    parents_nann = sort!(unique(push!(copy(node.parents_nann), parent)))
    NodeInTraining(node.index, node.target_as_parent, copy(node.parents_disc), copy(node.parents_cont), parents_nann)
end
function set_new_parent_targ(node::NodeInTraining, parent::Int)
    @assert(node.target_as_parent == 0)
    NodeInTraining(node.index, parent, copy(node.parents_disc), copy(node.parents_cont), copy(node.parents_nann))
end
function set_new_parent(node::NodeInTraining, parent::Int, sym::Symbol)
    if sym == :disc
        set_new_parent_disc(node, parent)
    elseif sym == :cont
        set_new_parent_cont(node, parent)
    elseif sym == :nann
        set_new_parent_nann(node, parent)
    elseif sym == :targ
        set_new_parent_targ(node, parent)
    else
        error("unknown parent set type $sym")
    end
end
function Base.hash(node::NodeInTraining, h::UInt=one(UInt))
    hash(node.index,
        hash(node.parents_disc,
            hash(node.parents_cont,
                hash(node.target_as_parent,
                    hash(node.parents_nann, h)))))
end
function Base.(:(==))(A::NodeInTraining, B::NodeInTraining)
    A.index            == B.index &&
    A.target_as_parent == B.target_as_parent &&
    A.parents_disc     == B.parents_disc &&
    A.parents_cont     == B.parents_cont &&
    A.parents_nann     == B.parents_nann
end
function Base.show(io::IO, node::NodeInTraining)
    println("Node In Training:")
    println("\tindex:            ", node.index)
    println("\ttarget_as_parent: ", node.target_as_parent)
    println("\tparents_disc:     ", node.parents_disc)
    println("\tparents_cont:     ", node.parents_cont)
    println("\tparents_nann:     ", node.parents_nann)
end
nparents_total(node::NodeInTraining) = length(node.parents_disc) + length(node.parents_cont) + length(node.parents_nann)

type TrainingData

    Y :: Matrix{Float64}
    X_disc :: Matrix{Float64}
    X_cont :: Matrix{Float64}
    X_nann :: Matrix{Float64}

    disc_parent_instantiations::Vector{Int}
    score_cache::Dict{NodeInTraining, Float64} # node -> (component_score)

    # function TrainingData(
    #     training_data::ModelTrainingData2,
    #     preallocated_data::LB_PreallocatedData,
    #     foldset::FoldSet,
    #     )

    #     retval = new()

    #     retval.Y = copy_matrix_fold(preallocated_data.Y, foldset)
    #     retval.X_disc = copy_matrix_fold(preallocated_data.X_disc, foldset)::Matrix{Int}
    #     retval.X_cont = copy_matrix_fold(preallocated_data.X_cont, foldset)::Matrix{Float64}
    #     retval.X_nann = copy_matrix_fold(preallocated_data.X_nann, foldset)::Matrix{Float64}

    #     retval.disc_parent_instantiations = convert(Vector{Int}, map(f->round(Int, upperbound(f))+1, preallocated_data.features_disc))
    #     retval.score_cache = Dict{NodeInTraining, Float64}()

    #     retval
    # end
end

nsamples(data) = size(data.Y, 1)

function _init_parent_stuff_nann(node::NodeInTraining)
    nparents_nann = length(node.parents_nann) + (node.target_as_parent != 0)

    # 1 -> all being nan, 2 -> first being cont, rest all nan, 3 -> 2nd cont, 4->1st 2 cont, etc.
    # last one is the optional target as parent
    parent_instantiation_counts_nann = fill(2, nparents_nann)

    # returns 1 if nparents_nann=0, which is what we want
    n_nann_parent_instantiations = prod(parent_instantiation_counts_nann)

    # NaN -> 1, cont -> 2
    parental_assignments_nann = Array(Int, nparents_nann)

    (parent_instantiation_counts_nann, n_nann_parent_instantiations, parental_assignments_nann)
end
function _init_parent_stuff_disc(node::NodeInTraining, data::TrainingData)

    nparents_disc = length(node.parents_disc)
    parent_instantiation_counts_disc = Array(Int, nparents_disc)
    n_disc_parent_instantiations = 1
    for (i,p) in enumerate(node.parents_disc)
        parent_instantiation_counts_disc[i] = data.disc_parent_instantiations[p]
        n_disc_parent_instantiations *= data.disc_parent_instantiations[p]
    end

    parental_assignments_disc = Array(Int, nparents_disc)

    (parent_instantiation_counts_disc, n_disc_parent_instantiations, parental_assignments_disc)
end

function _get_nann_assignment_index!(
    i::Int,
    node::NodeInTraining, # the indeces of the discrete parents
    data::TrainingData,
    parental_assignments_nann::Vector{Int}, # preallocated vector to hold the parental assignments
    parent_instantiation_counts_nann::Vector{Int} # all twos
    )

    for (j,p) in enumerate(node.parents_nann)
        parental_assignments_nann[j] = isnan(data.X_nann[i,p]) ? 1 : 2 # if NAN, then 1, if continuous, then 2
    end
    if node.target_as_parent != 0
        parental_assignments_nann[end] = isnan(data.Y[i,node.target_as_parent]) ? 1 : 2
    end

    sub2ind_vec(parent_instantiation_counts_nann, parental_assignments_nann)
end
function _get_disc_assignment_index!(
    i::Int,
    node::NodeInTraining, # the indeces of the discrete parents
    data::TrainingData,
    parental_assignments_disc::Vector{Int}, # preallocated vector to hold the parental assignments
    parent_instantiation_counts_disc::Vector{Int} # size of each parent's domain
    )

    for (j,p) in enumerate(node.parents_disc)
        parental_assignments_disc[j] = data.X_disc[i,p]
    end
    sub2ind_vec(parent_instantiation_counts_disc, parental_assignments_disc)
end
function _pull_cont_data!(i::Int, x::Vector{Float64}, node::NodeInTraining, data::TrainingData)

    y = data.Y[i, node.index]

    j = 0
    for p in node.parents_cont
        j += 1
        x[j] = data.X_cont[i,p]
    end
    for p in node.parents_nann
        v = data.X_nann[i,p]
        if !isnan(v)
            j += 1
            x[j] = v
        end
    end
    if node.target_as_parent != 0
        v = data.Y[i,node.target_as_parent]
        if !isnan(v)
            j += 1
            x[j] = v
        end
    end

    if j != length(x)-1
        println("node")
        println(node)
        println("x: ", x)
        println("j: ", j)
        println("expected: ", length(x)-1)
        println("target as parent: ", node.target_as_parent)
        sleep(0.5)
    end

    @assert(j == length(x)-1)

    y
end
function _update_ridge_regression!(
    lhs::Vector{Float64},
    rhs::Matrix{Float64},
    x::Vector{Float64},
    y::Float64
    )

    lenx = length(x)

    for i in 1 : lenx
        lhs[i] += x[i]*y

        for j in 1 : lenx
            rhs[i,j] += x[i]*x[j]
        end
    end

    nothing
end
function _calc_logl_contribution(w::Vector{Float64}, σ::Float64, x::Vector{Float64}, y::Float64)
    μ = dot(w, x)
    logpdf(Normal(μ, σ), y)
end

function _perform_ridge_regression(
    node::NodeInTraining,
    data::TrainingData,
    params::LB_TrainParams,
    parent_instantiation_counts_nann::Vector{Int},
    n_nann_parent_instantiations::Int,
    parental_assignments_nann::Vector{Int},
    parent_instantiation_counts_disc::Vector{Int},
    n_disc_parent_instantiations::Int,
    parental_assignments_disc::Vector{Int},
    )

    # perform ridge-regression and stdev calc online
    #   Xᵀy = (λI + XᵀX)w

    # LHS is the Xᵀy vector [nparents+1]
    # RHS is the λI + XᵀX matrix [nparents+1 × nparents+1]

    λ = params.ridge_regression_constant
    σ_min = node.index == 1 ? params.min_σ_lat : params.min_σ_lon

    nparents_disc = length(node.parents_disc)
    nparents_cont = length(node.parents_cont)
    nparents_nann = length(node.parents_nann)

    x_arr     = Array(Vector{Float64}, n_nann_parent_instantiations)
    LHS_arr   = Array(Vector{Float64}, n_nann_parent_instantiations, n_disc_parent_instantiations)
    RHS_arr   = Array(Matrix{Float64}, n_nann_parent_instantiations, n_disc_parent_instantiations)
    σ_var_arr = Array(Variance,        n_nann_parent_instantiations, n_disc_parent_instantiations)

    # dim_node = 0
    for n in 1 : n_nann_parent_instantiations
        n_nann_cont_parents = count_ones(n-1)

        x_len = nparents_cont + n_nann_cont_parents + 1 # one for the bias value
                                                        # note that target_as_parent is already in n_nann_cont_parents

        # x: [cont | nann | target_as_parent | bias]
        x_arr[n] = Array(Float64, x_len)
        x_arr[n][end] = 1.0 # set bias to 1.0
        # dim_node += n_disc_parent_instantiations * (x_len + 1) # one for the standard deviation

        for i in 1 : n_disc_parent_instantiations
            LHS_arr[n, i] = zeros(Float64, x_len)
            RHS_arr[n, i] = diagm(fill(λ, x_len))
            σ_var_arr[n, i] = Variance()
        end
    end

    # perform learning pass through data
    for i in 1 : nsamples(data)
        q = _get_nann_assignment_index!(i, node, data, parental_assignments_nann, parent_instantiation_counts_nann)
        x = x_arr[q]
        y = _pull_cont_data!(i, x, node, data)
        k = _get_disc_assignment_index!(i, node, data, parental_assignments_disc, parent_instantiation_counts_disc)
        _update_ridge_regression!(LHS_arr[q,k], RHS_arr[q,k], x, y)
    end

    # solve ridge regressions
    #  w = (λI + XᵀX)⁻¹ Xᵀy

    w_arr = Array(Vector{Float64}, n_nann_parent_instantiations, n_disc_parent_instantiations)
    for q in 1 : n_nann_parent_instantiations
        n_nann_cont_parents = count_ones(q-1)
        w_len = n_nann_cont_parents + nparents_cont + 1 # one for the 1.0 value

        for k in 1 : n_disc_parent_instantiations
            try
                w_arr[q,k] = RHS_arr[q,k] \ LHS_arr[q,k]
                @assert(findfirst(v->isnan(v), w_arr[q,k]) == 0)
            catch
                w_arr[q,k] = zeros(w_len)
            end

            @assert(findfirst(v->isnan(v), w_arr[q,k]) == 0)
        end
    end

    # perform σ fit pass
    for i in 1 : nsamples(data)
        q = _get_nann_assignment_index!(i, node, data, parental_assignments_nann, parent_instantiation_counts_nann)
        x = x_arr[q]
        y = _pull_cont_data!(i, x, node, data)
        k = _get_disc_assignment_index!(i, node, data, parental_assignments_disc, parent_instantiation_counts_disc)

        w = w_arr[q, k]
        y_pred = dot(w, x)
        fit!(σ_var_arr[q,k], y - y_pred)
    end

    # back out σ
    σ_arr = Array(Float64, n_nann_parent_instantiations, n_disc_parent_instantiations)
    for q in 1 : n_nann_parent_instantiations
        for k in 1 : n_disc_parent_instantiations
            σ_var = σ_var_arr[q,k]
            if StatsBase.nobs(σ_var) > 1
                σ_arr[q, k] = max(std(σ_var), σ_min)
            else
                σ_arr[q, k] = σ_min
            end
        end
    end

    (w_arr, σ_arr, x_arr)
end
function _get_component_score(node::NodeInTraining, data::TrainingData, params::LB_TrainParams)

    # Using BIC score
    # see: “Ideal Parent” Structure Learning for Continuous Variable Bayesian Networks
    #
    # BIC = max_θ  l(D|G, θ) - log(m)/2 * dim(G)
    #
    #   where max_θ  l(D|G, θ) is found using ridge regression
    #         and dim(G) is the number of parameters in G
    #
    #  Calling this function causes it to return:
    #    max_θ  l(D|G, θ) - log(m)/2*dim(node)
    #
    #   which can be added to other components to get the full BIC score

    m = nsamples(data)
    @assert(size(data.X_disc,1) == m)
    @assert(size(data.X_cont,1) == m)
    @assert(size(data.X_nann,1) == m)

    parent_instantiation_counts_nann, n_nann_parent_instantiations, parental_assignments_nann = _init_parent_stuff_nann(node)
    parent_instantiation_counts_disc, n_disc_parent_instantiations, parental_assignments_disc = _init_parent_stuff_disc(node, data)

    w_arr, σ_arr, x_arr = _perform_ridge_regression(node, data, params,
                            parent_instantiation_counts_nann, n_nann_parent_instantiations, parental_assignments_nann,
                            parent_instantiation_counts_disc, n_disc_parent_instantiations, parental_assignments_disc)

    # compute the log likelihood
    logl = 0.0
    for i in 1 : m
        q = _get_nann_assignment_index!(i, node, data, parental_assignments_nann, parent_instantiation_counts_nann)
        x = x_arr[q]
        y = _pull_cont_data!(i, x, node, data)
        k = _get_disc_assignment_index!(i, node, data, parental_assignments_disc, parent_instantiation_counts_disc)
        logl += _calc_logl_contribution(w_arr[q,k], σ_arr[q,k], x, y)
    end

    n_suff_stats = 0
    for w in w_arr
        n_suff_stats += length(w) + 1
    end

    logl - log(m)*n_suff_stats/2
end
function _get_component_score!(node::NodeInTraining, data::TrainingData, params::LB_TrainParams)

    if !haskey(data.score_cache, node)
        data.score_cache[node] = _get_component_score(node, data, params)
    end
    data.score_cache[node]
end
function _greedy_hillclimb_iter_on_node(node::NodeInTraining, data::TrainingData, params::LB_TrainParams, other_node_target_as_parent::Int)

    if params.verbosity > 1
        println("_greedy_hillclimb_iter_on_node: ")
        println(node)
    end

    start_score = data.score_cache[node]
    best_score = start_score
    parent_to_add = -1
    add_sym = :none

    n_indicators_cont = size(data.X_cont, 2)

    if nparents_total(node) ≥ params.max_parents
        if params.verbosity > 1
            println("skipping due to max_parents = $(params.max_parents) reached")
        end
        return (-1.0, parent_to_add, add_sym)
    end

    # try adding discrete edges
    for p in 1 : size(data.X_disc, 2)
        new_score = _get_component_score!(set_new_parent_disc(node, p), data, params)

        if params.verbosity > 1
            @printf("trying disc %5d  -> Δscore = %12.6f\n", p, new_score - start_score)
        end

        if new_score > best_score
            best_score = new_score
            parent_to_add = p
            add_sym = :disc
        end
    end

    # try adding continuous edges
    for p in 1 : n_indicators_cont
        new_score = _get_component_score!(set_new_parent_cont(node, p), data, params)

        if params.verbosity > 1
            @printf("trying cont %5d  -> Δscore = %12.6f\n", p, new_score - start_score)
        end

        if new_score > best_score
            best_score = new_score
            parent_to_add = p
            add_sym = :cont
        end
    end

    # try adding nannable edges
    for p in 1 : size(data.X_nann, 2)
        new_score = _get_component_score!(set_new_parent_nann(node, p), data, params)

        if params.verbosity > 1
            @printf("trying nann %5d  -> Δscore = %12.6f\n", p, new_score - start_score)
        end

        if new_score > best_score
            best_score = new_score
            parent_to_add = p
            add_sym = :nann
        end
    end

    # try adding other targets as parents
    if node.target_as_parent == 0 && other_node_target_as_parent == 0
        p = node.index == 1 ? 2 : 1

        new_score = _get_component_score!(set_new_parent_targ(node, p), data, params)

        if params.verbosity > 1
            @printf("trying targ %5d  -> Δscore = %12.6f\n", p, new_score - start_score)
        end

        if new_score > best_score
            best_score = new_score
            parent_to_add = p
            add_sym = :targ
        end
    end

    Δscore = best_score - start_score

    (Δscore, parent_to_add, add_sym)
end
function _build_linear_gaussian_node(
    node::NodeInTraining,
    data::TrainingData,
    params::LB_TrainParams,
    ind_old_to_new_disc::Dict{Int, Int},
    ind_old_to_new_cont::Dict{Int, Int},
    ind_old_to_new_nann::Dict{Int, Int},
    )

    index = length(ind_old_to_new_cont) + length(ind_old_to_new_nann) + node.index

    nparents_disc = length(node.parents_disc)
    parents_disc = Array(Int, nparents_disc)
    for (i,p) in enumerate(node.parents_disc)
        parents_disc[i] = ind_old_to_new_disc[p]
    end

    nparents_cont = length(node.parents_cont)
    parents_cont = Array(Int, nparents_cont)
    for (i,p) in enumerate(node.parents_cont)
        parents_cont[i] = ind_old_to_new_cont[p]
    end

    nparents_nann = length(node.parents_nann)
    parents_nann = Array(Int, nparents_nann)
    for (i,p) in enumerate(node.parents_nann)
        parents_nann[i] = ind_old_to_new_nann[p] + length(ind_old_to_new_cont)
    end
    if node.target_as_parent != 0
        nparents_nann += 1
        push!(parents_nann, node.target_as_parent+length(ind_old_to_new_cont)+length(ind_old_to_new_nann))
    end

    m = nsamples(data)
    @assert(size(data.X_disc,1) == m)
    @assert(size(data.X_cont,1) == m)
    @assert(size(data.X_nann,1) == m)

    parental_assignments_disc = Array(Int, nparents_disc)
    parental_assignments_cont = Array(Float64, nparents_cont+nparents_nann+1)

    parent_instantiation_counts_nann, n_nann_parent_instantiations, parental_assignments_nann = _init_parent_stuff_nann(node)
    parent_instantiation_counts_disc, n_disc_parent_instantiations, parental_assignments_disc = _init_parent_stuff_disc(node, data)

    w_arr, σ_arr, x_arr = _perform_ridge_regression(node, data, params,
                            parent_instantiation_counts_nann, n_nann_parent_instantiations, parental_assignments_nann,
                            parent_instantiation_counts_disc, n_disc_parent_instantiations, parental_assignments_disc)

    stats = Array(LinearGaussianStats, n_nann_parent_instantiations, n_disc_parent_instantiations)

    for n in 1 : n_nann_parent_instantiations
        for i in 1 : n_disc_parent_instantiations

            w = w_arr[n,i]
            σ = σ_arr[n,i]
            @assert(!isnan(σ))
            stats[n,i] = LinearGaussianStats(w, σ)
        end
    end

    ConditionalLinearGaussianNode(index, stats,
                       parents_disc, parents_cont, parents_nann,
                       parental_assignments_disc,
                       parental_assignments_nann,
                       parental_assignments_cont,
                       parent_instantiation_counts_disc,
                       parent_instantiation_counts_nann,
                    )
end

function train(
    training_data::ModelTrainingData2,
    preallocated_data::LB_PreallocatedData,
    params::LB_TrainParams,
    foldset::FoldSet,
    )

    @assert(findfirst(v->isnan(v), preallocated_data.Y) == 0)
    @assert(findfirst(v->isinf(v), preallocated_data.Y) == 0)

    # -----------------------------------------
    # run structure learning
    #  - always add the next best edge

    node_lat = NodeInTraining(1)
    node_lon = NodeInTraining(2)

    data = TrainingData(training_data, preallocated_data, foldset)

    _get_component_score!(node_lat, data, params)
    _get_component_score!(node_lon, data, params)

    finished = false
    while !finished

        Δscore_lat, parent_to_add_lat, add_sym_lat = _greedy_hillclimb_iter_on_node(node_lat, data, params, node_lon.target_as_parent)
        Δscore_lon, parent_to_add_lon, add_sym_lon = _greedy_hillclimb_iter_on_node(node_lon, data, params, node_lat.target_as_parent)

        if max(Δscore_lat, Δscore_lon) < 0.001
            # no improvement
            finished = true
        else
            if Δscore_lat > Δscore_lon

                node_lat = set_new_parent(node_lat, parent_to_add_lat, add_sym_lat)
                @assert(haskey(data.score_cache, node_lat))
                if params.verbosity > 0
                    println("\n===========================================")
                    @printf("adding to lat: %4d   %4s\n", parent_to_add_lat, string(add_sym_lat))
                    println(node_lat)
                    println("===========================================\n")
                end
            else
                node_lon = set_new_parent(node_lon, parent_to_add_lon, add_sym_lon)
                @assert(haskey(data.score_cache, node_lon))
                if params.verbosity > 0
                    println("\n===========================================")
                    @printf("adding to lon: %4d   %4s\n", parent_to_add_lon, string(add_sym_lon))
                    println(node_lon)
                    println("===========================================\n")
                end
            end
        end
    end

    # -----------------------------------------
    # now build the model

    ind_old_disc = sort!(unique([node_lat.parents_disc; node_lon.parents_disc]))
    extractor_disc = FeatureSubsetExtractor(preallocated_data.features_disc[ind_old_disc])
    ind_old_to_new_disc = Dict{Int,Int}()
    for (o,n) in enumerate(ind_old_disc)
        ind_old_to_new_disc[n] = o
    end

    ind_old_cont = sort!(unique([node_lat.parents_cont; node_lon.parents_cont]))
    ind_old_to_new_cont = Dict{Int,Int}()
    for (o,n) in enumerate(ind_old_cont)
        ind_old_to_new_cont[n] = o
    end

    # println("ind_old_cont: ", ind_old_cont)

    ind_old_nann = sort!(unique([node_lat.parents_nann; node_lon.parents_nann]))
    ind_old_to_new_nann = Dict{Int,Int}()
    for (n,o) in enumerate(ind_old_nann)
        ind_old_to_new_nann[o] = n
    end

    # println("ind_old_nann: ", ind_old_nann)

    extractor_cont = FeatureSubsetExtractor([preallocated_data.features_cont[ind_old_cont];
                                             preallocated_data.features_nann[ind_old_nann]])
    clamper_cont = DataClamper(extractor_cont.x,
                               [preallocated_data.clamper_cont.f_lo[ind_old_cont];
                                preallocated_data.clamper_nann.f_lo[ind_old_nann]],
                               [preallocated_data.clamper_cont.f_hi[ind_old_cont];
                                preallocated_data.clamper_nann.f_hi[ind_old_nann]])

    # println("extractor indicators: ", map(f->symbol(f), extractor_cont.indicators))
    # println("clamper lo: ", clamper_cont.f_lo)

    model_node_lat = _build_linear_gaussian_node(node_lat, data, params,
                                                 ind_old_to_new_disc, ind_old_to_new_cont, ind_old_to_new_nann)
    model_node_lon = _build_linear_gaussian_node(node_lon, data, params,
                                                 ind_old_to_new_disc, ind_old_to_new_cont, ind_old_to_new_nann)

    if params.verbosity > 0
        println("final node lat: ")
        println(model_node_lat)
        println("\n")
        println("final node lon: ")
        println(model_node_lon)
    end

    sample_lat_first = (node_lon.target_as_parent == 0)

    LinearGaussianBayesianNetBehavior(
        params.targets, extractor_disc, extractor_cont,
        clamper_cont, preallocated_data.clamper_act,
        sample_lat_first, model_node_lat, model_node_lon
        )
end