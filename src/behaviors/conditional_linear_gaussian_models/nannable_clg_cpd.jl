"""
A conditional linear Gaussian CPD, always returns a Normal{Float64}

    This is an extension of the ConditionalLinearGaussianCPD,
    For a variable with N discrete parents and M continuous parents, it will construct
    a linear gaussian distribution for all M parents for each discrete instantiation.

                      { Normal(μ=a₁×continuous_parents(x) + b₁, σ₁) for discrete instantiation 1
    P(x|parents(x)) = { Normal(μ=a₂×continuous_parents(x) + b₂, σ₂) for discrete instantiation 2
                      { ...

    The difference is that some parents can be NaN. If this is the case, a different split is used.
    IE, it can be thought of as a ConditionalLinearGaussianCPD except each nannable feature
    is replaced by two features - one which indicates whether it is missing and another which is the
    original feature, just set to zero when it is missing.
"""
type NannableConditionalLinearGaussianCPD <: CPD{Normal{Float64}}
    target::NodeName
    parents::Vector{NodeName} # list of all parents

    parents_nannable::Vector{NodeName} # list of nannable parents
    clgs::Vector{ConditionalLinearGaussianCPD} # set of conditional linear gaussian CPDs, one for each parents_nannable combination
end

name(cpd::NannableConditionalLinearGaussianCPD) = cpd.target
parents(cpd::NannableConditionalLinearGaussianCPD) = cpd.parents
nparams(cpd::NannableConditionalLinearGaussianCPD) = sum(d->nparams(d), cpd.clgs)
# @define_call NannableConditionalLinearGaussianCPD
@compat (cpd::NannableConditionalLinearGaussianCPD)() = (cpd)(Dict{Symbol, Any}()) # cpd()
@compat (cpd::NannableConditionalLinearGaussianCPD)(pair::Pair{Symbol}...) = (cpd)(Dict{Symbol, Any}(pair)) # cpd(:A=>1)

@compat function (cpd::NannableConditionalLinearGaussianCPD)(a::Dict{Symbol,Any})

    idx = 1
    if !isempty(cpd.parents_nannable)

        # get the index in cpd.distributions
        N = length(cpd.parents_nannable)
        idx = isnan(a[cpd.parents_nannable[N]])
        for i in N-1:-1:1
            idx = (isnan(a[cpd.parents_nannable[i]]) + 2*idx)
        end
        idx += 1
    end

    clg = cpd.clgs[idx]
    clg(a)
end

function Distributions.fit(::Type{NannableConditionalLinearGaussianCPD}, data::DataFrame, target::NodeName)

    arr = data[target]
    eltype(arr) <: Real || error("fit ConditionalLinearGaussianCPD requrires target to be numeric")

    clg = fit(ConditionalLinearGaussianCPD, data, target)
    NannableConditionalLinearGaussianCPD(target, NodeName[], NodeName[], [clg])
end
function Distributions.fit(::Type{NannableConditionalLinearGaussianCPD},
    data::DataFrame,
    target::NodeName,
    parents::Vector{NodeName};
    parents_nannable = parents[find(c->eltype(data[c])<:Float64 && any(v->isnan(v),data[c]), 1:ncol(data))],
    )

    if isempty(parents)
        return fit(NannableConditionalLinearGaussianCPD, data, target)
    end

    nparents_nann = length(parents_nannable)

    if nparents_nann != 0

        # ---------------------
        # fit linear gaussians

        dims = fill(2, length(nparents_nann))
        clgs = Array(ConditionalLinearGaussianCPD, 2^nparents_nann)
        for (q, parent_instantiation) in enumerate(product(dims...))
            indeces = Int[]
            non_nan_parents = parents[parent_instantiation .== 1]

            for i in 1 : nrow(data)
                is_match = true
                for j in 1 : nparents_nann
                    if parent_instantiation[j] == 2 # NaN
                        deleteat!(parent_subset, findfirst(parent_subset, ))
                    end

                    if !(
                          (parent_instantiation[j] == 1 && !isnan(data[i, parents_nannable[j]])) ||
                          (parent_instantiation[j] == 2 &&  isnan(data[i, parents_nannable[j]]))
                        )

                        is_match = false
                        break
                    end
                end

                if is_match # parental instantiation matches
                    push!(indeces, i)
                end
            end
            clgs[q] = fit(ConditionalLinearGaussianCPD, data[indeces, :], target, non_nan_parents)
        end
        NannableConditionalLinearGaussianCPD(target, parents, parents_nannable, clgs)

    else # no nannable parents
        clg = fit(ConditionalLinearGaussianCPD, data, target, parents)
        NannableConditionalLinearGaussianCPD(target, parents, Symbol[], [clg])
    end
end

function Distributions.fit(::Type{NannableConditionalLinearGaussianCPD}, data::DataFrame, target::NodeName, prior::ConjugatePriors.MvNormalInverseGamma)

    arr = data[target]
    eltype(arr) <: Real || error("fit ConditionalLinearGaussianCPD requrires target to be numeric")

    clg = fit(ConditionalLinearGaussianCPD, data, target, prior)
    NannableConditionalLinearGaussianCPD(target, NodeName[], NodeName[], [clg])
end
function Distributions.fit(::Type{NannableConditionalLinearGaussianCPD},
    data::DataFrame,
    target::NodeName,
    parents::Vector{NodeName},
    prior::ConjugatePriors.MvNormalInverseGamma;
    parents_nannable = parents[find(c->eltype(data[c])<:Float64 && any(v->isnan(v),data[c]), 1:ncol(data))],
    )

    if isempty(parents)
        return fit(NannableConditionalLinearGaussianCPD, data, target)
    end

    nparents_nann = length(parents_nannable)

    if nparents_nann != 0

        # ---------------------
        # fit linear gaussians

        dims = fill(2, length(nparents_nann))
        clgs = Array(ConditionalLinearGaussianCPD, 2^nparents_nann)
        for (q, parent_instantiation) in enumerate(product(dims...))
            indeces = Int[]
            non_nan_parent_indeces = parent_instantiation .== 1
            non_nan_parents = parents[non_nan_parent_indeces]
            sub_prior = ConjugatePriors.MvNormalInverseGamma(prior.μ[[non_nan_parent_indeces; end]], prior.Λ[[non_nan_parent_indeces; end], [non_nan_parent_indeces; end]], prior.a, prior.b)

            for i in 1 : nrow(data)
                is_match = true
                for j in 1 : nparents_nann
                    if parent_instantiation[j] == 2 # NaN
                        deleteat!(parent_subset, findfirst(parent_subset, ))
                    end

                    if !(
                          (parent_instantiation[j] == 1 && !isnan(data[i, parents_nannable[j]])) ||
                          (parent_instantiation[j] == 2 &&  isnan(data[i, parents_nannable[j]]))
                        )

                        is_match = false
                        break
                    end
                end

                if is_match # parental instantiation matches
                    push!(indeces, i)
                end
            end

            clgs[q] = fit(ConditionalLinearGaussianCPD, data[indeces, :], target, non_nan_parents, sub_prior)
        end
        NannableConditionalLinearGaussianCPD(target, parents, parents_nannable, clgs)

    else # no nannable parents
        clg = fit(ConditionalLinearGaussianCPD, data, target, parents, prior)
        NannableConditionalLinearGaussianCPD(target, parents, Symbol[], [clg])
    end
end