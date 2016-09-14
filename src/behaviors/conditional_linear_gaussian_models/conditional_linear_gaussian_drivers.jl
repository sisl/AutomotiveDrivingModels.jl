using BayesNets
using ConjugatePriors

include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "conditional_linear_gaussian_models", "nannable_clg_cpd.jl"))

export ConditionalLinearGaussianDriver

type ConditionalLinearGaussianDriver{A} <: DriverModel{A, IntegratedContinuous}

    rec::SceneRecord
    context::IntegratedContinuous

    cpds::Vector{NannableConditionalLinearGaussianCPD} # cpds for the targets, in topological order
    features::Vector{AbstractFeature}
    assignment::Dict{Symbol,Any}
    action::Vector{Float64}
end

get_name(::ConditionalLinearGaussianDriver) = "ConditionalLinearGaussian"
action_context(driver::ConditionalLinearGaussianDriver) = driver.context
function reset_hidden_state!(driver::ConditionalLinearGaussianDriver)
    empty!(driver.rec)
    driver
end
function observe!{A}(driver::ConditionalLinearGaussianDriver{A}, scene::Scene, roadway::Roadway, egoid::Int)

    update!(driver.rec, scene)
    vehicle_index = get_index_of_first_vehicle_with_id(scene, egoid)

    for i in 1: length(driver.features)
        feature = driver.features[i]
        fval = get(feature, driver.rec, roadway, vehicle_index)
        if is_feature_valid(fval)
            if inherent_type(feature) <: AbstractFloat
                driver.assignment[symbol(feature)] = convert(Float64, fval)
            else
                driver.assignment[symbol(feature)] = convert(Int, convert(Float64, fval))
            end
        else
            driver.assignment[symbol(feature)] = NaN
        end
    end

    driver
end

function Base.rand{A}(driver::ConditionalLinearGaussianDriver{A})
    for (i, cpd) in enumerate(driver.cpds)
        driver.action[i] = rand(cpd, driver.assignment)
    end
    convert(A, driver.action)
end
function Distributions.pdf{A}(driver::ConditionalLinearGaussianDriver{A}, a::A)

    prob = 1.0
    v = convert(Vector{Float64}, a)
    for (i,cpd) in enumerate(driver.cpds)
        driver.assignment[name(cpd)] = v[i]
        prob *= pdf(cpd, driver.assignment)
    end
    prob
end
function Distributions.logpdf{A}(driver::ConditionalLinearGaussianDriver{A}, a::A)

    logl = 0.0
    v = convert(Vector{Float64}, a)
    for (i,cpd) in enumerate(driver.cpds)
        driver.assignment[name(cpd)] = v[i]
        logl += pdf(cpd, driver.assignment)
    end
    logl
end

include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "conditional_linear_gaussian_models", "learning.jl"))