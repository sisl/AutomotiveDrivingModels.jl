include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "gaussian_mixture_regression_models", "gaussian_mixture_regression.jl"))

export GaussianMixtureRegressionDriver

# Based on Comparison of Parametric and Non-Parametric Approaches for Vehicle Speed Prediction
type GaussianMixtureRegressionDriver{A,F<:AbstractFeatureExtractor} <: DriverModel{A, IntegratedContinuous}
    rec::SceneRecord
    context::IntegratedContinuous

    gmr::GMR
    extractor::F
    features::Vector{Float64}
    action::Vector{Float64}
end

get_name(::GaussianMixtureRegressionDriver) = "GMR"
action_context(driver::GaussianMixtureRegressionDriver) = driver.context
function reset_hidden_state!(driver::GaussianMixtureRegressionDriver)
    empty!(driver.rec)
    driver
end
function observe!{A,F}(driver::GaussianMixtureRegressionDriver{A,F}, scene::Scene, roadway::Roadway, egoid::Int)
    update!(driver.rec, scene)
    vehicle_index = get_index_of_first_vehicle_with_id(scene, egoid)
    pull_features!(driver.extractor, driver.features, driver.rec, roadway, vehicle_index)
    driver.gmr(driver.features)
    driver
end
Base.rand{A,F}(driver::GaussianMixtureRegressionDriver{A,F}) = convert(A, rand!(driver.action, driver.gmr.mixture_Act_given_Obs))
Distributions.pdf{A,F}(driver::GaussianMixtureRegressionDriver{A,F}, a::A) = pdf(driver.gmr.mixture_Act_given_Obs, convert(Vector{Float64}, a))
Distributions.logpdf{A,F}(driver::GaussianMixtureRegressionDriver{A,F}, a::A) = logpdf(driver.gmr.mixture_Act_given_Obs, convert(Vector{Float64}, a))

include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "gaussian_mixture_regression_models", "learning.jl"))
include(Pkg.dir("AutomotiveDrivingModels", "src", "behaviors", "gaussian_mixture_regression_models", "io.jl"))