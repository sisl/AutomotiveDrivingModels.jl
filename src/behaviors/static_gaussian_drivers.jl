export StaticGaussianDriver

type StaticGaussianDriver{A} <: DriverModel{A,IntegratedContinuous}
    C::IntegratedContinuous
    P::MvNormal
end

get_name(::StaticGaussianDriver) = "StaticGaussian"
action_context(model::StaticGaussianDriver) = model.C
function Base.rand{A}(model::StaticGaussianDriver{A})
    a = rand(model.P)
    convert(A, a)
end
Distributions.pdf{A}(model::StaticGaussianDriver{A}, a::A) = pdf(model.P, convert(Vector{Float64}, a))
Distributions.logpdf{A}(model::StaticGaussianDriver{A}, a::A) = logpdf(model.P, convert(Vector{Float64}, a))