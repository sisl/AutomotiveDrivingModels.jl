type PrerecordedDriver <: DriverModel{NextState}
    trajdata::Trajdata # log we pull from
end

get_name(::StaticGaussianDriver) = "StaticGaussian"
function Base.rand{A}(model::StaticGaussianDriver{A})
    a = rand(model.P)
    convert(A, a)
end
Distributions.pdf{A}(model::StaticGaussianDriver{A}, a::A) = pdf(model.P, convert(Vector{Float64}, a))
Distributions.logpdf{A}(model::StaticGaussianDriver{A}, a::A) = logpdf(model.P, convert(Vector{Float64}, a))