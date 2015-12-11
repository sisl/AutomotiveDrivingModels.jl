using NLopt
using AutomotiveDrivingModels

##############################
# PARAMETERS
##############################

const INCLUDE_FILE_BASE = "realworld"

const INCLUDE_FILE = Pkg.dir("AutomotiveDrivingModels", "scripts", "extract_params.jl")
include(INCLUDE_FILE)

################################
# LOAD DATA
################################

dset = JLD.load(joinpath(EVALUATION_DIR, "dataset_subset_car_following.jld"), "model_training_data")
df = dset.dataframe

################################
# FIT IDM
################################
#=
Minimize the RSE on the IDM prediction

Extracted:
- ACCEL_MAX
- VEL_MAX

Params:
- MIN_HEADWAY
- DECEL_COMFORTABLE

=#


accel_max = maximum(filter!(v->!isinf(v) && !isnan(v), convert(Array{Float64}, df[:acc]))) # m/s²
accel_min = minimum(filter!(v->!isinf(v) && !isnan(v), convert(Array{Float64}, df[:acc]))) # m/s²
vel_max = maximum(filter!(v->!isinf(v) && !isnan(v), convert(Array{Float64}, df[:speed]))) # m/s

println("accel_max: ", accel_max)
println("accel_min: ", accel_min)
println("vel_max: ",   vel_max)

arr_Δv_front = Float64[]
arr_d_front = Float64[]
arr_T = Float64[]
arr_v = Float64[]
arr_a = Float64[]
for i in 1 : nrow(df)

    Δv_front = df[i, symbol(V_X_FRONT)]
    d_front = df[i, symbol(D_X_FRONT)]
    T = df[i, symbol(TIMEGAP_X_FRONT)]
    v = df[i, symbol(SPEED)]
    a = df[i, symbol(FUTUREACCELERATION_250MS)]

    if !isinf(Δv_front) && !isinf(d_front) && !isinf(T) && !isinf(v) && !isinf(a)
        push!(arr_Δv_front, Δv_front)
        push!(arr_d_front, d_front)
        push!(arr_T, T)
        push!(arr_v, v)
        push!(arr_a, a)
    end
end

function idm(
    Δv_front::Float64,
    d_front::Float64,
    v::Float64, # ego speed
    T::Float64, # timegap_x_front
    a_max::Float64,
    v_max::Float64,
    b_comf::Float64,
    d_front_min::Float64,
    )

    d_des = d_front_min + T*v - (v*Δv_front) / (2*sqrt(a_max*b_comf))
    a_idm = a_max * (1.0 - (v/v_max)^4 - (d_des/d_front)^2)

    a_idm
end

function opt_idm(x::Vector, grad::Vector)

    d_front_min = x[1]
    b_comf = x[2]
    a_max = x[3]
    v_max = x[4]

    rse = 0.0
    for i in 1 : length(arr_Δv_front)
        a_idm_pred = idm(arr_Δv_front[i], arr_d_front[i], arr_v[i], arr_T[i],
                         a_max, v_max, b_comf, d_front_min)
        a_true = arr_a[i]
        # @printf("%10.3f  %10.3f  %10.3f\n", a_idm_pred, a_true, (a_idm_pred - a_true)^2)
        rse += (a_idm_pred - a_true)^2
    end
    rse
end

x_init = [1.0, 1.5, 1.0, 30.0]
x_lo = [-5.0,  0.0, 0.0, 0.0]
x_hi = [5.0, 100.0, 5.0, 100.0]

# println(opt_idm([500.0,-1.4804697036743164], Float64[]))
# exit()

opt = Opt(:LN_SBPLX, length(x_init))
xtol_rel!(opt, 1e-4)
lower_bounds!(opt, x_lo)
upper_bounds!(opt, x_hi)
min_objective!(opt, opt_idm)

minf, minx, ret = optimize(opt, x_init)

println("minf: ", minf)
println("minx: ", minx)
println("ret:")
println(ret)

exit()

################################
# FIT SUMO
################################
#=
Minimize the RSE on the SUMO prediction

Extracted:
- ACCEL_MAX
- VEL_MAX

Params:
- RESPONSE_TIME_AVERAGE
=#

arr_v_front = Float64[]
arr_d_front = Float64[]
arr_v = Float64[]
arr_a = Float64[]
for i in 1 : nrow(df)

    v_front = df[i, symbol(V_X_FRONT)]
    d_front = df[i, symbol(D_X_FRONT)]
    v = df[i, symbol(SPEED)]
    a = df[i, symbol(FUTUREACCELERATION_250MS)]

    if !isinf(v_front) && !isinf(d_front) && !isinf(v) && !isinf(a)
        push!(arr_v_front, v_front)
        push!(arr_d_front, d_front)
        push!(arr_v, v)
        push!(arr_a, a)
    end
end

function sumo(
    v_front::Float64,
    d_front::Float64,
    v::Float64, # ego speed
    τ::Float64,
    a_max::Float64,
    b_max::Float64,
    v_max::Float64,
    )

    v_safe = -τ*b_max + sqrt((τ*b_max)^2 + v_front^2 + 2*b_max*d_front)
    v_des = min(min(v_safe, v + a_max*(1 - v/v_max)*DEFAULT_SEC_PER_FRAME), v_max)
    a_sumo = (max(0.0, v_des) - v_safe) / DEFAULT_SEC_PER_FRAME

    a_sumo
end
function opt_sumo(x::Vector, grad::Vector)

    τ = x[1]
    a_max = x[2]
    b_max = x[3]
    v_max = x[4]

    # a_max = accel_max
    # b_max = -accel_min
    # v_max = vel_max


    rse = 0.0
    for i in 1 : length(arr_Δv_front)
        a_idm_pred = sumo(arr_v_front[i], arr_d_front[i], arr_v[i], τ,
                          a_max, b_max, v_max)
        a_true = arr_a[i]
        # @printf("%10.3f  %10.3f  %10.3f\n", a_idm_pred, a_true, (a_idm_pred - a_true)^2)
        rse += (a_idm_pred - a_true)^2
    end
    rse
end

x_init = [1.2, 3.0, 2.0, 40.0]
x_lo = [0.0, 0.0, 0.0, 0.0]
x_hi = [5.0, 50.0, 5.0, 100.0]

# println(opt_sumo([1.2], Float64[]))

opt = Opt(:LN_SBPLX, length(x_init))
xtol_rel!(opt, 1e-4)
lower_bounds!(opt, x_lo)
upper_bounds!(opt, x_hi)
min_objective!(opt, opt_sumo)

minf, minx, ret = optimize(opt, x_init)

println("minf: ", minf)
println("minx: ", minx)
println("ret:")
println(ret)


println("DONE")
exit()