

# export  fit_cubic_spline,
#       sample_spline,
#       sample_spline_derivative,
#       sample_spline_derivative2,
#       sample_spline_speed,
#       sample_spline_theta,
#       sample_spline_curvature,
#       sample_spline_derivative_of_curvature,
#       calc_curve_length,
#       arclength,
#       calc_curve_param_given_arclen

function _integrate_simpsons(f::Function, a::Real, b::Real, n::Int)
    # integrate using Composite Simpson's rule
    # reference: https://en.wikipedia.org/wiki/Simpson%27s_rule

    @assert(n > 0) # number of intervals
    @assert(mod(n,2) == 0) # n must be even

    h = (b-a)/n
    retval = f(a) + f(b)
    flip = true
    for i = 1 : n-1
        retval += f(a+i*h) * (flip ? 4 : 2)
        flip = !flip
    end
    return h/3*retval
end

function _fit_open(pts::AbstractVector{Float64} )
    # fits the 1-D spline such that:
    #   spline goes through each point
    #   first and second derivatives match at each inner point
    #   the second derivative at the ends is zero
    # see: http://mathworld.wolfram.com/CubicSpline.html

    # this function returns a 4×(n-1) spline coefficient matrix, where n = |pts|

    n = length(pts)-1
    @assert(n > 0)

    M = spzeros(n+1,n+1)
    for i = 1 : n
        M[i,i] = 4
        M[i,i+1] = 1
        M[i+1,i] = 1
    end
    M[n+1,n+1] = 2
    M[1,1] = 2

    Y = Array(Float64, n+1)
    for i = 1 : n+1
        ind_hi = min(i+1,n)
        ind_lo = max(1,i-1)
        Y[i] = 3*(pts[ind_hi] - pts[ind_lo])
    end

    D = M\Y

    spline_coeffs = Array(Float64, 4, n) # col is <a,b,c,d>
    spline_coeffs[1,:] = pts[1:n]
    spline_coeffs[2,:] = D[1:n]
    spline_coeffs[3,:] = 3*(pts[2:n+1] - pts[1:n]) -2*D[1:n]-D[2:n+1]
    spline_coeffs[4,:] = 2*(pts[1:n] - pts[2:n+1]) + D[1:n] + D[2:n+1]

    spline_coeffs
end
function _fit_closed(pts::AbstractVector{Float64} )
    # fits the 1-D spline such that:
    #   spline goes through each point
    #   first and second derivatives match at each inner point
    #   first the second derivative at the ends match
    # see: http://mathworld.wolfram.com/CubicSpline.html

    # this function returns a 4×n spline coefficient matrix, where n = |pts|

    n = length(pts)-1
    @assert(n > 0)

    M = spzeros(n+1,n+1)
    for i = 1 : n
        M[i,i] = 4
        M[i,i+1] = 1
        M[i+1,i] = 1
    end
    M[n+1,n+1] = 4
    M[1,n+1] = 1
    M[n+1,1] = 1

    Y = Array(Float64, n+1)
    Y[1] = 3*(pts[2] - pts[n+1])
    for i = 2 : n
        Y[i] = 3*(pts[i+1] - pts[i-1])
    end
    Y[end] = 3*(pts[1] - pts[n])

    D = M\Y

    spline_coeffs = Array(Float64, 4, n+1) # col is <a,b,c,d>
    spline_coeffs[1,:] = pts
    spline_coeffs[2,:] = D
    spline_coeffs[3,1:n] = 3*(pts[2:n+1] - pts[1:n]) -2*D[1:n]-D[2:n+1]
    spline_coeffs[4,1:n] = 2*(pts[1:n] - pts[2:n+1]) + D[1:n] + D[2:n+1]
    spline_coeffs[3,n+1] = 3*(pts[1] - pts[n+1]) -2*D[n+1]-D[1]
    spline_coeffs[4,n+1] = 2*(pts[n+1] - pts[1]) + D[n+1] + D[1]

    spline_coeffs
end
function _fit_open(pts::Matrix{Float64}) # 2×n {x,y}

    # see http://mathworld.wolfram.com/CubicSpline.html

    d,n = size(pts)
    n -= 1

    Y = Array(Float64, n+1)

    M = sparse(Int[], Int[], Float64[], n+1,n+1)
    for i in 1 : n
        M[i,i] = 4.0
        M[i,i+1] = 1.0
        M[i+1,i] = 1.0
    end
    M[n+1,n+1] = 2.0
    M[1,1] = 2.0

    retval = Array(Matrix{Float64}, d)
    for k in 1 : d

        for i in 1 : n+1
            ind_hi = min(i+1,n)
            ind_lo = max(1,i-1)
            Y[i] = 3*(pts[k,ind_hi] - pts[k,ind_lo])
        end

        D = M \ Y

        spline_coeffs = Array(Float64, 4, n) # col is <a,b,c,d> for a + b⋅t + c⋅t² + d⋅t³
        spline_coeffs[1,:] = pts[k,1:n] # x₀
        spline_coeffs[2,:] = D[1:n]     # x'₀
        spline_coeffs[3,:] = 3*(pts[k,2:n+1]' - pts[k,1:n]') -2*D[1:n] - D[2:n+1] # -3x₀ + 3x₁ - 2x'₀ - x'₁
        spline_coeffs[4,:] = 2*(pts[k,1:n]' - pts[k,2:n+1]') +  D[1:n] + D[2:n+1] #  2x₀ - 2x₁ +  x'₀ + x'₁

        retval[k] = spline_coeffs
    end
    retval
end
function _fit_closed(pts::AbstractMatrix{Float64})
    d = size(pts,1)
    retval = Array(Matrix{Float64}, d)
    for i = 1 : d
        retval[i] = _fit_closed(vec(pts[i,:]))
    end
    retval
end

function fit_cubic_spline(pts::AbstractArray{Float64}; open::Bool=true)
    if open
        return _fit_open(pts)
    else
        return _fit_closed(pts)
    end
end

function sample_spline(spline_coeffs::AbstractVector{Float64}, t::Float64)
    # here t is generally expected to be t ∈ [0,1]
    return spline_coeffs[1] + t*(spline_coeffs[2] + t*(spline_coeffs[3] + t*spline_coeffs[4]))
end
function sample_spline(spline_coeffs::AbstractMatrix{Float64}, t::Float64)
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    @assert(size(spline_coeffs, 1) == 4)
    col_ind = clamp(ceil(Int, t), 1, size(spline_coeffs,2))
    sample_spline(spline_coeffs[:,col_ind], t-col_ind+1)
end
function sample_spline(spline_coeffs::AbstractVector{Float64}, t_arr::AbstractVector{Float64})
    # here t is generally expected to be t ∈ [0,1]

    a = spline_coeffs[1]
    b = spline_coeffs[2]
    c = spline_coeffs[3]
    d = spline_coeffs[4]

    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        retval[i] = a + t*(b + t*(c + t*d))
    end
    retval
end
function sample_spline(spline_coeffs::AbstractMatrix{Float64}, t_arr::AbstractVector{Float64})
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    @assert(size(spline_coeffs, 1) == 4)
    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        col_ind = clamp(ceil(Int, t), 1, size(spline_coeffs,2))
        retval[i] = sample_spline(spline_coeffs[:,col_ind], t-col_ind+1)
    end
    retval
end

function sample_spline_derivative(spline_coeffs::AbstractVector{Float64}, t::Float64)
    # here t is generally expected to be t ∈ [0,1]
    return spline_coeffs[2] + t*(2spline_coeffs[3] + t*3spline_coeffs[4])
end
function sample_spline_derivative(spline_coeffs::AbstractMatrix{Float64}, t::Float64)
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    @assert(size(spline_coeffs, 1) == 4)
    col_ind = clamp(ceil(Int, t), 1, size(spline_coeffs,2))
    sample_spline_derivative(spline_coeffs[:,col_ind], t-col_ind+1)
end
function sample_spline_derivative(spline_coeffs::AbstractVector{Float64}, t_arr::AbstractVector{Float64})
    # here t is generally expected to be t ∈ [0,1]

    b = spline_coeffs[2]
    c = spline_coeffs[3]
    d = spline_coeffs[4]

    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        retval[i] = b + t*(2c + t*3d)
    end
    retval
end
function sample_spline_derivative(spline_coeffs::AbstractMatrix{Float64}, t_arr::AbstractVector{Float64})
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    @assert(size(spline_coeffs, 1) == 4)
    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        col_ind = clamp(ceil(Int, t), 1, size(spline_coeffs,2))
        retval[i] = sample_spline_derivative(spline_coeffs[:,col_ind], t-col_ind+1)
    end
    retval
end

function sample_spline_derivative2(spline_coeffs::AbstractVector{Float64}, t::Float64)
    # here t is generally expected to be t ∈ [0,1]
    return 2spline_coeffs[3] + t*6spline_coeffs[4]
end
function sample_spline_derivative2(spline_coeffs::AbstractMatrix{Float64}, t::Float64)
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    @assert(size(spline_coeffs, 1) == 4)
    col_ind = clamp(ceil(Int, t), 1, size(spline_coeffs,2))
    sample_spline_derivative2(spline_coeffs[:,col_ind], t-col_ind+1)
end
function sample_spline_derivative2(spline_coeffs::AbstractVector{Float64}, t_arr::AbstractVector{Float64})
    # here t is generally expected to be t ∈ [0,1]

    b = spline_coeffs[2]
    c = spline_coeffs[3]
    d = spline_coeffs[4]

    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        retval[i] = 2c + t*6d
    end
    retval
end
function sample_spline_derivative2(spline_coeffs::AbstractMatrix{Float64}, t_arr::AbstractVector{Float64})
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    @assert(size(spline_coeffs, 1) == 4)
    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        col_ind = clamp(ceil(Int, t), 1, size(spline_coeffs,2))
        retval[i] = sample_spline_derivative2(spline_coeffs[:,col_ind], t-col_ind+1)
    end
    retval
end

function sample_spline_speed(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t::Float64)
    dxdt = sample_spline_derivative(spline_coeffs_x, t)
    dydt = sample_spline_derivative(spline_coeffs_y, t)
    hypot(dxdt, dydt)
end
function sample_spline_speed(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t::Float64)
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    col_ind = clamp(ceil(Int, t), 1, n)::Int
    sample_spline_speed(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1)
end
function sample_spline_speed(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t_arr::AbstractVector{Float64})
    # here t is generally expected to be t ∈ [0,1]

    bx = spline_coeffs_x[2]
    cx = spline_coeffs_x[3]
    dx = spline_coeffs_x[4]

    by = spline_coeffs_y[2]
    cy = spline_coeffs_y[3]
    dy = spline_coeffs_y[4]

    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        dxdt = bx + t*(2cx + t*3dx)
        dydt = by + t*(2cy + t*3dy)
        retval[i] = hypot(dxdt, dydt)
    end
    retval
end
function sample_spline_speed(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t_arr::AbstractVector{Float64})
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.

    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        col_ind = clamp(ceil(Int, t), 1, n)
        retval[i] = sample_spline_speed(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1)
    end
    retval
end

function sample_spline_theta(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t::Float64;
    stepsize=1e-4
    )

    # compute the angle from positive x-axis (counter-clockwise positive) of the curve in the positive t direction at t
    # uses an approximation via small step size instead of derivative due to zero-derivative issues
    # uses the forward derivative approximation unless it would put it out of range
    # result returned is in radians

    t_lo, t_hi = t, t+stepsize
    if t_hi > 1.0
        t_lo, t_hi = t-min(1000stepsize,0.1), t
    end

    x1 = sample_spline(spline_coeffs_x, t_lo)
    x2 = sample_spline(spline_coeffs_x, t_hi)
    y1 = sample_spline(spline_coeffs_y, t_lo)
    y2 = sample_spline(spline_coeffs_y, t_hi)

    # println("(t, lo, hi)  $t   $t_lo   $t_hi, ($(atan2(y2-y1, x2-x1)))")

    atan2(y2-y1, x2-x1)
end
function sample_spline_theta(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t::Float64)
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    col_ind = clamp(ceil(Int, t), 1, n)
    sample_spline_theta(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1)
end
function sample_spline_theta(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t_arr::AbstractVector{Float64})
    # here t is generally expected to be t ∈ [0,1]

    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        retval[i] = sample_spline_theta(spline_coeffs_x, spline_coeffs_y, t)
    end
    retval
end
function sample_spline_theta(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t_arr::AbstractVector{Float64})
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.

    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        col_ind = clamp(ceil(Int, t), 1, n)
        retval[i] = sample_spline_theta(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1)
    end
    retval
end

function sample_spline_curvature(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t::Float64)
    # computes the signed curvature

    dx  = sample_spline_derivative( spline_coeffs_x, t)
    dy  = sample_spline_derivative( spline_coeffs_y, t)
    ddx = sample_spline_derivative2(spline_coeffs_x, t)
    ddy = sample_spline_derivative2(spline_coeffs_y, t)

    (dx*ddy - dy*ddx)/(dx*dx + dy*dy)^1.5
end
function sample_spline_curvature(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t::Float64)
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    col_ind = clamp(ceil(Int, t), 1, n)
    sample_spline_curvature(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1)
end
function sample_spline_curvature(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t_arr::AbstractVector{Float64})
    # here t is generally expected to be t ∈ [0,1]

    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        retval[i] = sample_spline_curvature(spline_coeffs_x, spline_coeffs_y, t)
    end
    retval
end
function sample_spline_curvature(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t_arr::AbstractVector{Float64})
    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.

    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        col_ind = clamp(ceil(Int, t), 1, n)
        retval[i] = sample_spline_curvature(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1)
    end
    retval
end

function sample_spline_derivative_of_curvature(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t::Float64;
    stepsize=1e-4
    )

    # computes the derivative of the signed curvature

    t_lo, t_hi = t, t+stepsize
    if t_hi > 1.0
        t_lo, t_hi = t-stepsize, t
    end

    κ_hi = sample_spline_curvature(spline_coeffs_x, spline_coeffs_y, t_hi)
    κ_lo = sample_spline_curvature(spline_coeffs_x, spline_coeffs_y, t_lo)

    (κ_hi - κ_lo) / stepsize
end
function sample_spline_derivative_of_curvature(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t::Float64;
    stepsize=1e-4
    )

    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.
    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    col_ind = clamp(ceil(Int, t), 1, n)
    sample_spline_derivative_of_curvature(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1, stepsize=stepsize)
end
function sample_spline_derivative_of_curvature(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64}, t_arr::AbstractVector{Float64};
    stepsize=1e-4
    )

    # here t is generally expected to be t ∈ [0,1]

    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        retval[i] = sample_spline_derivative_of_curvature(spline_coeffs_x, spline_coeffs_y, t, stepsize=stepsize)
    end
    retval
end
function sample_spline_derivative_of_curvature(spline_coeffs_x::AbstractMatrix{Float64}, spline_coeffs_y::AbstractMatrix{Float64}, t_arr::AbstractVector{Float64};
    stepsize=1e-4
    )

    # for t ∈ (-∞,1] we use spline_coeffs[:,1]
    # for t ∈ [1,2] we use spline_coeffs[:,2]
    # etc.

    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x, 1) == 4)
    @assert(size(spline_coeffs_y, 1) == 4)
    @assert(n == size(spline_coeffs_y, 2))
    retval = Array(Float64, length(t_arr))
    for (i,t) in enumerate(t_arr)
        col_ind = clamp(ceil(Int, t), 1, n)
        retval[i] = sample_spline_derivative_of_curvature(spline_coeffs_x[:,col_ind], spline_coeffs_y[:,col_ind], t-col_ind+1, stepsize=stepsize)
    end
    retval
end

function calc_curve_length(spline_coeffs_x::AbstractVector{Float64}, spline_coeffs_y::AbstractVector{Float64};
    n_intervals::Int = 100
    )

    # integrate using Simpson's rule
    # _integrate_simpsons(t->sample_spline_speed(spline_coeffs_x, spline_coeffs_y, t), 0.0, 1.0, n_intervals)

    a = 0.0
    b = 1.0
    n = n_intervals

    h = (b-a)/n
    retval = sample_spline_speed(spline_coeffs_x, spline_coeffs_y, a) + sample_spline_speed(spline_coeffs_x, spline_coeffs_y, b)
    flip = true
    for i = 1 : n-1
        retval += sample_spline_speed(spline_coeffs_x, spline_coeffs_y, a+i*h) * (flip ? 4 : 2)
        flip = !flip
    end
    return h/3*retval
end
function calc_curve_length(
    spline_coeffs_x::AbstractMatrix{Float64},
    spline_coeffs_y::AbstractMatrix{Float64};
    n_intervals_per_segment::Int = 100
    )

    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_y, 2) == n)
    @assert(size(spline_coeffs_x, 1) == size(spline_coeffs_y, 1) == 4)

    len = 0.0
    for i = 1 : n
        len += calc_curve_length(spline_coeffs_x[:,i], spline_coeffs_y[:,i], n_intervals = n_intervals_per_segment)
    end
    len
end

function arclength(
    spline_coeffs_x::AbstractVector{Float64},
    spline_coeffs_y::AbstractVector{Float64},
    t_min::Real = 0.0,
    t_max::Real = 1.0,
    n_intervals::Int = 100
    )

    if isapprox(t_min, t_max)
        return 0.0
    end

    # _integrate_simpsons(t->sample_spline_speed(spline_coeffs_x, spline_coeffs_y, t), t_min, t_max, n_intervals)

    a = t_min
    b = t_max
    n = n_intervals

    h = (b-a)/n
    retval = sample_spline_speed(spline_coeffs_x, spline_coeffs_y, a) + sample_spline_speed(spline_coeffs_x, spline_coeffs_y, b)
    flip = true
    for i = 1 : n-1
        retval += sample_spline_speed(spline_coeffs_x, spline_coeffs_y, a+i*h) * (flip ? 4 : 2)
        flip = !flip
    end
    return h/3*retval
end
function arclength(
    spline_coeffs_x::AbstractMatrix{Float64},
    spline_coeffs_y::AbstractMatrix{Float64},
    t_min::Real = 0.0,
    t_max::Real = size(spline_coeffs_x, 2),
    n_intervals_per_segment::Int = 100
    )

    n = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_y, 2) == n)
    @assert(size(spline_coeffs_x, 1) == size(spline_coeffs_y, 1) == 4)

    if isapprox(t_min, t_max)
        return 0.0
    end

    # println("tmin/tmax: $t_min / $t_max")

    len = 0.0
    for i = floor(Int, t_min) : min(floor(Int, t_max), n-1)
        t_lo, t_hi = float(i), i+1.0

        spline_ind = i+1
        t_in_min = max(t_lo, t_min) - t_lo
        t_in_max = min(t_hi, t_max) - t_lo
        # println("($i) t_lo: $t_lo, t_hi: $t_hi, : $t_in_min → $t_in_max")
        len += arclength(spline_coeffs_x[:,spline_ind], spline_coeffs_y[:,spline_ind], t_in_min, t_in_max, n_intervals_per_segment)
    end
    # println("len: ", len)
    len
end

function calc_curve_param_given_arclen(
    spline_coeffs_x :: AbstractVector{Float64},
    spline_coeffs_y :: AbstractVector{Float64},
    s :: Float64;
    max_iterations :: Int=100,
    curve_length :: Float64 = calc_curve_length(spline_coeffs_x, spline_coeffs_y),
    epsilon::Float64 = 1e-4 # tolerance required before termination
    )

    # finds t such that p(t) is a distance s from start of curve
    # returns t=0 if s ≤ 0.0 and t=1 if s > L
    if s ≤ 0.0
        return 0.0
    elseif s ≥ curve_length
        return 1.0
    end

    t = s/curve_length
    lo, hi = 0.0, 1.0

    # @printf("%10s %10s %10s %10s %10s %10s\n", "iter", "lo", "hi", "t", "s", "F")
    # println("-"^65)

    for iter = 1 : max_iterations
        F = arclength(spline_coeffs_x, spline_coeffs_y, 0.0, t) - s

        # @printf("%10d %10.5f %10.5f %10.5f %10.5f %10.5f\n", iter-1, lo, hi, t, s, F)

        if abs(F) < epsilon
            # |F(t)| is close enough to zero, report it
            return t
        end

        DF = sample_spline_speed(spline_coeffs_x, spline_coeffs_y, t)
        tCandidate = t - F/DF
        if F > 0
            hi = t
            t = tCandidate ≤ lo ? 0.5*(lo+hi) : tCandidate
        else
            lo = t
            t = tCandidate ≥ hi ? 0.5*(lo+hi) : tCandidate
        end
    end

    # @printf("%10d %10.5f %10.5f %10.5f %10.5f %10s\n", max_iterations, lo, hi, t, s, "-")

    t
end
function calc_curve_param_given_arclen(
    spline_coeffs_x :: AbstractMatrix{Float64},
    spline_coeffs_y :: AbstractMatrix{Float64},
    s :: Float64;
    max_iterations :: Int=100,
    n_integration_intervals :: Int=100, # must be multiple of 2
    curve_length :: Float64 = calc_curve_length(spline_coeffs_x, spline_coeffs_y),
    epsilon::Float64 = 1e-4 # tolerance required before termination
    )

    # finds t such that p(t) is a distance s from start of curve
    # returns t=0 if s ≤ 0.0 and t=t_max if s > L

    n_segments = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x,1) == size(spline_coeffs_y,1) == 4)
    @assert(size(spline_coeffs_y,2) == n_segments)

    if s ≤ 0.0
        return 0.0
    elseif s ≥ curve_length
        return float(n_segments)
    end

    t = s/curve_length
    lo, hi = 0.0, float(n_segments)

    # @printf("%10s %10s %10s %10s %10s %10s\n", "iter", "lo", "hi", "t", "s", "F")
    # println("-"^65)

    for iter = 1 : max_iterations
        F = arclength(spline_coeffs_x, spline_coeffs_y, 0.0, t, n_integration_intervals) - s

        # @printf("%10d %10.5f %10.5f %10.5f %10.5f %10.5f\n", iter-1, lo, hi, t, s, F)

        if abs(F) < epsilon
            return t
        end

        DF = sample_spline_speed(spline_coeffs_x, spline_coeffs_y, t)
        tCandidate = t - F/DF
        if F > 0
            hi = t
            t = tCandidate ≤ lo ? 0.5*(lo+hi) : tCandidate
        else
            lo = t
            t = tCandidate ≥ hi ? 0.5*(lo+hi) : tCandidate
        end
    end

    # @printf("%10d %10.5f %10.5f %10.5f %10.5f %10s\n", max_iterations, lo, hi, t, s, "-")

    t
end
function calc_curve_param_given_arclen(
    spline_coeffs_x :: AbstractVector{Float64},
    spline_coeffs_y :: AbstractVector{Float64},
    s_arr :: AbstractVector{Float64}; # assumes s_arr is sorted
    max_iterations :: Int=100,
    curve_length :: Float64 = calc_curve_length(spline_coeffs_x, spline_coeffs_y),
    epsilon::Float64 = 1e-4 # tolerance required before termination
    )

    n = length(s_arr)
    t_arr = Array(Float64, n)

    s = s_arr[1]
    t = s/curve_length
    if s ≤ 0.0
        t = 0.0
    elseif s ≥ curve_length
        t = 1.0
    end

    lo =  0.0

    for (i,s) in enumerate(s_arr)

        if s ≤ 0.0
            t = 0.0
            t_arr[i], lo = t, t
            continue
        elseif s ≥ curve_length
            t = 1.0
            t_arr[i], lo = t, t
            continue
        end

        hi = 1.0
        for iter = 1 : max_iterations
            F = arclength(spline_coeffs_x, spline_coeffs_y, 0.0, t) - s

            if abs(F) < epsilon
                t_arr[i], lo = t, t
                continue
            end

            DF = sample_spline_speed(spline_coeffs_x, spline_coeffs_y, t)
            tCandidate = t - F/DF
            if F > 0
                hi = t
                t = tCandidate ≤ lo ? 0.5*(lo+hi) : tCandidate
            else
                lo = t
                t = tCandidate ≥ hi ? 0.5*(lo+hi) : tCandidate
            end
        end

        t_arr[i], lo = t, t
    end

    t_arr
end
function calc_curve_param_given_arclen(
    spline_coeffs_x :: AbstractMatrix{Float64},
    spline_coeffs_y :: AbstractMatrix{Float64},
    s_arr :: AbstractVector{Float64}; # assumes s_arr is sorted
    max_iterations :: Int = 50,
    curve_length :: Float64 = calc_curve_length(spline_coeffs_x, spline_coeffs_y),
    epsilon::Float64 = 1e-4, # tolerance required before termination
    n_intervals_in_arclen::Int = 100
    )

    n_segments = size(spline_coeffs_x, 2)
    @assert(size(spline_coeffs_x,1) == size(spline_coeffs_y,1) == 4)
    @assert(size(spline_coeffs_y,2) == n_segments)

    n = length(s_arr)
    t_arr = Array(Float64, n)

    s = s_arr[1]
    t = s/curve_length
    if s ≤ 0.0
        t = 0.0
    elseif s ≥ curve_length
        return float(n_segments)
    end

    lo =  0.0
    # println("L: ", curve_length)
    # println("s_max: ", s_arr[end])
    for (i,s) in enumerate(s_arr)

        # println("\ns: ", s)

        if s ≤ 0.0
            t = 0.0
            t_arr[i] = lo = t
            continue
        elseif s ≥ curve_length
            t = float(n_segments)
            t_arr[i] = lo = t
            continue
        end

        hi = float(n_segments)
        for iter = 1 : max_iterations
            F = arclength(spline_coeffs_x, spline_coeffs_y, 0.0, t, n_intervals_in_arclen) - s

            if abs(F) < epsilon
                break
            end

            DF = sample_spline_speed(spline_coeffs_x, spline_coeffs_y, t)
            tCandidate = t - F/DF
            if F > 0
                hi = t
                t = tCandidate ≤ lo ? 0.5*(lo+hi) : tCandidate
            else
                lo = t
                t = tCandidate ≥ hi ? 0.5*(lo+hi) : tCandidate
            end
        end

        t_arr[i] = lo = t
    end

    t_arr
end