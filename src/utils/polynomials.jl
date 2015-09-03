export
    Polynomial,
    Quintic,
    Quartic,

    p₁, p₂, p₃, p₄, p₁₂, p₂₃, p₁₂₃,
    translate

abstract Polynomial
immutable Quartic <: Polynomial # Fourth Order polynomial
    x₁::Float64
    x₂::Float64
    x₃::Float64
    x₄::Float64
    x₅::Float64
end
immutable Quintic <: Polynomial # Fifth Order polynomial
    x₁::Float64
    x₂::Float64
    x₃::Float64
    x₄::Float64
    x₅::Float64
    x₆::Float64
end

p₁(q::Quartic, t::Float64) = q.x₁ + t*(q.x₂ + t*(q.x₃ + t*(q.x₄ + t*q.x₅))) # eval poly
p₂(q::Quartic, t::Float64) = q.x₂ + t*(2q.x₃ + t*(3q.x₄ + t*4q.x₅)) # first derivative
p₃(q::Quartic, t::Float64) = 2q.x₃ + t*(6q.x₄ + t*12q.x₅) # second derivative
p₄(q::Quartic, t::Float64) = 6q.x₄ + t*24q.x₅ # third derivative

p₁(q::Quintic, t::Float64) = q.x₁ + t*(q.x₂ + t*(q.x₃ + t*(q.x₄ + t*(q.x₅ + t*q.x₆)))) # eval poly
p₂(q::Quintic, t::Float64) = q.x₂ + t*(2q.x₃ + t*(3q.x₄ + t*(4q.x₅ + t*5q.x₆))) # first derivative
p₃(q::Quintic, t::Float64) = 2q.x₃ + t*(6q.x₄ + t*(12q.x₅ + t*20q.x₆)) # second derivative
p₄(q::Quintic, t::Float64) = 6q.x₄ + t*(24q.x₅ + t*60q.x₆) # third derivative

p₁₂(p::Polynomial, t::Float64) = (p₁(p,t), p₂(p,t))
p₂₃(p::Polynomial, t::Float64) = (p₂(p,t), p₃(p,t))
p₁₂₃(p::Polynomial, t::Float64) = (p₁(p,t), p₂(p,t), p₃(p,t))

translate(q::Quartic, Δ₁::Float64) = Quartic(q.x₁+Δ₁, q.x₂, q.x₃, q.x₄, q.x₅)
translate(q::Quintic, Δ₁::Float64) = Quintic(q.x₁+Δ₁, q.x₂, q.x₃, q.x₄, q.x₅, q.x₆)