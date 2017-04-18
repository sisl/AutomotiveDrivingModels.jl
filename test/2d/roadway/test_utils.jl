@test isapprox(AutomotiveDrivingModels._mod2pi2(0.0), 0.0)
@test isapprox(AutomotiveDrivingModels._mod2pi2(0.5), 0.5)
@test isapprox(AutomotiveDrivingModels._mod2pi2(2pi + 1), 1.0)
@test isapprox(AutomotiveDrivingModels._mod2pi2(1 - 2pi), 1.0)