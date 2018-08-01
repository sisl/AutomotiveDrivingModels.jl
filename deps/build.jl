packages = keys(Pkg.installed())
if !in("Vec", packages)
    Pkg.clone("https://github.com/sisl/Vec.jl.git")
end
warn("Checking out Vec.jl branch 0_6 for compatibility with julia 0.6")
Pkg.checkout("Vec", "0_6")
if !in("Records", packages)
    Pkg.clone("https://github.com/sisl/Records.jl.git")
end

