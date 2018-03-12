packages = keys(Pkg.installed())
if !in("Vec", packages)
    Pkg.clone("https://github.com/sisl/Vec.jl.git")
end
if !in("Records", packages)
    Pkg.clone("https://github.com/sisl/Records.jl.git")
end

