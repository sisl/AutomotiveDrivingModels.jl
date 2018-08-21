using Pkg
packages = keys(Pkg.installed())
if !in("Vec", packages)
    Pkg.add("https://github.com/sisl/Vec.jl.git")
end
if !in("Records", packages)
    Pkg.add("https://github.com/sisl/Records.jl.git")
end
