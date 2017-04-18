packages = keys(Pkg.installed())
if !in("Vec", packages)
    Pkg.clone("https://github.com/tawheeler/Vec.jl.git")
end
if !in("Records", packages)
    Pkg.clone("https://github.com/tawheeler/Records.jl.git")
end

