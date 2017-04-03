# clone
urls = [
    "https://github.com/tawheeler/Vec.jl.git",
    "https://github.com/tawheeler/Records.jl.git",
]

for url in urls
    try
        Pkg.clone(url)
    catch e
        println("Exception when cloning $(url): $(e)")  
    end
end

# checkout specific branches
# checkouts = [
#     ("ForwardNets", "nextgen")
# ]

# for (pkg, branch) in checkouts
#     Pkg.checkout(pkg, branch)
# end
