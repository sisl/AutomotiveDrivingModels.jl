# change the path to have the given extension
toext(path::AbstractString, ext::AbstractString) = splitext(path)[1] * cleanext(ext)

# append the dot for an extension if need be
cleanext(ext::AbstractString) = ext[1] == '.' ? ext : "." * ext
