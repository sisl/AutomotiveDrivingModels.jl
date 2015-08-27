
# change the path to have the given extension
toext(path::String, ext::String) = splitext(path)[1] * _cleanext(ext)

# append the dot for an extension if need be
cleanext(ext::String) = ext[1] == '.' ? ext : "." * ext