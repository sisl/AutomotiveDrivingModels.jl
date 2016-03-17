export convert_model_name_to_short_name

function convert_model_name_to_short_name(name::AbstractString)
    retval = ""
    for word in split(name)
        retval *= string(uppercase(word[1]))
    end
    retval
end