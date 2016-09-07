export
    FoldSet,

    FOLD_TRAIN,
    FOLD_TEST,

    foldset_match,
    foldset_withhold,

    check_fold_match

#########################################

const FOLD_TRAIN = 1
const FOLD_TEST = 2

immutable FoldSet
    assignment::Vector{Int} # assignment[i] = j means the ith element is assigned to fold j
    fold::Int # target fold we either want to match or withhold, based on `match_fold`
    match_fold::Bool
end
foldset_match(assignment::Vector{Int}, fold::Int) = FoldSet(assignment, fold, true)
foldset_withhold(assignment::Vector{Int}, fold::Int) = FoldSet(assignment, fold, false)

function check_fold_match(fold::Integer, fold_assignment::Integer, match_fold::Bool)
    (fold != 0) && (fold_assignment != 0) && # NOTE(tim): zero never matches
        ((match_fold && fold_assignment == fold) || (!match_fold && fold_assignment != fold))
end
check_fold_match(value::Integer, foldset::FoldSet) = check_fold_match(foldset.fold, value, foldset.match_fold)

#########################################
# Iterator

function _find_next_valid_fold_match(foldset::FoldSet, state::Int)
    while state < length(foldset.assignment)
        state += 1
        if check_fold_match(foldset.assignment[state], foldset)
            return state
        end
    end
    state + 1 # returns length(foldset.assignment) + 1 on fail
end
Base.start(foldset::FoldSet) = _find_next_valid_fold_match(foldset, 0)
Base.done(foldset::FoldSet, state::Int) = state > length(foldset.assignment)
function Base.next(foldset::FoldSet, state::Int)
    @assert(check_fold_match(foldset.assignment[state], foldset))
    state, _find_next_valid_fold_match(foldset, state)
end
function Base.length(foldset::FoldSet)
    len = 0
    state = start(foldset)
    while !done(foldset, state)
        item, state = next(foldset, state)
        len += 1
    end
    len
end
function Base.collect(foldset::FoldSet)
    len = length(foldset)
    retval = Array(Int, len)
    for (i,j) in enumerate(foldset)
        retval[i] = j
    end
    retval
end