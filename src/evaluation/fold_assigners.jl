export
    FoldAssigner,
    RandomFoldAssigner,
    OrderedFoldAssigner,

    assign_folds

abstract FoldAssigner
assign_folds(fa::FoldAssigner, nitems::Int, nfolds::Int) = error("get_fold_assignment not implemented for $a")


"""
    RandomFoldAssigner
Assign folds using a random permutation
"""
type RandomFoldAssigner <: FoldAssigner
    use_all_samples::Bool # true - use all samples and assign extra ones to the first folds
                          # false - ensure all folds are of the same size
    RandomFoldAssigner(use_all_samples::Bool=true) = new(use_all_samples)
end
function assign_folds(fa::RandomFoldAssigner, nitems::Int, nfolds::Int)
    assignment = zeros(Int, nitems)

    N = fa.use_all_samples ? nitems : nfolds*div(nitems, nfolds)

    p = randperm(nitems)
    fold = 1
    for i in 1 : N
        assignment[p[i]] = fold
        fold += 1
        if fold > nfolds
            fold = 1
        end
    end

    assignment
end

"""
    OrderedFoldAssigner
Assigns the first values to fold 1, the next block to fold 2, etc.
"""
type OrderedFoldAssigner <: FoldAssigner
    use_all_samples::Bool # true - use all samples and assign extra ones to the first folds
                          # false - ensure all folds are of the same size
    OrderedFoldAssigner(use_all_samples::Bool=true) = new(use_all_samples)
end
function assign_folds(fa::OrderedFoldAssigner, nitems::Int, nfolds::Int)
    assignment = zeros(Int, nitems)

    fold_count = div(nitems, nfolds)
    n_folds_with_extra = rem(nitems, nfolds)

    i = 0
    for fold in 1 : nfolds
        this_fold_count = fold_count
        if fa.use_all_samples && fold ≤ n_folds_with_extra
            this_fold_count += 1
        end

        for j in 1 : this_fold_count
            assignment[i+=1] = fold
        end
    end

    assignment
end