# using Base.Test

# immutable LaneTag
#     index_e :: Int
#     index_n :: Int
#     segment :: Int
#     lane    :: Int
# end

# include(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "vec", "Vec.jl")); using .Vec
# include(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "CommonTypes.jl")); using .CommonTypes

# reload(Pkg.dir("AutomotiveDrivingModels", "src", "utils", "Trajdata.jl"));

function are_sizes_consistent(pdset::PrimaryDataset)

    nframes = nrow(pdset.df_ego)
    @test nframes == length(pdset.ego_car_on_freeway)
    @test nframes == length(pdset.frameind2validfind)

    nvalidfinds = nrow(pdset.df_other)
    @test nvalidfinds ≤ nframes
    @test nvalidfinds == size(pdset.mat_other_indmap, 1)
    @test sum(pdset.ego_car_on_freeway) == nvalidfinds

    n_matinds = size(pdset.mat_other_indmap, 2)
    @test length(unique(collect(values(pdset.dict_other_idmap)))) == n_matinds
    @test pdset.maxcarind == div(ncol(pdset.df_other), pdset.df_other_ncol_per_entry)-1

    @test pdset.df_other_ncol_per_entry > 0
    @test pdset.df_other_ncol_per_entry == length(pdset.df_other_column_map)
end
function is_time_ascending(pdset::PrimaryDataset)

    num_frameinds = nframeinds(pdset)
    if num_frameinds ≥ 1
        for frameind = 2 : num_frameinds
            if pdset.df_ego[frameind, :time] ≤ pdset.df_ego[frameind-1, :time]
                return false
            end
        end
    end

    true
end
function test_pdset_for_common_problems(pdset::PrimaryDataset)
    are_sizes_consistent(pdset)
    @test is_time_ascending(pdset)
end

function set_ego!(
    pdset::PrimaryDataset,
    frameind::Integer,
    posGx::Float64,
    posGy::Float64,
    posGyaw::Float64,
    posFyaw::Float64,
    velFx::Float64,
    velFy::Float64,
    lanetag::LaneTag,
    curvature::Float64,
    d_cl::Float64,
    nll::Int,
    nlr::Int,
    d_ml::Float64,
    d_mr::Float64,
    d_merge::Float64=NA,
    d_split::Float64=NA,
    )

    sete!(pdset, :posGx,     frameind, posGx)
    sete!(pdset, :posGy,     frameind, posGy)
    sete!(pdset, :posGyaw,   frameind, posGyaw)
    sete!(pdset, :posFyaw,   frameind, posFyaw)
    sete!(pdset, :velFx,     frameind, velFx)
    sete!(pdset, :velFy,     frameind, velFy)
    sete!(pdset, :lanetag,   frameind, lanetag)
    sete!(pdset, :curvature, frameind, curvature)
    sete!(pdset, :d_cl,      frameind, d_cl)
    sete!(pdset, :nll,       frameind, nll)
    sete!(pdset, :nlr,       frameind, nlr)
    sete!(pdset, :d_ml,      frameind, d_ml)
    sete!(pdset, :d_mr,      frameind, d_mr)
    sete!(pdset, :d_merge,   frameind, d_merge)
    sete!(pdset, :d_split,   frameind, d_split)

    pdset
end
function set_other!(
    pdset::PrimaryDataset,
    carind::Integer,
    validfind::Integer,
    posGx::Float64,
    posGy::Float64,
    posGyaw::Float64,
    posFyaw::Float64,
    velFx::Float64,
    velFy::Float64,
    lanetag::LaneTag,
    curvature::Float64,
    d_cl::Float64,
    nll::Int,
    nlr::Int,
    d_ml::Float64,
    d_mr::Float64,
    d_merge::Float64=NA,
    d_split::Float64=NA,
    )

    setc!(pdset, :posGx,     carind, validfind, posGx)
    setc!(pdset, :posGy,     carind, validfind, posGy)
    setc!(pdset, :posGyaw,   carind, validfind, posGyaw)
    setc!(pdset, :posFyaw,   carind, validfind, posFyaw)
    setc!(pdset, :velFx,     carind, validfind, velFx)
    setc!(pdset, :velFy,     carind, validfind, velFy)
    setc!(pdset, :lanetag,   carind, validfind, lanetag)
    setc!(pdset, :curvature, carind, validfind, curvature)
    setc!(pdset, :d_cl,      carind, validfind, d_cl)
    setc!(pdset, :nll,       carind, validfind, nll)
    setc!(pdset, :nlr,       carind, validfind, nlr)
    setc!(pdset, :d_ml,      carind, validfind, d_ml)
    setc!(pdset, :d_mr,      carind, validfind, d_mr)
    setc!(pdset, :d_merge,   carind, validfind, d_merge)
    setc!(pdset, :d_split,   carind, validfind, d_split)

    pdset
end

pdset = create_empty_pdset()::PrimaryDataset

test_pdset_for_common_problems(pdset)
@test nframeinds(pdset) == 0
@test nvalidfinds(pdset) == 0
@test !validfind_inbounds(pdset, 0)
@test !validfind_inbounds(pdset, 1)
@test !frameind_inbounds(pdset, 0)
@test !frameind_inbounds(pdset, 1)

expand!(pdset)

test_pdset_for_common_problems(pdset)
@test nframeinds(pdset) == 1
@test nvalidfinds(pdset) == 1
@test !validfind_inbounds(pdset, 0)
@test validfind_inbounds(pdset, 1)
@test !frameind_inbounds(pdset, 0)
@test frameind_inbounds(pdset, 1)

@test isapprox(gete(pdset, :time, 1), 0.0)
@test gete(pdset, :frame, 1) == 1

expand!(pdset, 2)

test_pdset_for_common_problems(pdset)
@test nframeinds(pdset) == 3
@test nvalidfinds(pdset) == 3
@test !validfind_inbounds(pdset, 0)
@test validfind_inbounds(pdset, 1)
@test validfind_inbounds(pdset, 3)
@test !validfind_inbounds(pdset, 4)
@test !frameind_inbounds(pdset, 0)
@test frameind_inbounds(pdset, 1)
@test frameind_inbounds(pdset, 3)
@test !frameind_inbounds(pdset, 4)

@test isapprox(gete(pdset, :time, 1), 0.0)
@test isapprox(gete(pdset, :time, 2), DEFAULT_SEC_PER_FRAME)
@test isapprox(gete(pdset, :time, 3), 2DEFAULT_SEC_PER_FRAME)
@test gete(pdset, :frame, 1) == 1
@test gete(pdset, :frame, 2) == 2
@test gete(pdset, :frame, 3) == 3

@test !AutomotiveDrivingModels.RunLogs.idinframe(pdset, CARID_EGO, 0)
@test  AutomotiveDrivingModels.RunLogs.idinframe(pdset, CARID_EGO, 1)
@test  AutomotiveDrivingModels.RunLogs.idinframe(pdset, CARID_EGO, 2)
@test  AutomotiveDrivingModels.RunLogs.idinframe(pdset, CARID_EGO, 3)
@test !AutomotiveDrivingModels.RunLogs.idinframe(pdset, CARID_EGO, 4)

@test !AutomotiveDrivingModels.RunLogs.idinframe(pdset, 0, 0)
@test !AutomotiveDrivingModels.RunLogs.idinframe(pdset, 0, 1)
@test !AutomotiveDrivingModels.RunLogs.idinframe(pdset, 0, 2)
@test !AutomotiveDrivingModels.RunLogs.idinframe(pdset, 0, 3)
@test !AutomotiveDrivingModels.RunLogs.idinframe(pdset, 0, 4)

@test !indinframe(pdset, CARIND_EGO, 0)
@test  indinframe(pdset, CARIND_EGO, 1)
@test  indinframe(pdset, CARIND_EGO, 2)
@test  indinframe(pdset, CARIND_EGO, 3)
@test !indinframe(pdset, CARIND_EGO, 4)

@test !indinframe(pdset, 0, 0)
@test !indinframe(pdset, 0, 1)
@test !indinframe(pdset, 0, 2)
@test !indinframe(pdset, 0, 3)
@test !indinframe(pdset, 0, 4)

@test frameind2validfind(pdset, 1) == 1
@test frameind2validfind(pdset, 2) == 2
@test frameind2validfind(pdset, 3) == 3

@test validfind2frameind(pdset, 1) == 1
@test validfind2frameind(pdset, 2) == 2
@test validfind2frameind(pdset, 3) == 3

@test isapprox(calc_sec_per_frame(pdset), DEFAULT_SEC_PER_FRAME)
@test isapprox(get_elapsed_time(pdset, 1, 2),  DEFAULT_SEC_PER_FRAME)
@test isapprox(get_elapsed_time(pdset, 2, 1), -DEFAULT_SEC_PER_FRAME)
@test isapprox(get_elapsed_time(pdset, 1, 3), 2DEFAULT_SEC_PER_FRAME)

@test closest_frameind(pdset, -1.0DEFAULT_SEC_PER_FRAME) == 1
@test closest_frameind(pdset,  0.0DEFAULT_SEC_PER_FRAME) == 1
@test closest_frameind(pdset,  0.4DEFAULT_SEC_PER_FRAME) == 1
@test closest_frameind(pdset,  1.0DEFAULT_SEC_PER_FRAME) == 2
@test closest_frameind(pdset,  0.6DEFAULT_SEC_PER_FRAME) == 2
@test closest_frameind(pdset,  1.8DEFAULT_SEC_PER_FRAME) == 3
@test closest_frameind(pdset,  2.4DEFAULT_SEC_PER_FRAME) == 3

@test closest_validfind(pdset, -1.0DEFAULT_SEC_PER_FRAME) == 1
@test closest_validfind(pdset,  0.0DEFAULT_SEC_PER_FRAME) == 1
@test closest_validfind(pdset,  0.4DEFAULT_SEC_PER_FRAME) == 1
@test closest_validfind(pdset,  1.0DEFAULT_SEC_PER_FRAME) == 2
@test closest_validfind(pdset,  0.6DEFAULT_SEC_PER_FRAME) == 2
@test closest_validfind(pdset,  1.8DEFAULT_SEC_PER_FRAME) == 3
@test closest_validfind(pdset,  2.4DEFAULT_SEC_PER_FRAME) == 3

@test get_maxcarind(pdset, 1) == -1
@test get_maxcarind(pdset, 2) == -1
@test get_maxcarind(pdset, 3) == -1
@test get_carids(pdset) == Set{Int}([CARID_EGO])
@test carid_exists(pdset, CARID_EGO)
@test !carid_exists(pdset, 0)
@test !carid_exists(pdset, 1)

@test get_valid_frameinds(pdset) == [1,2,3]

add_car_slot!(pdset)
test_pdset_for_common_problems(pdset)
@test get_num_cars_in_frame(pdset, 1) == 1

set_ego!(pdset, 1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, LaneTag(1,2,3,4), 0.6,
                   0.7, 0, 1, 0.8, 0.9, 0.11, 0.12)

@test isapprox(gete(pdset, :posGx,     1), 0.0)
@test isapprox(gete(pdset, :posGy,     1), 0.1)
@test isapprox(gete(pdset, :posGyaw,   1), 0.2)
@test isapprox(gete(pdset, :posFyaw,   1), 0.3)
@test isapprox(gete(pdset, :velFx,     1), 0.4)
@test isapprox(gete(pdset, :velFy,     1), 0.5)
@test          gete(pdset, :lanetag,   1) == LaneTag(1,2,3,4)
@test isapprox(gete(pdset, :curvature, 1), 0.6)
@test isapprox(gete(pdset, :d_cl,      1), 0.7)
@test          gete(pdset, :nll,       1) == 0
@test          gete(pdset, :nlr,       1) == 1
@test isapprox(gete(pdset, :d_ml,      1), 0.8)
@test isapprox(gete(pdset, :d_mr,      1), 0.9)
@test isapprox(gete(pdset, :d_merge,   1), 0.11)
@test isapprox(gete(pdset, :d_split,   1), 0.12)

add_car_to_validfind!(pdset, 1, 1)
test_pdset_for_common_problems(pdset)
@test get_num_cars_in_frame(pdset, 1) == 2

set_other!(pdset, 0, 1, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18,
                  LaneTag(5,6,7,8), 0.19,
                  0.20, 2, 3, 0.21, 0.22, 0.23, 0.24)

@test isapprox(getc(pdset, :posGx,     0, 1), 0.13)
@test isapprox(getc(pdset, :posGy,     0, 1), 0.14)
@test isapprox(getc(pdset, :posGyaw,   0, 1), 0.15)
@test isapprox(getc(pdset, :posFyaw,   0, 1), 0.16)
@test isapprox(getc(pdset, :velFx,     0, 1), 0.17)
@test isapprox(getc(pdset, :velFy,     0, 1), 0.18)
@test          getc(pdset, :lanetag,   0, 1) == LaneTag(5,6,7,8)
@test isapprox(getc(pdset, :curvature, 0, 1), 0.19)
@test isapprox(getc(pdset, :d_cl,      0, 1), 0.20)
@test          getc(pdset, :nll,       0, 1) == 2
@test          getc(pdset, :nlr,       0, 1) == 3
@test isapprox(getc(pdset, :d_ml,      0, 1), 0.21)
@test isapprox(getc(pdset, :d_mr,      0, 1), 0.22)
@test isapprox(getc(pdset, :d_merge,   0, 1), 0.23)
@test isapprox(getc(pdset, :d_split,   0, 1), 0.24)

@test isapprox(get(pdset, :posGx,     CARIND_EGO, 1), 0.0)
@test isapprox(get(pdset, :posGy,     CARIND_EGO, 1), 0.1)
@test isapprox(get(pdset, :posGyaw,   CARIND_EGO, 1), 0.2)
@test isapprox(get(pdset, :posFyaw,   CARIND_EGO, 1), 0.3)
@test isapprox(get(pdset, :velFx,     CARIND_EGO, 1), 0.4)
@test isapprox(get(pdset, :velFy,     CARIND_EGO, 1), 0.5)
@test          get(pdset, :lanetag,   CARIND_EGO, 1) == LaneTag(1,2,3,4)
@test isapprox(get(pdset, :curvature, CARIND_EGO, 1), 0.6)
@test isapprox(get(pdset, :d_cl,      CARIND_EGO, 1), 0.7)
@test          get(pdset, :nll,       CARIND_EGO, 1) == 0
@test          get(pdset, :nlr,       CARIND_EGO, 1) == 1
@test isapprox(get(pdset, :d_ml,      CARIND_EGO, 1), 0.8)
@test isapprox(get(pdset, :d_mr,      CARIND_EGO, 1), 0.9)
@test isapprox(get(pdset, :d_merge,   CARIND_EGO, 1), 0.11)
@test isapprox(get(pdset, :d_split,   CARIND_EGO, 1), 0.12)

@test isapprox(get(pdset, :posGx,     0, 1), 0.13)
@test isapprox(get(pdset, :posGy,     0, 1), 0.14)
@test isapprox(get(pdset, :posGyaw,   0, 1), 0.15)
@test isapprox(get(pdset, :posFyaw,   0, 1), 0.16)
@test isapprox(get(pdset, :velFx,     0, 1), 0.17)
@test isapprox(get(pdset, :velFy,     0, 1), 0.18)
@test          get(pdset, :lanetag,   0, 1) == LaneTag(5,6,7,8)
@test isapprox(get(pdset, :curvature, 0, 1), 0.19)
@test isapprox(get(pdset, :d_cl,      0, 1), 0.20)
@test          get(pdset, :nll,       0, 1) == 2
@test          get(pdset, :nlr,       0, 1) == 3
@test isapprox(get(pdset, :d_ml,      0, 1), 0.21)
@test isapprox(get(pdset, :d_mr,      0, 1), 0.22)
@test isapprox(get(pdset, :d_merge,   0, 1), 0.23)
@test isapprox(get(pdset, :d_split,   0, 1), 0.24)