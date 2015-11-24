using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors
using StreamStats

#=
1 - load the best inputs for every model
2 - train a final model on the test dataset
3 - save them
4 - compute validation metrics on the validation dataset
5 - save results to a .csv
=#

# For LOOCV2:
#=
- load full dataset
- construct split over drives
- run CV split get the best hyperparams for each model based on likelihood
- for each drive:
   - train a model on the other drives
   - compute train and test metrics (logl, emergent kldiv counts, rwse) on the withheld set
- aggregate the resuts
=#

##############################
# PARAMETERS
##############################

const INCLUDE_FILE_BASE = "realworld"
const N_SIMULATIONS_PER_TRACE = 50
const DEFAULT_TRACE_HISTORY = 2*DEFAULT_FRAME_PER_SEC
const N_BAGGING_SAMPLES = 10
const CONFIDENCE_LEVEL = 0.95

const MAX_CV_OPT_TIME_PER_MODEL = 60.0 # [s]
const AM_ON_TULA = gethostname() == "tula"
const INCLUDE_FILE = AM_ON_TULA ? joinpath("/home/wheelert/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl") :
                                  joinpath("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl")
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

################################
# METRICS
################################

metric_types_test_frames = [LoglikelihoodMetric]
metric_types_test_frames_bagged = [LoglikelihoodMetric]
metric_types_train_frames = [LoglikelihoodMetric]
metric_types_train_frames_bagged = [LoglikelihoodMetric]

metric_types_test_traces = [
                            EmergentKLDivMetric{symbol(SPEED)},
                            EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                            EmergentKLDivMetric{symbol(D_CL)},
                            RootWeightedSquareError{symbol(SPEED), 0.5},
                            RootWeightedSquareError{symbol(SPEED), 1.0},
                            RootWeightedSquareError{symbol(SPEED), 1.5},
                            RootWeightedSquareError{symbol(SPEED), 2.0},
                            RootWeightedSquareError{symbol(SPEED), 2.5},
                            RootWeightedSquareError{symbol(SPEED), 3.0},
                            RootWeightedSquareError{symbol(SPEED), 3.5},
                            RootWeightedSquareError{symbol(SPEED), 4.0},
                            RootWeightedSquareError{symbol(D_CL), 0.5},
                            RootWeightedSquareError{symbol(D_CL), 1.0},
                            RootWeightedSquareError{symbol(D_CL), 1.5},
                            RootWeightedSquareError{symbol(D_CL), 2.0},
                            RootWeightedSquareError{symbol(D_CL), 2.5},
                            RootWeightedSquareError{symbol(D_CL), 3.0},
                            RootWeightedSquareError{symbol(D_CL), 3.5},
                            RootWeightedSquareError{symbol(D_CL), 4.0},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 0.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.0},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.0},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.0},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.5},
                            RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 4.0},
                           ]
metric_types_test_traces_bagged = [
                                   EmergentKLDivMetric{symbol(SPEED)},
                                   EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                                   EmergentKLDivMetric{symbol(D_CL)},
                                   # RootWeightedSquareError{symbol(SPEED), 0.5},
                                   # RootWeightedSquareError{symbol(SPEED), 1.0},
                                   # RootWeightedSquareError{symbol(SPEED), 1.5},
                                   # RootWeightedSquareError{symbol(SPEED), 2.0},
                                   # RootWeightedSquareError{symbol(SPEED), 2.5},
                                   # RootWeightedSquareError{symbol(SPEED), 3.0},
                                   # RootWeightedSquareError{symbol(SPEED), 3.5},
                                   # RootWeightedSquareError{symbol(SPEED), 4.0},
                                   # RootWeightedSquareError{symbol(D_CL), 0.5},
                                   # RootWeightedSquareError{symbol(D_CL), 1.0},
                                   # RootWeightedSquareError{symbol(D_CL), 1.5},
                                   # RootWeightedSquareError{symbol(D_CL), 2.0},
                                   # RootWeightedSquareError{symbol(D_CL), 2.5},
                                   # RootWeightedSquareError{symbol(D_CL), 3.0},
                                   # RootWeightedSquareError{symbol(D_CL), 3.5},
                                   # RootWeightedSquareError{symbol(D_CL), 4.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 0.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 1.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 2.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.0},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 3.5},
                                   # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT), 4.0},
                                  ]

################################
# LOAD TRAIN AND VALIDATION SETS
################################

include(Pkg.dir("AutomotiveDrivingModels", "scripts", "model_params.jl"))
nmodels = length(behaviorset)

dset_filepath_modifier = "_subset_car_following"

METRICS_OUTPUT_FILE = joinpath(EVALUATION_DIR, "validation_results" * dset_filepath_modifier * ".jld")
MODEL_OUTPUT_JLD_FILE = joinpath(EVALUATION_DIR, "validation_models" * dset_filepath_modifier * ".jld")
TRAIN_VALIDATION_JLD_FILE = joinpath(EVALUATION_DIR, "train_validation_split" * dset_filepath_modifier * ".jld")
DATASET_JLD_FILE = joinpath(EVALUATION_DIR, "dataset" * dset_filepath_modifier * ".jld")

dset = JLD.load(DATASET_JLD_FILE, "model_training_data")

split_drives = get_fold_assignment_across_drives(dset)

params = BN_TrainParams(
  indicators = INDICATOR_SET,
  preoptimize_target_bins = true,
  preoptimize_indicator_bins = true,
  optimize_structure = true,
  optimize_target_bins = false,
  optimize_parent_bins = false,
  ncandidate_bins = 7,
  max_parents = 5
)

fold = 1
preallocated_data = BN_PreallocatedData(dset, params)
tic()
train(DynamicBayesianNetworkBehavior, dset, preallocated_data, params, fold, split_drives, false)
toc()

# println("DONE")
println("DONE")
exit()
