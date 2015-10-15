using AutomotiveDrivingModels

using RandomForestBehaviors
using DynamicBayesianNetworkBehaviors

##############################
# PARAMETERS
##############################

# const INCLUDE_FILE_BASE = "vires_highway_2lane_sixcar"
const INCLUDE_FILE_BASE = "realworld"

const AM_ON_TULA = gethostname() == "tula"
const INCLUDE_FILE = AM_ON_TULA ? joinpath("/home/wheelert/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl") :
                                  joinpath("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels", INCLUDE_FILE_BASE, "extract_params.jl")
const INCLUDE_NAME = splitdir(splitext(INCLUDE_FILE)[1])[2]

include(INCLUDE_FILE)

################################
# LOAD TRAIN AND VALIDATION SETS
################################

df_orig  = readtable("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/df_train.csv")
df_clean = readtable("/media/tim/DATAPART1/PublicationData/2015_TrafficEvolutionModels/realworld/df_train_treated.csv")

##############################
# MODELS
##############################

# TODO(tim): load optimal behavior set params from file
behaviorset = BehaviorSet()
add_behavior!(behaviorset, VehicleBehaviorGaussian, "Static Gaussian")
add_behavior!(behaviorset, VehicleBehaviorLinearGaussian, "Linear Gaussian",
    [:indicators=>INDICATOR_SET,
     :ridge_regression_constant=>0.3157894736842105,
    ])
add_behavior!(behaviorset, GindeleRandomForestBehavior, "Random Forest",
    [:indicators=>INDICATOR_SET,
     :ntrees=>10,
     :max_depth=>20,
     :min_samples_split=>10,
     :min_samples_leaves=>8,
     :min_split_improvement=>0.0,
     :partial_sampling=>0.8,
     :n_split_tries=>100,
    ])
add_behavior!(behaviorset, DynamicForestBehavior, "Dynamic Forest",
    [:indicators=>INDICATOR_SET,
     :ntrees=>10,
     :max_depth=>20,
     :min_samples_split=>10,
     :min_samples_leaves=>8,
     :min_split_improvement=>0.0,
     :partial_sampling=>0.8,
     :n_split_tries=>100,
    ])
# add_behavior!(behaviorset, DynamicBayesianNetworkBehavior, "Bayesian Network",
#     [:indicators=>INDICATOR_SET,
#      :preoptimize_target_bins=>true,
#      :preoptimize_parent_bins=>true,
#      :optimize_structure=>true,
#      :optimize_target_bins=>false,
#      :optimize_parent_bins=>false,
#      :ncandidate_bins=>35,
#      :max_parents=>5,
#      # :nbins_lat=>5,
#      # :nbins_lon=>5,
#      ])

##############################
# METRICS
##############################

metric_types_test_frames = [LoglikelihoodMetric]
metric_types_test_traces = [
                            EmergentKLDivMetric{symbol(SPEED)},
                            EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                            EmergentKLDivMetric{symbol(D_CL)},
                            RootWeightedSquareError{symbol(SPEED), 1.0},
                            RootWeightedSquareError{symbol(SPEED), 2.0},
                            RootWeightedSquareError{symbol(SPEED), 3.0},
                            RootWeightedSquareError{symbol(SPEED), 4.0},
                            # RootWeightedSquareError{symbol(TIMEGAP_X_FRONT)},
                            # RootWeightedSquareError{symbol(D_CL)},
                           ]
metric_types_test_frames_bagged = [LoglikelihoodMetric]
metric_types_test_traces_bagged = [
                                   EmergentKLDivMetric{symbol(SPEED)},
                                   EmergentKLDivMetric{symbol(TIMEGAP_X_FRONT)},
                                   EmergentKLDivMetric{symbol(D_CL)},
                                   RootWeightedSquareError{symbol(SPEED), 1.0},
                                   RootWeightedSquareError{symbol(SPEED), 2.0},
                                   RootWeightedSquareError{symbol(SPEED), 3.0},
                                   RootWeightedSquareError{symbol(SPEED), 4.0},
                                  ]

metric_types_cv_train_frames = DataType[]
metric_types_cv_test_frames = [LoglikelihoodMetric]
metric_types_cv_train_traces = DataType[]
metric_types_cv_test_traces = DataType[]

##############################
# EVALUATION ORIG
##############################

nframes_tot = nrow(df_orig)
nframes_train = int(0.9*nframes_tot)

models = train_special(behaviorset, df_orig[1:nframes_train,:])


println("TEST LOGL ON ORIGINAL")
for i in 1 : length(behaviorset.names)

    behavior = models[i]
    name = behaviorset.names[i]

    logl = 0.0
    nframes = 0
    for frameind in [nframes_train+1 : nframes_tot]
        nframes += 1
        logl += calc_action_loglikelihood(behavior, df_orig, frameind)
    end
    logl /= nframes

    println("\t", name, ":  ", logl)
end


##############################
# EVALUATION TREATED
##############################

models = train_special(behaviorset, df_clean[1:nframes_train,:])

println("TEST LOGL ON TREATED")
for i in 1 : length(behaviorset.names)

    behavior = models[i]
    name = behaviorset.names[i]

    logl = 0.0
    nframes = 0
    for frameind in nframes_train+1 : nframes_tot
        nframes += 1
        logl += calc_action_loglikelihood(behavior, df_clean, frameind)
    end
    logl /= nframes

    println("\t", name, ":  ", logl)
end

# println("DONE")
println("DONE")