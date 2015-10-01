using AutomotiveDrivingModels
using AutomotiveDrivingModels.StreetNetworks.RoadNetwork

# for (RNDF_FILE, STREETMAP_OUTPUT_FILE) in (
#         ("/media/tim/DATAPART1/Data/Bosch/source2/detroit_v4_rndf_copy.txt",
#          "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/streetmap_detroit.jld"),
#         ("/media/tim/DATAPART1/Data/Bosch/source2/its_both_loops_v3_smoothed.txt",
#          "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/streetmap_its_both_loops_v3_smoothed.jld"),
#         ("/media/tim/DATAPART1/Data/Bosch/source2/its_inner_loop_v1_2_rndf.txt",
#          "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/streetmap_its_inner_loop_v1_2_rndf.jld"),
#         ("/media/tim/DATAPART1/Data/Bosch/philippruns_20150324/280_pagemill_to_92.txt",
#          "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/streetmap_280_pagemill_to_92.jld")
#     )

#     @printf("Loading RNDF... "); tic()
#     rndf = RoadNetwork.load_rndf(RNDF_FILE)
#     toc()

#     println(methods(AutomotiveDrivingModels.rndf2streetnetwork))
#     @printf("Building streetmap... "); tic()
#     sn = AutomotiveDrivingModels.rndf2streetnetwork(rndf)
#     toc()

#     @printf("Saving streetmap... "); tic()
#     save(STREETMAP_OUTPUT_FILE, "streetmap", sn)
#     toc()
# end

# NOTE(tim): this was back when I was inferring the 280 RNDF from the curves obtained from Jorge

for (CURVE_FILE, STREETMAP_OUTPUT_FILE) in (
        ("/media/tim/DATAPART1/Data/Bosch/source/curves/280N_big_rndf-curves.csv",
         "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/streetmap_280N_big.jld"),
        ("/media/tim/DATAPART1/Data/Bosch/source/curves/280S_big_rndf-curves.csv",
         "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/streetmap_280S_big.jld"),
    )

    @printf("Loading Curves... "); tic()
    curves = Curves.load(CURVE_FILE)
    toc()

    @printf("Building streetmap... "); tic()
    rndf = curves2rndf(curves)
    curves = []
    sn = AutomotiveDrivingModels.rndf2streetnetwork(rndf, convert_ll2utm=false)
    toc()

    @printf("Saving streetmap... "); tic()
    save(STREETMAP_OUTPUT_FILE, "streetmap", sn)
    toc()
end