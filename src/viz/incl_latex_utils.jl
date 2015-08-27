push!(LOAD_PATH, "/home/tim/Documents/wheelerworkspace/UtilityCode/")

using LaTeXeXport
using DataFrames

function write_model_comparison_bar_plot_rmse(io::IO, df::DataFrame)

    # \addplot 
    #   coordinates {(rmse1s,0.1114) (rmse2s,0.3678)
    #        (rmse3s,0.7497) (rmse4s,1.2379)};

    # \addplot 
    #   coordinates {(rmse1s,0.1082) (rmse2s,0.3168) 
    #       (rmse3s,0.6191) (rmse4s,1.0054)};

    # \addplot 
    #   coordinates {(rmse1s,0.1568) (rmse2s,0.6923) 
    #       (rmse3s,1.6704) (rmse4s,3.0611)};

    # \legend{BN Model Small,Gaussian Filter,Random Forest}


    dfnames = names(df)
    dfrownames = df[:,1]
    row_rmse1s = findfirst(dfrownames, "mean rmse 1s")
    row_rmse2s = findfirst(dfrownames, "mean rmse 2s")
    row_rmse3s = findfirst(dfrownames, "mean rmse 3s")
    row_rmse4s = findfirst(dfrownames, "mean rmse 4s")

    for col = 3 : length(df)
        print(io, "\\addplot coordinates{\n")
        @printf(io, "\t(rmse1s,%s) (rmse2s,%s) (rmse3s,%s) (rmse4s,%s)};\n", 
                df[row_rmse1s, col], df[row_rmse2s, col], df[row_rmse3s, col], df[row_rmse4s, col])
    end

    print(io, "\\legend{")
    for col = 3 : length(df)
        modelname = replace(string(dfnames[col]), "_", " ")
        print(io, modelname)
        if col != length(df)
            print(io, ",")
        end
    end
    print(io, "}\n")
end
function write_model_comparison_bar_plot_kldiv(io::IO, df::DataFrame)
    dfnames = names(df)
    dfrownames = df[:,1]
    row_laneoffset = findfirst(dfrownames, "mean lane offset kldiv")
    row_speed = findfirst(dfrownames, "mean speed kldiv")
    row_timegap = findfirst(dfrownames, "mean timegap kldiv")
    row_histobin = findfirst(dfrownames, "histobin kldiv")

    baseline = :Random_Forest
    base_laneoffset = float(df[row_laneoffset, baseline])
    base_speed = float(df[row_speed, baseline])
    base_timegap = float(df[row_timegap, baseline])
    base_histobin = float(df[row_histobin, baseline])

    for col = 3 : length(df)
        print(io, "\\addplot coordinates{\n")
        @printf(io, "\t(laneoffset,%.4f) (speed,%.4f) (timegap,%.4f) (histobin,%.4f)};\n", 
                float(df[row_laneoffset, col])/base_laneoffset, 
                float(df[row_speed, col])/base_speed,
                float(df[row_timegap, col])/base_timegap, 
                float(df[row_histobin, col])/base_histobin)
    end

    print(io, "\\legend{")
    for col = 3 : length(df)
        modelname = replace(string(dfnames[col]), "_", " ")
        print(io, modelname)
        if col != length(df)
            print(io, ",")
        end
    end
    print(io, "}\n")
end
function export_tikz_frenet(log)

    const ROAD_NLANES = 4
    const ROAD_LANE_WIDTH = 3.7
    const ROAD_WIDTH = ROAD_NLANES * ROAD_LANE_WIDTH
    const IMAGE_WIDTH = 17.0
    const IMAGE_HEIGHT = 2.0
    const IMAGE_BORDER = 0.5
    const SCALE_Y_M = IMAGE_HEIGHT / ROAD_WIDTH
    const SCALE_Y_B = 0.5IMAGE_HEIGHT / ROAD_NLANES

    const PRINT_SKIP = 8

    traces = (Float64, Matrix{Float64})[]

    for carind = 1 : ncars(log)
        
        baseind = logindexbase(carind)
        posFyaw = log[1,baseind+LOG_COL_ϕ]
        track   = log[:,[baseind+LOG_COL_X,baseind+LOG_COL_Y]]
        push!(traces, (posFyaw, track'))
    end

    # ------------------

    max_pos_x = -Inf
    min_pos_x =  Inf
    for (ϕ,track) in traces
        max_pos_x = max(max_pos_x, maximum(track[1,:]))
        min_pos_x = min(min_pos_x, minimum(track[1,:]))
    end
    scale_m = (IMAGE_WIDTH-2IMAGE_BORDER) / (max_pos_x - min_pos_x)
    scale_b = IMAGE_BORDER - min_pos_x * scale_m

    for (i,tup) in enumerate(traces)
        ϕ, track = tup[1], tup[2]
        track[1,:] = track[1,:] .* scale_m .+ scale_b
        track[2,:] = track[2,:] .*SCALE_Y_M .+ SCALE_Y_B
        @printf("\\draw[->,>=stealth'] plot[smooth] coordinates {")
        for i = 1 : size(track,2)
            @printf("(%.3f,%.3f)", track[1,i], track[2,i])
            if i != length(size(track))
                @printf(" ")
            end
        end
        println("};")
        for i = (PRINT_SKIP+1) : PRINT_SKIP : size(track,2)
            @printf("\\filldraw (%.3f,%.3f) circle (1pt);\n", track[1,i], track[2,i])
        end
    end
    for (i,tup) in enumerate(traces)
        ϕ, track = tup[1], tup[2]
        @printf("\\node [car,rotate=%.3f] at (%.3f,%.3f){\$\\blacktriangleright\$};\n", rad2deg(ϕ), track[1,1], track[2,1])
    end
end

texfile = "/home/tim/Documents/presentations/weekly/2015_SUM_01/main2.tex"
df = readtable("/home/tim/Documents/presentations/weekly/2015_SUM_01/data_model_compare.csv")
write_to_texthook(fh->write_model_comparison_bar_plot_rmse(fh, df), texfile, "model_compare_rmse")
write_to_texthook(fh->write_model_comparison_bar_plot_kldiv(fh, df), texfile, "model_compare_kldiv")