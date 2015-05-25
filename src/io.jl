export 
        export_to_text,
        trace_to_tex,

        load_em,

        load_tracesets_carfollow,
        load_tracesets_freeflow,

        print_structure

function export_to_text(em::EM, filename::String)
	# export a bayesian network to an encounter definition file 

	n = em.n # the number of nodes

	open(filename, "w") do fout
	  	
		# Labels: the node names
		###############################################
		println(fout, "# labels")
		for i = 1 : n
			@printf(fout, "\"%s\"", string(em.labels[i]))
			if i < n
				@printf(fout, ", ")
			else
				@printf(fout, "\n")
			end
		end

		# G: the graphical structure
		##################################################
		println(fout, "# G: graphical structure")
		for i = 1 : n
			for j = 1 : n
				@printf(fout, "%c ", em.G[i,j] ? '1' : '0')
			end
			@printf(fout, "\n")
		end

		# r: number of bins for each variable
		##################################################
		println(fout, "# r: number of bins")
		for i = 1 : n
			@printf(fout, "%d ", em.r[i])
		end
		@printf(fout, "\n")

		# N: the sufficient statistics, in integer form
		##################################################
		println(fout, "# N: sufficient statistics")
		for i = 1 : n
			stats = em.N[i]
			r, q = size(stats)
			for b = 1:q, a = 1:r
				@printf(fout, "%d ", stats[a,b])
			end
		end
		@printf(fout, "\n")

		# binmaps, '*' if discrete
		##################################################
		@printf(fout, "# binmaps\n")
		for i = 1 : n
			bmap = em.binmaps[i]
			
			if isa(bmap, LabelMap)
				@printf(fout, "*")
			elseif isa(bmap, DataBinMap) || isa(bmap, BinMap)
				for e in bmap.binedges
					@printf(fout, "%f ", e)
				end
			end
			@printf(fout, "\n")
		end
	end
end
function trace_to_tex(runlog::Matrix{Float64}, road::StraightRoadway, filename::String;
	frameskip :: Int = 8 # how many frames we iterate by
	)

	const N_FRAMES   = size(runlog, 1)
	const SCALE_X    = 13.0 # length of x_direction before extra border is added
	const SCALE_Y    =  2.0 # width of road
	const OFFSET_X   =  0.5

	min_x, max_x = Inf, -Inf
	for carind = 1 : ncars(runlog)
		base = logindexbase(carind)
		for frameind = 1 : N_FRAMES
			x = runlog[frameind, base + LOG_COL_X]
			if x < min_x
				min_x = x
			end
			if x > max_x
				max_x = x
			end
		end
	end

	scale_x = SCALE_X/(max_x - min_x)
	scale_y = SCALE_Y/(road.nlanes * road.lanewidth)

	min_xs = min_x * scale_x - OFFSET_X
	max_xs = max_x * scale_x + OFFSET_X

	open(filename, "w") do fout
	
		println(fout, "\\begin{tikzpicture}")
		println(fout, "\t\\tikzset{")
		println(fout, "\t\tcar/.style={minimum height=2.5mm, minimum width=5.0mm, inner sep=0pt, draw=black,fill=white,rectangle,rounded corners=2pt, font={\\tiny}},")
		println(fout, "\t}\n")

		# draw the road markers
		for (i,y) in enumerate(laneborders(road))
			ys = y * scale_y
			if i != 1 && i != length(laneborders(road))
				@printf(fout, "\t\\draw [dashed] (%.3f,%.3f) -- (%.3f,%.3f);\n", min_xs, ys, max_xs, ys)
			else
				@printf(fout, "\t\\draw (%.3f,%.3f) -- (%.3f,%.3f);\n", min_xs, ys, max_xs, ys)
			end
		end

		last_frame = 0

		# render each car's trace
		for carind = 1 : ncars(runlog)
			base = logindexbase(carind)

			@printf(fout, "\t\\draw[->,>=stealth'] plot[smooth] coordinates { ")
			for frameind = 1 : N_FRAMES
				x = runlog[frameind, base + LOG_COL_X]
				y = runlog[frameind, base + LOG_COL_Y]
				xs = x * scale_x
				ys = y * scale_y
				@printf(fout, "(%.3f,%.3f)\t", xs, ys)
			end
			@printf(fout, " };\n")

			for frameind = 1 : frameskip : N_FRAMES
				x = runlog[frameind, base + LOG_COL_X]
				y = runlog[frameind, base + LOG_COL_Y]
				xs = x * scale_x
				ys = y * scale_y
				@printf(fout, "\t\\filldraw (%.3f,%.3f) circle (1pt);\n", xs, ys)
				last_frame = max(last_frame, frameind)
			end

		end

		# render all of the cars
		for carind = 1 : ncars(runlog)
			base = logindexbase(carind)
			x = runlog[last_frame, base + LOG_COL_X]
			y = runlog[last_frame, base + LOG_COL_Y]
			xs = x * scale_x
			ys = y * scale_y
			yaw = runlog[last_frame, base + LOG_COL_ϕ]
			@printf(fout, "\t\\node [car,rotate=%.4f] at (%.3f,%.3f){\$\\blacktriangleright\$};\n", yaw, xs, ys)
		end

		println(fout, "\\end{tikzpicture}\n")
	end
end

function load_em(
    discretizerdict :: Dict{Symbol, AbstractDiscretizer},
    targets    :: Vector{AbstractFeature},
    indicators :: Vector{AbstractFeature},
    stats      :: Vector{Matrix{Float64}}, 
    adj        :: BitMatrix
    )

    bn = build_bn(targets, indicators, stats, adj)
    em = encounter_model(bn, stats, discretizerdict, targets, indicators)
end
function load_em(
    binmapdict :: Dict{Symbol, AbstractBinMap},
    targets    :: Vector{AbstractFeature},
    indicators :: Vector{AbstractFeature},
    stats      :: Vector{Matrix{Float64}}, 
    adj        :: BitMatrix
    )

    discretizerdict = Dict{Symbol, AbstractDiscretizer}()

    for f in [targets, indicators]
        sym = symbol(f)
        bmap = binmapdict[sym]
        
        if isa(bmap, BinMap)
            discretizerdict[sym] = LinearDiscretizer(bmap.binedges, bmap.nbins, bmap.i2bin, bmap.bin2i, bmap.force_outliers_to_closest)
        elseif isa(bmap, LabelMap)
            discretizerdict[sym] = CategoricalDiscretizer(bmap.v2i, bmap.i2v)
        elseif isa(bmap, DataBinMap)
            cat = CategoricalDiscretizer([Inf=>1])
            lin = LinearDiscretizer(bmap.binedges, bmap.nbins-1, bmap.i2bin, bmap.bin2i, bmap.force_outliers_to_closest)
            discretizerdict[sym] = HybridDiscretizer(cat, lin)
        else
            println(sym, "\n\t", typeof(bmap))
            error("unknown type!")
        end
    end

    load_em(discretizerdict, targets, indicators, stats, adj)
end
function load_em(emstats::Dict{String, Any})
    
    binmapdict = emstats["binmaps"]
    targets    = emstats["targets"]
    indicators = emstats["indicators"]
    stats      = emstats["statistics"]
    adj        = emstats["adjacency"]

    load_em(binmapdict, convert(Vector{AbstractFeature}, targets), 
                        convert(Vector{AbstractFeature}, indicators), 
                        stats, adj)  
end
function load_em(empstats_file::String)
    emstats = load(empstats_file)
    
    binmapdict = emstats["binmaps"]
    targets    = emstats["targets"]
    indicators = emstats["indicators"]
    stats      = emstats["statistics"]
    adj        = emstats["adjacency"]

    load_em(binmapdict, convert(Vector{AbstractFeature}, targets), 
                        convert(Vector{AbstractFeature}, indicators), 
                        stats, adj)
end

function _load_tracesets(filematch::Regex)
    traces = Vector{VehicleTrace}[]
    for blob in readdir(TRACE_DIR)
        if ismatch(filematch, blob)
            append!(traces, load(TRACE_DIR*blob, "traces"))
        end
    end
    traces
end
load_tracesets_carfollow() = _load_tracesets(r"^carfollow(\S)*.jld")
load_tracesets_freeflow() = _load_tracesets(r"^freeflow(\S)*.jld")

function print_structure(em::EM)
    target = get_target_lat(em)
    println("target lat: ", symbol(target))
    for indicator in get_indicators_lat(em, target)
        println("\t", symbol(indicator))
    end

    target = get_target_lon(em)
    println("target lon: ", symbol(target))
    for indicator in get_indicators_lon(em, target)
        println("\t", symbol(indicator))
    end
end