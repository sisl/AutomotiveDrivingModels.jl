export 
        trace_to_tex,

        load_tracesets_carfollow,
        load_tracesets_freeflow


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