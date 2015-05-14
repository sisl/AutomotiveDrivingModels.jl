export export_to_text, trace_to_tex

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

     
#=
      \draw[->,>=stealth'] plot[smooth] coordinates {(1.420,1.766) (1.501,1.766)(1.909,1.770) (2.320,1.778) (2.737,1.789) (3.154,1.799) (3.573,1.807) (3.995,1.809) (4.421,1.808) (4.846,1.808) (5.269,1.805) (5.694,1.795) (6.124,1.780) (6.556,1.764) (6.981,1.745) (7.410,1.733) (7.844,1.724) (8.280,1.721) (8.719,1.722) (9.157,1.728) (9.596,1.735) (10.038,1.741) (10.508,1.725) (11.018,1.676) (11.501,1.646) (11.959,1.645) (12.429,1.628) (12.892,1.612) (13.341,1.614) (13.787,1.617) (14.236,1.623) (14.684,1.627) (15.118,1.634) (15.546,1.638) };
      \filldraw ($(2.737,1.789)+(-1pt,-1pt)$) rectangle ($(2.737,1.789)+(1pt,1pt)$);
      \filldraw ($(4.421,1.808)  +(-1pt,-1pt)$) rectangle ($(4.421,1.808)  +(1pt,1pt)$);
      \filldraw ($(6.124,1.780)  +(-1pt,-1pt)$) rectangle ($(6.124,1.780)  +(1pt,1pt)$);
      \filldraw ($(7.844,1.724)  +(-1pt,-1pt)$) rectangle ($(7.844,1.724)  +(1pt,1pt)$);
      \filldraw ($(9.596,1.735)  +(-1pt,-1pt)$) rectangle ($(9.596,1.735)  +(1pt,1pt)$);
      \filldraw ($(11.501,1.646) +(-1pt,-1pt)$) rectangle ($(11.501,1.646) +(1pt,1pt)$);
      \filldraw ($(13.341,1.614) +(-1pt,-1pt)$) rectangle ($(13.341,1.614) +(1pt,1pt)$);
      \filldraw ($(15.118,1.634) +(-1pt,-1pt)$) rectangle ($(15.118,1.634) +(1pt,1pt)$);
      \draw[->,>=stealth'] plot[smooth] coordinates {(0.500,0.157) (0.572,0.159)(0.927,0.169) (1.285,0.180) (1.643,0.189) (2.001,0.200) (2.361,0.209) (2.720,0.217) (3.080,0.222) (3.440,0.229) (3.799,0.226) (4.165,0.229) (4.529,0.232) (4.890,0.234) (5.253,0.239) (5.615,0.245) (5.974,0.246) (6.337,0.246) (6.702,0.247) (7.066,0.250) (7.427,0.260) (7.793,0.268) (8.161,0.270) (8.529,0.269) (8.897,0.266) (9.267,0.259) (9.639,0.252) (10.012,0.253) (10.385,0.252) (10.756,0.254) (11.128,0.250) (11.501,0.247) (11.877,0.247) (12.253,0.247) (12.630,0.247) (13.008,0.249) (13.385,0.250) (13.762,0.253) (14.141,0.254) (14.521,0.253) (14.903,0.251) (15.287,0.251) };
      \draw ($(1.643,0.189)  + (-1pt,-1pt)$) --  ($(1.643,0.189)  + (1pt,1pt)$);
      \draw ($(3.080,0.222)  + (-1pt,-1pt)$) --  ($(3.080,0.222)  + (1pt,1pt)$);
      \draw ($(4.529,0.232)  + (-1pt,-1pt)$) --  ($(4.529,0.232)  + (1pt,1pt)$);
      \draw ($(5.974,0.246)  + (-1pt,-1pt)$) --  ($(5.974,0.246)  + (1pt,1pt)$);
      \draw ($(7.427,0.260)  + (-1pt,-1pt)$) --  ($(7.427,0.260)  + (1pt,1pt)$);
      \draw ($(8.897,0.266)  + (-1pt,-1pt)$) --  ($(8.897,0.266)  + (1pt,1pt)$);
      \draw ($(10.385,0.252) + (-1pt,-1pt)$) --  ($(10.385,0.252) + (1pt,1pt)$);
      \draw ($(11.877,0.247) + (-1pt,-1pt)$) --  ($(11.877,0.247) + (1pt,1pt)$);
      \draw ($(13.385,0.250) + (-1pt,-1pt)$) --  ($(13.385,0.250) + (1pt,1pt)$);
      \draw ($(14.903,0.251) + (-1pt,-1pt)$) --  ($(14.903,0.251) + (1pt,1pt)$);
      \draw ($(1.643,0.189)  + (-1pt,-1pt)$) --  ($(1.643,0.189)  + (1pt,1pt)$);
      \draw ($(1.643,0.189)  + (-1pt, 1pt)$) --  ($(1.643,0.189)  + (1pt,-1pt)$);
      \draw ($(3.080,0.222)  + (-1pt, 1pt)$) --  ($(3.080,0.222)  + (1pt,-1pt)$);
      \draw ($(4.529,0.232)  + (-1pt, 1pt)$) --  ($(4.529,0.232)  + (1pt,-1pt)$);
      \draw ($(5.974,0.246)  + (-1pt, 1pt)$) --  ($(5.974,0.246)  + (1pt,-1pt)$);
      \draw ($(7.427,0.260)  + (-1pt, 1pt)$) --  ($(7.427,0.260)  + (1pt,-1pt)$);
      \draw ($(8.897,0.266)  + (-1pt, 1pt)$) --  ($(8.897,0.266)  + (1pt,-1pt)$);
      \draw ($(10.385,0.252) + (-1pt, 1pt)$) --  ($(10.385,0.252) + (1pt,-1pt)$);
      \draw ($(11.877,0.247) + (-1pt, 1pt)$) --  ($(11.877,0.247) + (1pt,-1pt)$);
      \draw ($(13.385,0.250) + (-1pt, 1pt)$) --  ($(13.385,0.250) + (1pt,-1pt)$);
      \draw ($(14.903,0.251) + (-1pt, 1pt)$) --  ($(14.903,0.251) + (1pt,-1pt)$);
      \draw[->,>=stealth'] plot[smooth] coordinates {(1.021,0.691) (1.081,0.696)(1.373,0.737) (1.708,0.770) (2.031,0.822) (2.354,0.862) (2.685,0.880) (3.010,0.900) (3.342,0.929) (3.686,0.969) (4.033,1.012) (4.382,1.050) (4.736,1.085) (5.095,1.115) (5.458,1.131) (5.826,1.137) (6.187,1.126) (6.557,1.128) (6.935,1.141) (7.311,1.153) (7.678,1.118) (8.048,1.099) (8.428,1.116) (8.810,1.133) (9.191,1.152) (9.573,1.174) (9.953,1.191) (10.335,1.213) (10.718,1.234) (11.102,1.253) (11.488,1.268) (11.874,1.275) (12.259,1.283) (12.648,1.292) (13.039,1.300) (13.430,1.304) (13.821,1.306) (14.204,1.286) (14.598,1.291) (14.995,1.295) (15.391,1.302) (15.787,1.306) };
      \filldraw ($(2.031,0.822)  + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(3.342,0.929)  + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(4.736,1.085)  + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(6.187,1.126)  + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(7.678,1.118)  + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(9.191,1.152)  + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(10.718,1.234) + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(12.259,1.283) + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(13.821,1.306) + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      \filldraw ($(15.391,1.302) + (-1pt,-0.45pt)$) -- ++(2pt,0pt) -- ++ (-1pt,1.5pt) -- cycle;
      
      \node [car,rotate=-0.095] at (2.247,0.722){$\blacktriangleright$};
      \node [car,rotate=-1.439] at (1.420,1.766){$\blacktriangleright$};
      \node [car,rotate=-0.138] at (0.500,0.157){$\blacktriangleright$};
      \node [car,rotate=0.814] at (1.021,0.691){$\blacktriangleright$};
    \end{tikzpicture}
=#