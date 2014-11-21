
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