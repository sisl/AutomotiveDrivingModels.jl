
for ticks in tick_list

	if ticks <= 2
		continue
	end

	counts, unit = ticks_to_time_string(ticks)

	# Standard deviation of Turnrate
	fname_str    = "StdTurnRate$counts$unit"
	feature_name = symbol("Feature_" * fname_str)
	str_description  = "the standard deviation of turnrate for the last $counts $unit"
	sym_feature = symbol("stdturnrate$counts$unit")
	lstr = latexstring("\\sigma\\left(\\dot{\\psi}\\right)_{$counts$unit}")

	create_feature_basics( fname_str, "rad/s", false, false, Inf, 0.0, true, sym_feature, lstr, str_description)

	@eval begin
		function _get(::$feature_name, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

			pdset = basics.pdset
			maxjump = -$ticks
			vals    = Float64[]
			carid   = carind == CARIND_EGO ? CARID_EGO : carind2id(pdset, carind, validfind)

			for jump = -N_FRAMES_PER_SIM_FRAME:-N_FRAMES_PER_SIM_FRAME:maxjump

				jvfind1 = jumpframe(pdset, validfind, jump)
				jvfind2 = jumpframe(pdset, validfind, jump-N_FRAMES_PER_SIM_FRAME)
				if jvfind1 == 0 || jvfind2 == 0 # Does not exist
					continue
				end

				cur, fut = 0.0, 0.0
				if carind == CARIND_EGO
					f1 = validfind2frameind(pdset, jvfind1)
					f2 = validfind2frameind(pdset, jvfind2)
					cur = gete(pdset, :posFyaw, f1)
					fut = gete(pdset, :posFyaw, f2)
				elseif idinframe(pdset, carid, jvfind1) && idinframe(pdset, carid, jvfind2)
					ind1 = carid2ind(pdset, carid, jvfind1)
					ind2 = carid2ind(pdset, carid, jvfind2)
					cur = getc(pdset, :posFyaw, ind1, jvfind1)
					fut = getc(pdset, :posFyaw, ind2, jvfind2)	
				else
					continue
				end

				push!(vals, (fut - cur)/(DEFAULT_SEC_PER_FRAME*N_FRAMES_PER_SIM_FRAME))
			end

			if length(vals) <= 2
				return NA_ALIAS
			end
			std(vals)
		end
	end

end

