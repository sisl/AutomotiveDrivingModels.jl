
for ticks in tick_list

	if ticks <= 2
		continue
	end

	counts, unit = ticks_to_time_string(ticks)

	# Standard deviation of accel F x
	fname_str = "StdAccFx$counts$unit"
	feature_name = symbol("Feature_" * fname_str)
	str_description  = "the standard deviation of acceleration along the lane for the last $counts $unit"
	sym_feature = symbol("stdaccFx$counts$unit")
	lstr = latexstring("\\sigma\\left(a^F_x\\right)_{$counts$unit}")

	create_feature_basics( fname_str, "m/s2", false, false, Inf, 0.0, true, sym_feature, lstr, str_description)

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
					cur = gete(pdset, :velFx, f1)
					fut = gete(pdset, :velFx, f2)
				elseif idinframe(pdset, carid, jvfind1) && idinframe(pdset, carid, jvfind2)
					ind1 = carid2ind(pdset, carid, jvfind1)
					ind2 = carid2ind(pdset, carid, jvfind2)
					cur = getc(pdset, :velFx, ind1, jvfind1)
					fut = getc(pdset, :velFx, ind2, jvfind2)
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

	# Standard deviation of accel F y
	fname_str = "StdAccFy$counts$unit"
	feature_name = symbol("Feature_" * fname_str)
	str_description  = "the standard deviation of acceleration perpendicular to the lane for the last $counts $unit"
	sym_feature = symbol("stdaccFy$counts$unit")
	lstr = latexstring("\\sigma\\left(a^F_y\\right)_{$counts$unit}")

	create_feature_basics( fname_str, "m/s2", false, false, Inf, 0.0, true, sym_feature, lstr, str_description)

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
					cur = gete(pdset, :velFy, f1)
					fut = gete(pdset, :velFy, f2)
				elseif idinframe(pdset, carid, jvfind1) && idinframe(pdset, carid, jvfind2)
					ind1 = carid2ind(pdset, carid, jvfind1)
					ind2 = carid2ind(pdset, carid, jvfind2)
					cur = getc(pdset, :velFy, ind1, jvfind1)
					fut = getc(pdset, :velFy, ind2, jvfind2)
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

