
for ticks in tick_list_short

	counts, unit = ticks_to_time_string(ticks)

	# Past Acceleration
	fname_str = "PastAcc$counts$unit"
	feature_name = symbol("Feature_" * fname_str)
	str_description  = "the longitudinal acceleration $counts $unit ago"
	sym_feature = symbol("pastacc$counts$unit")
	lstr = latexstring("{a}^\\text{past}_{-$counts$unit}")

	create_feature_basics(fname_str, "m/s2", false, false, Inf, -Inf, true, sym_feature, lstr, str_description)

	@eval begin
		function _get(::$feature_name, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

			pdset = basics.pdset
			jump = -$ticks
			carid = carind == CARIND_EGO ? CARID_EGO : carind2id(pdset, carind, validfind)


			validfind_past1 = int(jumpframe(pdset, validfind, jump))
			validfind_past2 = int(jumpframe(pdset, validfind, jump-N_FRAMES_PER_SIM_FRAME))
			if validfind_past1 == 0 || validfind_past2 == 0 # Does not exist
				return NA_ALIAS
			end

			cur, fut = 0.0, 0.0
			if carind == CARIND_EGO
				curr = get(SPEED, basics, CARIND_EGO, validfind_past1)
				past = get(SPEED, basics, CARIND_EGO, validfind_past2)
			elseif idinframe(pdset, carid, validfind_past1) && idinframe(pdset, carid, validfind_past2)
				ind1 = carid2ind(pdset, carid, validfind_past1)
				ind2 = carid2ind(pdset, carid, validfind_past2)
				curr = get(SPEED, basics, ind1, validfind_past1)
				past = get(SPEED, basics, ind2, validfind_past2)
			else
				return NA_ALIAS
			end

			(curr - past)/(DEFAULT_SEC_PER_FRAME*N_FRAMES_PER_SIM_FRAME)
		end
	end


end

