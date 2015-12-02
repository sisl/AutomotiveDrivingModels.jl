
for ticks in tick_list_short

	counts, unit = ticks_to_time_string(ticks)

	# Past Acceleration
	fname_str = "PastVelFy$counts$unit"
	feature_name = symbol("Feature_" * fname_str)
	str_description  = "the lateral velocity $counts $unit ago"
	sym_feature = symbol("pastvelFy$counts$unit")
	lstr = latexstring("v^F_{y,\\text{-$counts$unit}}")

	create_feature_basics( fname_str, "m/s", false, false, Inf, -Inf, true, sym_feature, lstr, str_description)

	@eval begin
		function _get(::$feature_name, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

			pdset = basics.pdset
			jump = -$ticks
			carid = carind == CARIND_EGO ? CARID_EGO : carind2id(pdset, carind, validfind)

			jvfind1 = convert(Int, jumpframe(pdset, validfind, jump))
			if jvfind1 == 0 # Does not exist
				return NA_ALIAS
			end

			curr = 0.0
			if carind == CARIND_EGO
				return get(VELFY, basics, CARIND_EGO, jvfind1)
			elseif idinframe(pdset, carid, jvfind1)
				ind = carid2ind(pdset, carid, jvfind1)
				return get(VELFY, basics, ind, jvfind1)
			else
				return NA_ALIAS
			end
		end
	end


end

