
for ticks in tick_list

	counts, unit = ticks_to_time_string(ticks)

	# Mean Turnrate
	fname_str = "MeanTurnRate$counts$unit"
	feature_name = symbol("Feature_" * fname_str)
	str_description  = "the mean turnrate (+ to left) for the last $counts $unit"
	sym_feature = symbol("meanturnrate$counts$unit")
	lstr = latexstring("\\bar{\\dot{\\psi}}_{$counts$unit}")

	create_feature_basics( fname_str, "rad/s", false, false, Inf, -Inf, true, sym_feature, lstr, str_description)

	@eval begin
		function _get(::$feature_name, pdset::PrimaryDataset, ::StreetNetwork, carind::Int, validfind::Int)

			maxjump = -$ticks

			val = 0.0
			count = 0

			carid = carind == CARIND_EGO ? CARID_EGO : carind2id(pdset, carind, validfind)

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

				count += 1
				val += (fut - cur)/(SEC_PER_FRAME*N_FRAMES_PER_SIM_FRAME)
			end

			if count == 0
				return NA_ALIAS
			end
			val / count
		end
		function _get(::$feature_name, basics::FeatureExtractBasicsPdSet, carind::Int, validfind::Int)

			pdset = basics.pdset
			maxjump = -$ticks

			val = 0.0
			count = 0

			carid = carind == CARIND_EGO ? CARID_EGO : carind2id(pdset, carind, validfind)

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

				count += 1
				val += (fut - cur)/(SEC_PER_FRAME*N_FRAMES_PER_SIM_FRAME)
			end

			if count == 0
				return NA_ALIAS
			end
			val / count
		end
	end

end

