# Road Network

# Copyright (c) 2014 Tim wheeler, Robert Bosch Gmbh

# Module for the use of rndf, or Road Network files


module RoadNetwork

export
	RNDF,
	RNDF_Segment,
	RNDF_Lane,
	WaypointID,
	LaneID,
	load_rndf

# constants
# ==============================================

const REGEX_LANEID   = r"\d+.\d+"
const REGEX_WAYPOINT = r"\d+.\d+.\d+"
const REGEX_UTM_FLOAT = r" [-+]?[\d]+\.[\d]+"
const REGEX_WAYPOINT_LINE = r"\d+.\d+.\d+ [-+]?[\d]+\.[\d]+ [-+]?[\d]+\.[\d]+ [-+]?[\d]+\.[\d]+"
const REGEX_WAYPOINT_LINE_NOALT = r"\d+.\d+.\d+ [-+]?[\d]+\.[\d]+ [-+]?[\d]+\.[\d]+ \d+"

const VERBOSITY = 0

# types
# ==============================================

immutable LaneID
	segment :: UInt
	lane    :: UInt
end
immutable WaypointID
	segment :: UInt
	lane    :: UInt
	pt      :: UInt
end
immutable LatLonAlt
	lat :: Float64 # [deg]
	lon :: Float64 # [deg]
	alt :: Float64 # [m]

	function LatLonAlt(lat::Float64, lon::Float64, alt::Float64 = 0.0)
		if lat < -180.0 || lat > 180.0
			# warn("latitude out of range: $lat")
		elseif lon < -90.0 || lon > 90.0
			# warn("longitude out of range: $lon")
		elseif alt < -400.0 || alt > 9000.0
			# warn("altitude out of range: $alt")
		end
		new(lat, lon, alt)
	end
end

type RNDF_Lane
	id             :: LaneID

	width          :: Float64 # > 0, optional
	speed_limit    :: Tuple{Float64, Float64} # [mph] speed_limit MAX MIN allowed in segment
	boundary_left  :: Symbol # ∈ {:double_yellow, :solid_yellow, :solid_white, :broken_white, :unknown}
	boundary_right :: Symbol # ∈ {:double_yellow, :solid_yellow, :solid_white, :broken_white, :unknown}
	# checkpoints    :: Vector{(Int, Int)} # list of checkpoints (index, checkpoint_id)
	merge_link     :: Tuple{Int, WaypointID} # (id → id2) denotes a section of road where merging logic may be used
	merge_end_link :: Tuple{Int, WaypointID} # (id → id2)
	stops          :: Vector{Int} # list of waypoints associated with stop signs
	exits          :: Vector{Tuple{Int, WaypointID}} # list of waypoints associated with exits

	waypoints      :: Dict{Int, LatLonAlt} # col is <lat,lon>

	function RNDF_Lane(id::LaneID = LaneID(-1,-1))
		self = new()
		self.id = id
		self.width = NaN
		self.speed_limit = (NaN, NaN)
		self.boundary_left = self.boundary_right = :unknown
		self.merge_link = (0,WaypointID(0,0,0))
		self.merge_end_link = (0,WaypointID(0,0,0))
		self.stops = Int[]
		self.exits = Tuple{Int, WaypointID}[]
		self.waypoints = Dict{Int, LatLonAlt}()
		self
	end
end
type RNDF_Segment
	id         :: Int # > 0
	name       :: AbstractString # optional
	lanes      :: Dict{Int, RNDF_Lane} # id -> lane

	RNDF_Segment(id::Int = -1) = new(id,"",Dict{Int, RNDF_Lane}())
end
type RNDF
	name           :: AbstractString
	format_version :: AbstractString # optional
	creation_date  :: AbstractString # optional
	segments       :: Dict{Int, RNDF_Segment} # id -> segment

	RNDF() = new("UNNAMED","","", Dict{Int, RNDF_Segment}())
end

# functions
# ==============================================

Base.show(io::IO, id::LaneID) = @printf(io, "%d.%d", convert(Int, id.segment), convert(Int, id.lane))
Base.show(io::IO, id::WaypointID) = @printf(io, "%d.%d.%d", convert(Int, id.segment), convert(Int, id.lane), convert(Int, id.pt))
Base.show(io::IO, pt::LatLonAlt) = @printf(io, "(%8.4f, %9.4f, %8.2f)", pt.lat, pt.lon, pt.alt)

get_segment(rndf::RNDF, id::Int) = rndf.segments[id]::RNDF_Segment
get_lane(rndf::RNDF, id::LaneID) = rndf.segments[id.segment].lanes[id.lane]::RNDF_Lane
get_lane(rndf::RNDF, id::WaypointID) = rndf.segments[id.segment].lanes[id.lane]::RNDF_Lane
get_waypoint(rndf::RNDF, id::WaypointID) = rndf.segments[id.segment].lanes[id.lane].waypoints[id.pt]::LatLonAlt

Base.hash(id::LaneID, h::UInt=zero(UInt)) = hash(id.segment, hash(id.lane, h))
Base.hash(id::WaypointID, h::UInt=zero(UInt)) = hash(id.segment, hash(id.lane, hash(id.pt, h)))

function add_segment!(rndf::RNDF, id::Int)
	!haskey(rndf.segments, id) || error("rndf already has segment $id")
	rndf.segments[id] = RNDF_Segment(id)
	return rndf
end
function add_segment!(rndf::RNDF, seg::RNDF_Segment)
	!haskey(rndf.segments, seg.id) || error("rndf already has segment $(seg.id)")
	rndf.segments[seg.id] = seg
	return rndf
end

function add_lane!(seg::RNDF_Segment, id::LaneID)
	!haskey(seg.lanes, id.lane) || error("segment already has lane $(id.lane)")
	seg.lanes[id.lane] = RNDF_Lane(id)
	return seg
end
add_lane!(seg::RNDF_Segment, lane_id::Integer) = add_lane!(seg, LaneID(seg.id, lane_id))
function add_lane!(rndf::RNDF, id::LaneID)
	if !haskey(rndf.segments, id.segment)
		add_segment!(rndf, id.segment)
	end
	add_lane!(rndf.segments[id.segment], id)
	return rndf
end
function add_lane!(rndf::RNDF, id::WaypointID)
	if !haskey(rndf.segments, id.segment)
		add_segment!(rndf, int(id.segment))
	end
	add_lane!(rndf.segments[id.segment], int(id.lane))
	return rndf
end
function add_lane!(rndf::RNDF, lane::RNDF_Lane)
	seg_id, lane_id = lane.id.segment, lane.id.lane
	if !haskey(rndf.segments, seg_id)
		add_segment!(rndf, seg_id)
	end
	seg = rndf.segments[seg_id]
	!haskey(seg.lanes, lane_id) || error("segment already has lane $lane_id")
	seg.lanes[lane_id] = lane
	return rndf
end
add_lane!(rndf::RNDF, segment_id::Integer, lane_id::Integer) = add_lane!(rndf, LaneID(segment_id, lane_id))

function add_waypoint!(lane::RNDF_Lane, waypoint::WaypointID, lla::LatLonAlt; override::Bool=false)
	(lane.id.lane == waypoint.lane && lane.id.segment == waypoint.segment) || error("waypoint $waypoint does not match lane $(lane.id)")
	override || !haskey(lane.waypoints, waypoint.pt) || error("lane $(lane.id) already contains waypoint $(waypoint)")
	lane.waypoints[waypoint.id] = lla
	return lane
end
function add_waypoint!(lane::RNDF_Lane, pt_id::Integer, lla::LatLonAlt; override::Bool=false)
	override || !haskey(lane.waypoints, pt_id) || error("lane already contains waypoint $(pt_id)")
	lane.waypoints[pt_id] = lla
	return lane
end
function add_waypoint!(seg::RNDF_Segment, waypoint::WaypointID, lla::LatLonAlt; override::Bool=false)
	seg.id == waypoint.segment || error("segment $(seg.id) does not match waypoint $(waypoint)")
	if !haskey(seg.lanes, waypoint.lane)
		add_lane!(seg, waypoint.lane)
	end
	add_waypoint!(seg.lanes[waypoint.lane], waypoint.pt, lla, override=override)
	return seg
end
function add_waypoint!(rndf::RNDF, waypoint::WaypointID, lla::LatLonAlt; override::Bool=false)
	if !haskey(rndf.segments, waypoint.segment)
		add_segment!(rndf, waypoint.segment)
	end
	add_waypoint!(rndf.segments[waypoint.segment], waypoint, lla, override=override)
	return rndf
end

function has_waypoint(rndf::RNDF, w::WaypointID)
	if !haskey(rndf.segments, w.segment)
		return false
	end
	seg = rndf.segments[w.segment]
	if !haskey(seg.lanes, w.lane)
	   return false
	end
	lane = seg.lanes[w.lane]
	return haskey(lane.waypoints, w.pt)
end
function has_lane(rndf::RNDF, id::LaneID)
	if !haskey(rndf.segments, id.segment)
		return false
	end
	seg = rndf.segments[id.segment]
	return haskey(seg.lanes, id.lane)
end
function has_lane(rndf::RNDF, id::WaypointID)
	if !haskey(rndf.segments, id.segment)
		return false
	end
	seg = rndf.segments[id.segment]
	return haskey(seg.lanes, id.lane)
end
has_segment(rndf::RNDF, id::Int) = return haskey(rndf.segments, id)

function load_rndf( filepath::AbstractString )
	# Loads a Road Network file
	# input:
	#    - filepath: the rndf text file to load
	@assert(isfile(filepath))

	fin = open(filepath, "r")
	lines = readlines(fin)
	close(fin)

	readparam(str::ASCIIString) = str[searchindex(str, " ")+1:end-1]
	function readparam(str::ASCIIString, param_name::AbstractString, ::Type{Int})
		@assert(startswith(str, param_name))
		return parse(Int, str[searchindex(str, " ")+1:end-1])
	end
	function readparam(str::ASCIIString, param_name::AbstractString, ::Type{Float64})
		@assert(startswith(str, param_name))
		return parse(Float64, str[searchindex(str, " ")+1:end-1])
	end
	function readparam{S<:AbstractString}(str::ASCIIString, param_name::AbstractString, ::Type{S})
		@assert(startswith(str, param_name))
		return str[searchindex(str, " ")+1:end-1]
	end
	function readparam(str::ASCIIString, param_name::AbstractString, ::Type{LaneID})
		@assert(startswith(str, param_name))
		str = str[searchindex(str, " ")+1:end-1]
		@assert(ismatch(REGEX_LANEID, str))
		matches = matchall(r"\d+", str)
		@assert(length(matches) == 2)
		LaneID(parse(UInt, matches[1]), parse(UInt, matches[2]))
	end
	function readparam(str::ASCIIString, param_name::AbstractString, ::Type{WaypointID})
		@assert(startswith(str, param_name))
		str = str[searchindex(str, " ")+1:end-1]
		@assert(ismatch(REGEX_WAYPOINT, str))
		matches = matchall(r"\d+", str)
		@assert(length(matches) == 3)
		WaypointID(parse(UInt, matches[1]), parse(UInt, matches[2]), parse(UInt, matches[3]))
	end

	rndf = RNDF()

	num_segments = -1
	num_zones = -1
	n_lines_skipped = 0

	current_segment = nothing
	current_lane = nothing

	predicted_num_lanes = Dict{Int,Int}() # seg_id -> n_lanes
	predicted_num_waypoints = Dict{LaneID,Int}() # seg_id -> n_lanes

	for line in lines
		if startswith(line, "RNDF_name")
			if rndf.name != "UNNAMED"
				print_with_color(:red, "overwriting RNDF name $(rndf.name)\n")
			end
			rndf.name = readparam(line)
			if VERBOSITY > 0
				println("RNDF name: ", rndf.name)
			end
		elseif startswith(line, "num_segments")
			if num_segments != -1
				print_with_color(:red, "overwriting num_segments $num_segments\n")
			end
			num_segments = readparam(line, "num_segments", Int)
			if VERBOSITY > 0
				println("num_segments: ", num_segments)
			end
		elseif startswith(line, "num_zones")
			if num_zones != -1
				print_with_color(:red, "overwriting num_zones $num_zones\n")
			end
			num_zones = readparam(line, "num_zones", Int)
			if VERBOSITY > 0
				println("num_zones: ", num_zones)
			end
		elseif startswith(line, "format_version")
			if rndf.format_version != ""
				print_with_color(:red, "overwriting format version $(rndf.format_version)\n")
			end
			rndf.format_version = readparam(line)
		elseif startswith(line, "creation_date")
			if rndf.creation_date != ""
				print_with_color(:red, "overwriting creation date $(rndf.creation_date)\n")
			end
			rndf.creation_date = readparam(line)
		elseif startswith(line, "segment ")
			seg_index = readparam(line, "segment", Int)
			if !haskey(rndf.segments, seg_index)
				add_segment!(rndf, seg_index)
			end
			current_segment = rndf.segments[seg_index]
			current_lane = nothing
			if VERBOSITY > 0
				println("\tsegment: ", seg_index)
			end
		elseif startswith(line, "segment_name")
			name = readparam(line)
			if current_segment == nothing
				print_with_color(:red, "no current segment for segment name $name")
			else
				current_segment.name = name
			end
		elseif startswith(line, "num_lanes")
			if current_segment == nothing
				print_with_color(:red, "no current segment for segment name $name")
			else
				if haskey(predicted_num_lanes, current_segment.id)
					print_with_color(:red, "ovewriting predicted num lanes for segment $(current_segment.id)")
				end
				predicted_num_lanes[current_segment.id] = readparam(line, "num_lanes", Int)
			end
		elseif startswith(line, "lane ")
			lane_id = readparam(line, "lane", LaneID)
			print(line)
			if !has_lane(rndf, lane_id)
				add_lane!(rndf, lane_id)
			end
			current_lane = get_lane(rndf, lane_id)
			if VERBOSITY > 0
				println("\t\tlane ", lane_id)
			end
		elseif startswith(line, "num_waypoints")
			if current_lane == nothing
				print_with_color(:red, "no current lane for num_waypoints")
			else
				if haskey(predicted_num_waypoints, current_lane.id)
					print_with_color(:red, "ovewriting predicted num waypoints for lane $(current_lane.id)")
				end
				predicted_num_waypoints[current_lane.id] = readparam(line, "num_waypoints", Int)
			end
		elseif startswith(line, "lane_width")
			if current_lane == nothing
				print_with_color(:red, "no current lane to assign lane width to\n")
			else
				if !isnan(current_lane.width)
					print_with_color(:red, "overwriting current lane's lane width of $(current_lane.width)\n")
				end
				current_lane.width = readparam(line, "lane_width", Float64)
			end
		elseif startswith(line, "speed_limit")
			if current_lane == nothing
				# TODO - allow this to affect segments
				print_with_color(:red, "no current lane to assign speed limit to\n")
			else
				if !isnan(current_lane.speed_limit[1])
					print_with_color(:red, "overwriting current lane's speed limit of $(current_lane.speed_limit)\n")
				end
				matches = matchall(r"\d+", line)
				current_lane.speed_limit = (float(matches[1]), float(matches[1]))
			end
		elseif startswith(line, "exit ")
			if current_lane == nothing
				print_with_color(:red, "no current lane to assign exit to")
			else
				matches = matchall(REGEX_WAYPOINT, line)
				exit_waypoint = parse_waypoint(matches[1])
				entry_waypoint = parse_waypoint(matches[2])
				push!(current_lane.exits, (exit_waypoint.pt, entry_waypoint))
				if VERBOSITY > 1
					println("\t\t\texit: ", exit_waypoint, " →  ", entry_waypoint)
				end
			end
		elseif ismatch(REGEX_WAYPOINT_LINE, line)
			pt = parse_waypoint(match(REGEX_WAYPOINT, line).match)
			if VERBOSITY > 2
				println(pt)
			end
			if has_waypoint(rndf, pt)
				print_with_color(:red, "overwriting waypoint $(pt)")
			end
			matches = matchall(REGEX_UTM_FLOAT, line)
			lla = LatLonAlt(parse(Float64, matches[1]), parse(Float64, matches[2]), parse(Float64, matches[3]))
			if VERBOSITY > 2
				println(lla)
			end
			add_waypoint!(rndf, pt, lla, override=true)
		elseif ismatch(REGEX_WAYPOINT_LINE_NOALT, line)
			pt = parse_waypoint(match(REGEX_WAYPOINT, line).match)
			if VERBOSITY > 2
				println(pt)
			end
			if has_waypoint(rndf, pt)
				print_with_color(:red, "overwriting waypoint $(pt)")
			end
			matches = matchall(REGEX_UTM_FLOAT, line)
			lla = LatLonAlt(parse(Float64, matches[1]), parse(Float64, matches[2]))
			add_waypoint!(rndf, pt, lla, override=true)
		elseif startswith(line, "end_file")
			if line != lines[end]
				print_with_color(:red, "end_file does not coincide with end of file\n")
			end
		elseif startswith(line, "merge_link")
			matches = matchall(REGEX_WAYPOINT, line)
			w_start = parse_waypoint(matches[1])
			w_end = parse_waypoint(matches[2])
			if !has_lane(rndf, w_start)
				add_lane!(rndf, w_start)
			end
			lane = get_lane(rndf, w_start)
			if lane.merge_link[1] != 0
				print_with_color(:red, "overwriting lane $(lane.id) merge link $(lane.merge_link)")
			end
			lane.merge_link = (w_start.pt, w_end)
			if VERBOSITY > 1
				println("\t\t\tmerge_link: ", w_start, " →  ", w_end)
			end
		elseif startswith(line, "merge_end_link")
			matches = matchall(REGEX_WAYPOINT, line)
			w_start = parse_waypoint(matches[1])
			w_end = parse_waypoint(matches[2])
			if !has_lane(rndf, w_start)
				add_lane!(rndf, w_start)
			end
			lane = get_lane(rndf, w_start)
			if lane.merge_end_link[1] != 0
				print_with_color(:red, "overwriting lane $(lane.id) merge end link $(lane.merge_end_link)")
			end
			lane.merge_end_link = (w_start.pt, w_end)
			if VERBOSITY > 1
				println("\t\t\tmerge_end_link: ", w_start, " →  ", w_end)
			end
		elseif !startswith(line, "end_segment") && !startswith(line, "end_lane") && !startswith(line, "checkpoint")
			n_lines_skipped += 1
			print_with_color(:red, "UNKNOWN LIME FORMAT: ", line)
		end
	end

	if VERBOSITY > 0
		println("DONE PARSING")
	end
	if n_lines_skipped > 0
		print_with_color(:red, "SKIPPED ", n_lines_skipped, "lines\n")
	end
	if num_segments != length(rndf.segments)
		print_with_color(:red, "PREDICTED $num_segments but got $(length(rndf.segments)) segments\n")
	end
	for seg in values(rndf.segments)
		if haskey(predicted_num_lanes, seg.id)
			pred = predicted_num_lanes[seg.id]
			if pred != length(seg.lanes)
				print_with_color(:red, "PREDICTED $pred but got $(length(seg.lanes)) lanes in segment $(seg.id)\n")
			end
		end
		if isempty(seg.lanes)
			print_with_color(:red, "segment $(seg.id) has no lanes")
		end
		for lane in values(seg.lanes)
			pred = predicted_num_waypoints[lane.id]

			if pred != length(lane.waypoints)
				print_with_color(:red, "PREDICTED $pred but got $(length(lane.waypoints)) waypoints\n")
			end
			if isempty(lane.waypoints)
				print_with_color(:red, "lane $(lane.id) has no waypoints")
			end
		end
	end

	return rndf
end

function parse_waypoint(str::AbstractString)
	# parses x.y.z, x,y,z ∈ integer > 0
	@assert ismatch(REGEX_WAYPOINT, str)
	ints = matchall(r"\d+", str)
	WaypointID(parse(UInt, ints[1]), parse(UInt, ints[2]), parse(UInt, ints[3]))
end

end # end module
