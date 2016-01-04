module StreetNetworks

using LightGraphs
using HDF5

using AutomotiveDrivingModels.Vec
using AutomotiveDrivingModels.Curves

include("RoadNetwork.jl")
using .RoadNetwork

import .RoadNetwork: LaneID, WaypointID

# ===============================================================================

export TILE_WIDTH, TILE_HEIGHT, EASTING_BASE, NORTHING_BASE
export DEFAULT_LANE_WIDTH, STREETMAP_BASE
export StreetNode, StreetLane, StreetSegment, NetworkTile, StreetNetwork
export utm2tileindex_east, utm2tileindex_north, utm2tileindex
export tile_center_north, tile_center_east, tile_lowerbound_north, tile_lowerbound_east
export tile_upperbound_north, tile_upperbound_east, tile_bounds, tile_center
export get_tile, get_tile!, add_tile!, has_tile, same_tile
export has_segment, get_segment, get_segment!
export get_lane, has_lane, add_lane!, has_next_lane, next_lane, has_prev_lane, prev_lane
export next_lanetag, prev_lanetag
export next_node_index, prev_node_index
export same_lane, same_lane_and_tile
export closest_node_to_extind, closest_node_above_extind
export neighbor_north, neighbor_east, neighbor_south, neighbor_west
export has_neighbor_east, has_neighbor_south, has_neighbor_west, extisting_neighbor_tiles, extisting_neighbor_tiles_inclusive
export TilePoint2DProjectionResult
export project_point_to_tile, project_point_to_streetmap, project_point_to_lanetag
export distance_to_lane_split, distance_to_lane_merge, distance_to_lane_end, distance_to_node
export frenet_distance_between_points
export num_lanes_on_sides, marker_distances, get_neighbor_lanetag_left, get_neighbor_lanetag_right
export rndf2streetnetwork, curves2streetnetwork, curves2rndf
export generate_straight_nlane_streetmap

export LaneID, WaypointID, LaneTag

# ===============================================================================

const TILE_WIDTH    = 250 # [m]
const TILE_HEIGHT   = 250 # [m]
const EASTING_BASE  = 0.0 # [m], the easting of the center of tile (0,0)
const NORTHING_BASE = 0.0 # [m], the northing of the center of tile (0,0)

const METERS_PER_FOOT = 0.3048 # [m/ft]

const DEFAULT_LANE_WIDTH = 3.7 # [m]
const DESIRED_DISTANCE_BETWEEN_CURVE_SAMPLES = 0.5 # [m]

const SATURATION_DISTANCE_TO_LANE_END   = 100.0 # [m]
const SATURATION_DISTANCE_TO_LANE_SPLIT = 100.0 # [m]
const SATURATION_DISTANCE_TO_LANE_MERGE = 100.0 # [m]
const SATURATION_DISTANCE_TO_NODE       = 100.0 # [m]
const SATURATION_DISTANCE_BETWEEN       = 500.0 # [m]
const THRESHOLD_DISTANCE_TO_NODE        = 500.0 # [m]

const STREETMAP_BASE = "/media/tim/DATAPART1/Data/Bosch/processed/streetmaps/"

# ===============================================================================

immutable LaneTag
	index_e :: Int
	index_n :: Int
	segment :: Int
	lane    :: Int
end
immutable StreetNode
	id :: WaypointID
	pos :: VecE3 # (UTM-e, UTM-n, alt [m])

	extind  :: Float64 # the extind of the node in its lane curve
	d_along :: Float64 # distance along the lane, 0.0 at source node [m]

	d_merge :: Float64 # distance to the next merge [m]
	d_split :: Float64 # distance to the next split [m]
	d_end   :: Float64 # distance to the lane end [m]

	n_lanes_left :: Int # number of lanes to the left (>= 0)
	n_lanes_right :: Int # number of lanes to the right (>= 0)

	marker_dist_left :: Float64 # estimated distance to left lane marker [m]
	marker_dist_right :: Float64 # estimated distance to right lane marker [m]
end
type StreetLane
	# an ordered list of centerline nodes comprising a lane

	id             :: LaneTag
	width          :: Float64 # [m]
	boundary_left  :: Symbol  # TODO(tim): make this a LaneBoundary type?
	boundary_right :: Symbol  # ∈ {:double_yellow, :solid_yellow, :solid_white, :broken_white, :unknown}
	node_indeces   :: Vector{Int} # indeces within the streenet nodes array
	                              # only includes nodes in this lane, not leading or trailing nodes
	curve          :: Curve # this curve stretches from 1 node before source to 1 node after end, if applicable
	                        # x = 0.0 occurs at source node and is measured positive along the centerline

	has_leading_node :: Bool # true iff curve includes a source node leading into this lane
	has_trailing_node :: Bool # true iff curve includes a destination node leaving this lane
end
type StreetSegment
	# a list of lanes forming a single road with a common direction
	id :: Int  # > 0
	lanes :: Dict{Int, StreetLane}

	StreetSegment(id::Int, lanes::Dict{Int, StreetLane} = Dict{Int,StreetLane}()) = new(id, lanes)
end
type NetworkTile
	index_e :: Int
	index_n :: Int
	segments :: Dict{Int, StreetSegment} # maps segment id to segment

	function NetworkTile(
		index_e::Int,
		index_n::Int,
		segments::Dict{Int,StreetSegment} = Dict{Int, StreetSegment}()
		)

		new(index_e, index_n, segments)
	end
end
type StreetNetwork
	nodes::Vector{StreetNode}
	tile_dict::Dict{Tuple{Int,Int},NetworkTile} # (index_n, index_e)
	graph::DiGraph # vertices in the graph are indeces in the nodes vector

	function StreetNetwork()

		nodes = StreetNode[]
		tile_dict = Dict{Tuple{Int,Int},NetworkTile}()
		graph = DiGraph()

		new(nodes, tile_dict, graph)
	end
	function StreetNetwork(
		nodes::Vector{StreetNode},
		tile_dict::Dict{Tuple{Int,Int},NetworkTile},
		graph::DiGraph
		)

		new(nodes, tile_dict, graph)
	end
end

function Base.deepcopy(lane::StreetLane)
	StreetLane(
		lane.id,
		lane.width,
		lane.boundary_left,
		lane.boundary_right,
		deepcopy(lane.node_indeces),
		deepcopy(lane.curve),
		lane.has_leading_node,
		lane.has_trailing_node
		)
end
Base.deepcopy(seg::StreetSegment) = StreetSegment(seg.id, deepcopy(seg.lanes))
Base.deepcopy(tile::NetworkTile) = NetworkTile(tile.index_e, tile.index_n, deepcopy(tile.segments))
Base.deepcopy(sn::StreetNetwork) = StreetNetwork(deepcopy(sn.nodes, sn.tile_dict, sn.graph))

Base.convert(::Type{LaneID}, tag::LaneTag) = LaneID(tag.segment, tag.lane)

immutable TilePoint2DProjectionResult
	successful :: Bool       # whether the projection was successful
	footpoint  :: CurvePt    # the closest point on the curve
	extind     :: Float64    # the extind on the curve
	lane       :: StreetLane # the corresponding lane
	sqdist     :: Float64    # square distance to the curve

	TilePoint2DProjectionResult() = new(false)
	function TilePoint2DProjectionResult(
		footpoint  :: CurvePt,
		extind     :: Float64,
		lane       :: StreetLane,
		sqdist     :: Float64,
		)

		new(true, footpoint, extind, lane, sqdist)
	end
end

function Base.show(io::IO, proj::TilePoint2DProjectionResult)
	println(io, "TilePoint2DProjectionResult:")
	println(io, "\tcurvept:    ", proj.footpoint)
	println(io, "\textind:     ", proj.extind)
	println(io, "\tlanetag:    ", proj.lane.id)
	println(io, "\tsqdist:     ", proj.sqdist)
	println(io, "\tsuccessful: ", proj.successful)
end

Base.(:(==))(A::LaneTag, B::LaneTag) = A.index_e == B.index_e &&
							 A.index_n == B.index_n &&
							 A.segment == B.segment &&
							 A.lane    == B.lane
Base.isequal(A::LaneTag, B::LaneTag) = A.index_e == B.index_e &&
							      A.index_n == B.index_n &&
							      A.segment == B.segment &&
							      A.lane    == B.lane
same_tile(A::LaneTag, B::LaneTag) = A.index_e == B.index_e && A.index_n == B.index_n

# ===============================================================================

# NOTE(tim): returns the smallest in magnitude signed angle a must be rotated by to get to b
# TODO(tim): move to a utility file
_signed_dist_btw_angles(a::Real, b::Real) = atan2(sin(b-a), cos(b-a))

utm2tileindex_east(utm_easting::Real) = round(Int, (utm_easting  - EASTING_BASE)  / TILE_WIDTH, RoundNearestTiesUp)
utm2tileindex_north(utm_northing::Real) = round(Int, (utm_northing - NORTHING_BASE) / TILE_HEIGHT, RoundNearestTiesUp)
utm2tileindex(utm_easting::Real, utm_northing::Real) = (utm2tileindex_east(utm_easting), utm2tileindex_north(utm_northing))

tile_center_north(index_n::Integer)     =  index_n      * TILE_HEIGHT + NORTHING_BASE
tile_lowerbound_north(index_n::Integer) = (index_n-0.5) * TILE_HEIGHT + NORTHING_BASE
tile_upperbound_north(index_n::Integer) = (index_n+0.5) * TILE_HEIGHT + NORTHING_BASE
tile_center_east(index_e::Integer)      =  index_e      * TILE_WIDTH  + EASTING_BASE
tile_lowerbound_east(index_e::Integer)  = (index_e-0.5) * TILE_WIDTH  + EASTING_BASE
tile_upperbound_east(index_e::Integer)  = (index_e+0.5) * TILE_WIDTH  + EASTING_BASE
function tile_bounds(tile::NetworkTile)
	index_e, index_n = tile.index_e, tile.index_n
	min_e = tile_lowerbound_east(index_e)
    max_e = tile_upperbound_east(index_e)
    min_n = tile_lowerbound_north(index_n)
    max_n = tile_upperbound_north(index_n)
    (min_e, max_e, min_n, max_n)
end
tile_center(tile::NetworkTile) = (tile_center_east(tile.index_e), tile_center_north(tile.index_n))

function get_tile(sn::StreetNetwork, index_e::Int, index_n::Int)
	retval = sn.tile_dict[(index_e, index_n)]
	@assert(retval.index_e == index_e && retval.index_n == index_n)
	retval
end
function get_tile(sn::StreetNetwork, easting::Float64, northing::Float64)
	index_e, index_n = utm2tileindex(easting, northing)
	get_tile(sn, index_e, index_n)
end
get_tile(sn::StreetNetwork, node::StreetNode) = get_tile(sn, node.pos.x, node.pos.y)
get_tile(sn::StreetNetwork, tag::LaneTag) = get_tile(sn, tag.index_e, tag.index_n)

function get_tile!(sn::StreetNetwork, index_e::Int, index_n::Int)
	key = (index_e, index_n)
	if !haskey(sn.tile_dict, key)
		sn.tile_dict[key] = NetworkTile(index_e, index_n, Dict{Int, StreetSegment}())
	end
	retval = sn.tile_dict[key]
	@assert(retval.index_e == index_e && retval.index_n == index_n)
	retval
end
function get_tile!(sn::StreetNetwork, easting::Float64, northing::Float64)
	index_e, index_n = utm2tile(easting, northing)
	get_tile!(sn, index_e, index_n)
end
function add_tile!(sn::StreetNetwork, tile::NetworkTile; override=false)
	key = (tile.index_e, tile.index_n)
	if haskey(sn.tile_dict, key) && !override
		error("tile already exists: ", key)
	end
	sn.tile_dict[key] = tile
	nothing
end

has_tile(sn::StreetNetwork, index_e::Int, index_n::Int) = haskey(sn.tile_dict, (index_e, index_n))

has_segment(tile::NetworkTile, segment_id::Integer) = haskey(tile.segments, segment_id)
function get_segment(tile::NetworkTile, segment_id::Int)
	if !haskey(tile.segments, segment_id)
		error("segment not found in tile")
	end
	retval = tile.segments[segment_id]
	@assert(retval.id == segment_id)
	retval
end
get_segment(sn::StreetNetwork, tag::LaneTag) = get_segment(get_tile(sn, tag), tag.segment)
function get_segment!(tile::NetworkTile, segment_id::Int)
	if !haskey(tile.segments, segment_id)
		segment = StreetSegment(segment_id, Dict{Int, StreetLane}())
		tile.segments[segment_id] = segment
		return segment
	end
	retval = tile.segments[segment_id]
	@assert(retval.id == segment_id)
	retval
end

has_lane(segment::StreetSegment, id::Integer) = haskey(segment.lanes, id)
has_lane(segment::StreetSegment, lane::StreetLane) = haskey(segment.lanes, lane.id.lane)
has_lane(tile::NetworkTile, id::LaneID) = haskey(tile.segments, id.segment) && has_lane(tile.segments[id.segment], id.lane)
has_lane(tile::NetworkTile, id::WaypointID) = haskey(tile.segments, id.segment) && has_lane(tile.segments[id.segment], id.lane)
function has_lane(sn::StreetNetwork, tag::LaneTag)
	if has_tile(sn, tag.index_e, tag.index_n)
		tile = sn.tile_dict[(tag.index_e, tag.index_n)]
		return haskey(tile.segments, tag.segment) && has_lane(tile.segments[tag.segment], tag.lane)
	end
	return false
end
get_lane(tile::NetworkTile, id::LaneID) = tile.segments[id.segment].lanes[id.lane]
get_lane(tile::NetworkTile, id::WaypointID) = tile.segments[id.segment].lanes[id.lane]
get_lane(sn::StreetNetwork, tag::LaneTag) = sn.tile_dict[(tag.index_e, tag.index_n)].segments[tag.segment].lanes[tag.lane]
get_lane(sn::StreetNetwork, node::StreetNode) = get_lane(get_tile(sn, node.pos.x, node.pos.y), node.id)
get_lane(proj::TilePoint2DProjectionResult) = proj.lane

function add_lane!(segment::StreetSegment, lane::StreetLane; override=false)
	if !override && haskey(segment.lanes, lane.id.lane)
		error("segment already has lane: ", lane)
	end
	segment.lanes[lane.id.lane] = lane
	nothing
end
function has_next_lane(sn::StreetNetwork, lane::StreetLane)
	if isempty(lane.node_indeces)
		return false
	end
	outdegree(sn.graph, lane.node_indeces[end]) > 0
end
function next_lane(sn::StreetNetwork, lane::StreetLane)
	lastnode_index = lane.node_indeces[end]
	nextnode_index = next_node_index(sn, lastnode_index)
	if nextnode_index == lastnode_index
		error("no next lane!")
	end
	get_lane(sn, sn.nodes[nextnode_index])
end
function next_lanetag(sn::StreetNetwork, lanetag::LaneTag)
	# retuns the current one if the next does not exist
	lane = get_lane(sn, lanetag)
	if has_next_lane(sn, lane)
		return next_lane(sn, lane).id
	end
	return lanetag
end

has_prev_lane(sn::StreetNetwork, lane::StreetLane) = !isempty(lane.node_indeces) && indegree(sn.graph, lane.node_indeces[1]) > 0
function prev_lane(sn::StreetNetwork, lane::StreetLane)
	firstnode_index = lane.node_indeces[1]
	prevnode_index = prev_node_index(sn, firstnode_index)
	if prevnode_index == firstnode_index
		error("no previous lane!")
	end
	get_lane(sn, sn.nodes[prevnode_index])
end
function prev_lanetag(sn::StreetNetwork, lanetag::LaneTag)
	# retuns the current one if the prev does not exist
	lane = get_lane(sn, lanetag)
	if has_prev_lane(sn, lane)
		return prev_lane(sn, lane).id
	end
	return lanetag
end

function next_node_index(sn::StreetNetwork, node_index::Int)

	#=
	Given the node_index, find the next one if there is one
		if there are none, return node_index
		if there are multiple, pick the one in the same lane
		if there are multiple by all in different lanes, pick the first one given by out_neighbors
	=#

	G = sn.graph

	if outdegree(G, node_index) == 0
		return node_index
	end

	node = sn.nodes[node_index]
	laneid = node.id.lane
	segid  = node.id.segment

	best_node_index = node_index
	found_new = false
	for new_node_index in out_neighbors(G, node_index)
		new_node = sn.nodes[node_index]
		if new_node.id.lane == laneid && new_node.id.segment == segid
			return new_node_index
		elseif !found_new
			best_node_index = new_node_index
			found_new = true
		end
	end

	best_node_index
end
function prev_node_index(sn::StreetNetwork, node_index::Int)

	#=
	given the node, find the previous one if there is one
		if there are none, return node
		if there are multiple, pick the one in the same lane
		if there are multiple by all in different lanes, pick the first one given by out_neighbors
	=#

	G = sn.graph

	if indegree(G, node_index) == 0
		return node_index
	end

	node = sn.nodes[node_index]
	laneid = node.id.lane
	segid  = node.id.segment

	best_node_index = node_index
	found_new = false
	for new_node_index in in_neighbors(G, node_index)
		new_node = sn.nodes[node_index]
		if !found_new || (new_node.id.lane == laneid && new_node.id.segment == segid)
			best_node_index = new_node_index
			found_new = true
		end
	end

	best_node_index
end

same_lane(a::StreetNode, b::StreetNode) = a.id.lane == b.id.lane && a.id.segment == b.id.segment
same_tile(a::StreetNode, b::StreetNode) = utm2tileindex_east(a.pos.x)  == utm2tileindex_east(b.pos.x) &&
										  utm2tileindex_north(a.pos.y) == utm2tileindex_north(b.pos.y)
same_lane_and_tile(a::StreetNode, b::StreetNode) = same_lane(a,b) && same_tile(a,b)
same_lane_and_tile(sn::StreetNetwork, a::Integer, b::Integer) = same_lane_and_tile(sn.nodes[a], sn.nodes[b])

function closest_node_to_extind(sn::StreetNetwork, lane::StreetLane, extind::Float64; sq_dist_threshold = 0.1)

	#=
	Do a bisection search to get the closest node to the given extind
	returns a node index
	=#

	a = 1
	b = length(lane.node_indeces)

	δ = sn.nodes[lane.node_indeces[a]].extind - extind
	sqdist_a = δ*δ

	δ = sn.nodes[lane.node_indeces[b]].extind - extind
	sqdist_b = δ*δ

	if b == a
		return lane.node_indeces[a]
	elseif b == a + 1
		return sqdist_b < sqdist_a ? lane.node_indeces[b] : lane.node_indeces[a]
	end

	c = div(a+b, 2)
	δ = sn.nodes[lane.node_indeces[c]].extind - extind
	sqdist_c = δ*δ

	n = 1
	while true
		if b == a
			return lane.node_indeces[a]
		elseif b == a + 1
			return sqdist_b < sqdist_a ? lane.node_indeces[b]: lane.node_indeces[a]
		elseif c == a + 1 && c == b - 1
			if sqdist_a < sqdist_b && sqdist_a < sqdist_c
				return lane.node_indeces[a]
			elseif sqdist_b < sqdist_a && sqdist_b < sqdist_c
				return lane.node_indeces[b]
			else
				return lane.node_indeces[c]
			end
		end

		left = div(a+c, 2)

		δ = sn.nodes[lane.node_indeces[left]].extind - extind
		sqdist_l = δ*δ

		if sqdist_l < sq_dist_threshold
			return lane.node_indeces[left]
		end

		right = div(c+b, 2)

		δ = sn.nodes[lane.node_indeces[right]].extind - extind
		sqdist_r = δ*δ

		if sqdist_r < sq_dist_threshold
			return lane.node_indeces[right]
		elseif sqdist_l < sqdist_r
			b = c
			c = left
		else
			a = c
			c = right
		end
	end

	error("invalid codepath")
end
function closest_node_above_extind(sn::StreetNetwork, lane::StreetLane, extind::Float64; isapprox_threshold::Float64=0.1)
	# do a linear search
	# returns a node index
	# NOTE(tim): returns last node if it is a dead-end

	# TODO(tim): implement bisection search

	if extind > sn.nodes[lane.node_indeces[end]].extind
		return next_node_index(sn, lane.node_indeces[end])
	end

	a = 1
	b = length(lane.node_indeces)

	n = 1
	while true
		if b == a || b == a+1
			return lane.node_indeces[b]
		end

		c = div( a+b, 2 )
		fc = sn.nodes[lane.node_indeces[c]].extind - extind

		if 0 ≤ fc < isapprox_threshold
			return lane.node_indeces[c]
		elseif fc < 0.0
			a = c
		else
			b = c
		end
	end

	error("invalid codepath")
end

neighbor_north(sn::StreetNetwork, tile::NetworkTile) = get_tile(sn, tile.index_e,   tile.index_n+1)
neighbor_east(sn::StreetNetwork,  tile::NetworkTile) = get_tile(sn, tile.index_e+1, tile.index_n  )
neighbor_south(sn::StreetNetwork, tile::NetworkTile) = get_tile(sn, tile.index_e,   tile.index_n-1)
neighbor_west(sn::StreetNetwork,  tile::NetworkTile) = get_tile(sn, tile.index_e-1, tile.index_n  )
has_neighbor_north(sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e, tile.index_n+1)
has_neighbor_east( sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e+1, tile.index_n)
has_neighbor_south(sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e, tile.index_n-1)
has_neighbor_west( sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e-1, tile.index_n)
function extisting_neighbor_tiles(sn::StreetNetwork, tile::NetworkTile)
	retval = Array(NetworkTile, 8)
	total = 0

	del_x = [-1, 0, 1,-1, 1,-1, 0, 1]
	del_y = [-1,-1,-1, 0, 0, 1, 1, 1]

	for i = 1 : 8
		dx, dy = del_x[i], del_y[i]
		if has_tile(sn, tile.index_e + dx, tile.index_n + dy)
			retval[total+=1] = get_tile(sn, tile.index_e + dx, tile.index_n + dy)
		end
	end

	return retval[1:total]
end
function extisting_neighbor_tiles_inclusive(sn::StreetNetwork, tile::NetworkTile)
	retval = Array(NetworkTile, 9)
	retval[1] = tile
	total = 1

	del_x = [-1, 0, 1,-1, 1,-1, 0, 1]
	del_y = [-1,-1,-1, 0, 0, 1, 1, 1]

	for i = 1 : 8
		dx, dy = del_x[i], del_y[i]
		if has_tile(sn, tile.index_e + dx, tile.index_n + dy)
			retval[total+=1] = get_tile(sn, tile.index_e + dx, tile.index_n + dy)
		end
	end

	return retval[1:total]
end

function project_point_to_tile(easting::Float64, northing::Float64, tile::NetworkTile )

	retval = TilePoint2DProjectionResult()

	for (segment_id, segment) in tile.segments
		for (lane_id, lane) in segment.lanes
			extind = closest_point_extind_to_curve(lane.curve, easting, northing)
			pt = curve_at(lane.curve, extind)
			de = pt.x - easting
			dn = pt.y - northing
			square_dist = de*de + dn*dn
			if !retval.successful || square_dist < retval.sqdist
				retval = TilePoint2DProjectionResult(pt, extind, lane, square_dist)
			end
		end
	end

	retval
end
function project_point_to_tile(easting::Float64, northing::Float64, tile::NetworkTile, f_filter::Function)

	retval = TilePoint2DProjectionResult()

	for (segment_id, segment) in tile.segments
		for (lane_id, lane) in segment.lanes
			extind = closest_point_extind_to_curve(lane.curve, easting, northing)
			pt = curve_at(lane.curve, extind)
			if f_filter(pt, lane)
				de = pt.x - easting
				dn = pt.y - northing
				square_dist = de*de + dn*dn
				if !retval.successful || square_dist < retval.sqdist
					retval = TilePoint2DProjectionResult(pt, extind, lane, square_dist)
				end
			end
		end
	end

	retval
end
function project_point_to_streetmap(easting::Float64, northing::Float64, sn::StreetNetwork)

	index_e, index_n = utm2tileindex(easting, northing)

	tile = first(sn.tile_dict)[2]
	proj = TilePoint2DProjectionResult()

	if has_tile(sn, index_e, index_n)
		tile = get_tile(sn, index_e, index_n)
		proj = project_point_to_tile(easting, northing, tile)
	end

	if !proj.successful || proj.sqdist > 1.5
		# NOTE(tim): if were are already close to a centerline on this tile then
		# no need to do extra work to check neighboring tiles

		min_e, max_e, min_n, max_n = tile_bounds(tile)

		d_e_lo = easting - min_e
		d_e_hi = max_e - easting
		d_n_lo = northing - min_n
		d_n_hi = max_n - northing

		d_e_lo_sq = d_e_lo * d_e_lo
		d_e_hi_sq = d_e_hi * d_e_hi
		d_n_lo_sq = d_n_lo * d_n_lo
		d_n_hi_sq = d_n_hi * d_n_hi

		border_sq_distances = [d_e_lo_sq, d_e_hi_sq, d_n_lo_sq, d_n_hi_sq,
		                       d_e_lo_sq + d_n_lo_sq, d_e_lo_sq + d_n_hi_sq,
		                       d_e_hi_sq + d_n_lo_sq, d_e_hi_sq + d_n_hi_sq]
		borders          = [(-1, 0), ( 1,0), (0,-1), (0,1),
		                    (-1,-1), (-1,1), (1,-1), (1,1)]
		p = sortperm(border_sq_distances)
		permute!(border_sq_distances, p)
		permute!(borders, p)

		best_sqdist = proj.successful ? proj.sqdist : Inf
		i = 1
		while i ≤ 8 && best_sqdist > border_sq_distances[i]
			b = borders[i]
			# println("checking $(b[1]), $(b[2])")
			if has_tile(sn, index_e+b[1], index_n+b[2])
				proj2 = project_point_to_tile(easting, northing, get_tile(sn, index_e+b[1], index_n+b[2]))
				if proj2.sqdist < best_sqdist
					best_sqdist, proj = proj2.sqdist, proj2
				end
			end
			i += 1
		end
	end

	return proj
end
function project_point_to_streetmap(easting::Float64, northing::Float64, sn::StreetNetwork, f_filter::Function)

	index_e, index_n = utm2tileindex(easting, northing)

	tile = first(sn.tile_dict)[2]
	proj = TilePoint2DProjectionResult()

	if has_tile(sn, index_e, index_n)
		tile = get_tile(sn, index_e, index_n)
		proj = project_point_to_tile(easting, northing, tile, f_filter)
	end

	if !proj.successful || proj.sqdist > 1.5
		# NOTE(tim): if were are already close to a centerline on this tile then
		# no need to do extra work to check neighboring tiles

		min_e, max_e, min_n, max_n = tile_bounds(tile)

		d_e_lo = easting - min_e
		d_e_hi = max_e - easting
		d_n_lo = northing - min_n
		d_n_hi = max_n - northing

		d_e_lo_sq = d_e_lo * d_e_lo
		d_e_hi_sq = d_e_hi * d_e_hi
		d_n_lo_sq = d_n_lo * d_n_lo
		d_n_hi_sq = d_n_hi * d_n_hi

		border_sq_distances = [d_e_lo_sq, d_e_hi_sq, d_n_lo_sq, d_n_hi_sq,
		                       d_e_lo_sq + d_n_lo_sq, d_e_lo_sq + d_n_hi_sq,
		                       d_e_hi_sq + d_n_lo_sq, d_e_hi_sq + d_n_hi_sq]
		borders          = [(-1, 0), ( 1,0), (0,-1), (0,1),
		                    (-1,-1), (-1,1), (1,-1), (1,1)]
		p = sortperm(border_sq_distances)
		permute!(border_sq_distances, p)
		permute!(borders, p)

		best_sqdist = proj.successful ? proj.sqdist : Inf
		i = 1
		while i ≤ 8 && best_sqdist > border_sq_distances[i]
			b = borders[i]
			# println("checking $(b[1]), $(b[2])")
			if has_tile(sn, index_e+b[1], index_n+b[2])
				proj2 = project_point_to_tile(easting, northing, get_tile(sn, index_e+b[1], index_n+b[2]), f_filter)
				if proj2.sqdist < best_sqdist
					best_sqdist, proj = proj2.sqdist, proj2
				end
			end
			i += 1
		end
	end

	return proj
end
function project_point_to_lanetag(
	sn::StreetNetwork,
	posGx::Float64,
	posGy::Float64,
	lanetag::LaneTag;
	max_n_jumps::Integer = 5 # maximum number of lane segments to go forward or back
	)

	lane = get_lane(sn, lanetag)
    curve = lane.curve

	n_lane_jumps = 0
    extind = closest_point_extind_to_curve(curve, posGx, posGy)

    if extind == 1.0
	    while extind == 1.0 && n_lane_jumps < 5
	        n_lane_jumps += 1
	        @assert(has_prev_lane(sn, lane))
	        lane = prev_lane(sn, lane)
	        curve = lane.curve
	        extind = closest_point_extind_to_curve(curve, posGx, posGy)
	    end
	else
	    while is_extind_at_curve_end(curve, extind) && n_lane_jumps < 5
	        n_lane_jumps += 1
	        if extind > 1.0
	            @assert(has_next_lane(sn, lane))
	            lane = next_lane(sn, lane)
	            curve = lane.curve
	            extind = closest_point_extind_to_curve(curve, posGx, posGy)
	        end
	    end
	end

	(extind, lane)
end

function _distance_to_lane_merge(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_MERGE # [m]
	)

	# NOTE(tim): follow the nodes until we get to one that has two incoming edges
	# follows next_node_index()
	# returns Inf if the lane ends

	lane = segment.lanes[lane_index]
	node_index = closest_node_above_extind(sn, lane, extind)
	node = sn.nodes[node_index]
	dist = node.d_along - curve_at(lane.curve, extind).s

	while indegree(sn.graph, node_index) < 2

	    if outdegree(sn.graph, node_index) == 0
	        return max_dist # no merge
	    end

	    nextnode_index = next_node_index(sn, node_index)
	    nextnode = sn.nodes[nextnode_index]

	    if same_lane_and_tile(sn, node_index, nextnode_index)
	        dist += nextnode.d_along - node.d_along
	    else
	    	# NOTE(tim): presumably the curve ends with the next node, so this is valid
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	if !(lane.curve.s[end] - node.d_along ≥ 0.0)
	    		println(lane.curve.s[end], "  ", node.d_along, "  ", node.id, "  ", nextnode.id)
	    	end
	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end
	    node_index, node = nextnode_index, nextnode
	    if dist > max_dist
	    	return max_dist
	    end
	end

	return min(dist, max_dist)
end
function _distance_to_lane_split(
    sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_SPLIT # [m]
	)

	# NOTE(tim): follow the nodes until we get to one that has two outgoing edges
	# follows next_node_index()
	# returns Inf if the lane ends

	lane = segment.lanes[lane_index]
	node_index = closest_node_above_extind(sn, lane, extind)
	node = sn.nodes[node_index]
	dist = node.d_along - curve_at(lane.curve, extind).s

	while outdegree(sn.graph, node_index) < 2

	    if outdegree(sn.graph, node_index) == 0
	        return max_dist
	    end

	    nextnode_index = next_node_index(sn, node_index)
	    nextnode = sn.nodes[nextnode_index]

	    if same_lane_and_tile(node, nextnode)
	        dist += nextnode.d_along - node.d_along
	    else
	    	# NOTE(tim): presumably the curve ends with the next node, so this is valid
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	if !(lane.curve.s[end] - node.d_along ≥ 0.0)
	    		println(lane.curve.s[end], "  ", node.d_along, "  ", node.id, "  ", nextnode.id)
	    	end
	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end
	    node_index, node = nextnode_index, nextnode
	    if dist > max_dist
	    	return max_dist
	    end
	end

	return min(dist, max_dist)
end
function _distance_to_lane_end(
    sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_END # [m]
	)

	# NOTE(tim): follow the nodes until we get to the end
	# follows next_node_index()
	# returns Inf if it gets back to a node is has previously seen (loop)

	lane = segment.lanes[lane_index]
	firstnode_index = node_index = closest_node_above_extind(sn, lane, extind)
	node = sn.nodes[node_index]
	dist = node.d_along - curve_at(lane.curve, extind).s

	while outdegree(sn.graph, node_index) != 0

	    nextnode_index = next_node_index(sn, node_index)
	    if nextnode_index === firstnode_index
	    	return max_dist
	    end

	    nextnode = sn.nodes[nextnode_index]

	    if same_lane_and_tile(node, nextnode)
	        dist += nextnode.d_along - node.d_along
	    else
	    	# NOTE(tim): presumably the curve ends with the next node, so this is valid
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	if !(lane.curve.s[end] - node.d_along ≥ 0.0)
	    		println(lane.curve.s[end], "  ", node.d_along, "  ", node.id, "  ", nextnode.id)
	    	end
	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end
	    node_index, node = nextnode_index, nextnode

	    if dist > max_dist
	    	return max_dist
	    end
	end

	min(dist, max_dist)
end

function distance_to_lane_merge(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_MERGE
	)

	lane = segment.lanes[lane_index]
	node_index = closest_node_to_extind(sn, lane, extind)
	node = sn.nodes[node_index]

	if node.extind > extind
        min(node.d_along - curve_at(lane.curve, extind).s + node.d_merge, max_dist)
    else
        min(curve_at(lane.curve, extind).s - node.d_along + node.d_merge, max_dist)
    end
end
function distance_to_lane_split(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
    max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_SPLIT
	)

	lane = segment.lanes[lane_index]
	node_index = closest_node_to_extind(sn, lane, extind)
	node = sn.nodes[node_index]

    if node.extind > extind
        min(node.d_along - curve_at(lane.curve, extind).s + node.d_split, max_dist)
    else
        min(curve_at(lane.curve, extind).s - node.d_along + node.d_split, max_dist)
    end
end
function distance_to_lane_end(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
    max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_END
	)

	lane = segment.lanes[lane_index]
	node_index = closest_node_to_extind(sn, lane, extind)
	node = sn.nodes[node_index]

    if node.extind > extind
        min(node.d_along - curve_at(lane.curve, extind).s + node.d_end, max_dist)
    else
        min(curve_at(lane.curve, extind).s - node.d_along + node.d_end, max_dist)
    end
end
function distance_to_node(
	sn           :: StreetNetwork,
	segment      :: StreetSegment,
	lane_index   :: Int,
	extind       :: Float64,
	target_index :: Int; # Node
	max_dist     :: Float64 = THRESHOLD_DISTANCE_TO_NODE # [m]
	)

	# TODO(tim): currently only follows primary search path
	#            how should we handle offramps?

	target = sn.nodes[target_index]
	lane = segment.lanes[lane_index]
	node_index = closest_node_to_extind(sn, lane, extind)
	node = sn.nodes[node_index]
	dist = node.d_along - curve_at(lane.curve, extind).s

	if hypot(node.pos.x - target.pos.x, node.pos.y - target.pos.y) > max_dist
		# NOTE(tim): if out of circular range cannot possibly be within max_dist
		return max_dist
	end

	if node_index == target_index
		return dist ≥ 0.0 ? dist : max_dist
	end

	while outdegree(sn.graph, node_index) > 0

	    nextnode_index = next_node_index(sn, node_index)
	    nextnode = sn.nodes[nextnode_index]

	    if same_lane_and_tile(node, nextnode)
	        dist += nextnode.d_along - node.d_along
	    else
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end

	    if nextnode_index == target_index
	    	return dist
	    end
	    if dist > max_dist
	    	return max_dist
	    end

		node, node_index = nextnode, nextnode_index
	end

	max_dist
end
function frenet_distance_between_points(
	sn         :: StreetNetwork,
	A_east     :: Float64,
	A_north    :: Float64,
	B_east     :: Float64,
	B_north    :: Float64,
	max_dist   :: Float64 = SATURATION_DISTANCE_BETWEEN # [m]
	)

	# NOTE(tim): only works for B downstream from A

	# find closest point to start
	# run along lane curve until we can project B to the same curve

	projA = project_point_to_streetmap(A_east, A_north, sn)
	if projA.successful

		A_s, A_d = Curves.pt_to_frenet_xy(projA.footpoint, A_east, A_north)

		# println("A: ", (A_s, A_d))

		lane = get_lane(projA)
		search_dist = 0.0
		finished = false

		# println(search_dist, "  ", lane.id, "  ", finished)

		while !finished

			extind = Curves.closest_point_extind_to_curve(lane.curve, B_east, B_north)
			# println("extind: ", extind)
			# println(lane.curve.x[1])
			# println(lane.curve.y[1])
			# println(lane.curve.x[end])
			# println(lane.curve.y[end])
			if !is_extind_at_curve_end(lane.curve, extind)

				# println("\t1")

				ptC = Curves.curve_at(lane.curve, extind)
				s, d = Curves.pt_to_frenet_xy(ptC, B_east, B_north)
				Δs = s - A_s + search_dist
				Δd = d - A_d
				return (Δs, Δd)
			elseif has_next_lane(sn, lane)
				search_dist += lane.curve.s[end]
				lane = next_lane(sn, lane)
				finished = search_dist > max_dist
				# println("\t2: ", search_dist, "  ", lane.id, "  ", finished)
			else
				# println("\t3")
				finished = true
			end
		end
	end

	(NaN, NaN)
end

function num_lanes_on_sides(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64
	)

	lane = segment.lanes[lane_index]
	node_index = closest_node_to_extind(sn, lane, extind)
	node = sn.nodes[node_index]

	@assert(node.n_lanes_left ≥ 0)
	@assert(node.n_lanes_right ≥ 0)

	(node.n_lanes_left, node.n_lanes_right)
end
function marker_distances(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64
	)

	lane = segment.lanes[lane_index]
	node_index = closest_node_to_extind(sn, lane, extind)
	node = sn.nodes[node_index]

	(node.marker_dist_left, node.marker_dist_right)
end

function get_neighbor_lanetag_left(sn::StreetNetwork, lane::StreetLane, closest_node::StreetNode)
    @assert(closest_node.n_lanes_left > 0)
    @assert(!isnan(closest_node.marker_dist_left))

    dcl = closest_node.marker_dist_left + 0.1
    footpoint = curve_at(lane.curve, closest_node.extind)
    footvec = VecSE2(footpoint.x, footpoint.y, footpoint.θ)
    inertial = footvec + Vec.polar(dcl, π/2 + footvec.θ, footvec.θ)

    proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
    @assert(proj.successful)
    proj.lane.id
end
function get_neighbor_lanetag_right(sn::StreetNetwork, lane::StreetLane, closest_node::StreetNode)
    @assert(closest_node.n_lanes_right > 0)
    @assert(!isnan(closest_node.marker_dist_right))

    dcl = closest_node.marker_dist_right + 0.1
    footpoint = curve_at(lane.curve, closest_node.extind)
    footvec = VecSE2(footpoint.x, footpoint.y, footpoint.θ)
    inertial = footvec + Vec.polar(dcl, -π/2 + footvec.θ, footvec.θ)

    proj = project_point_to_streetmap(inertial.x, inertial.y, sn)
    @assert(proj.successful)
    proj.lane.id
end

function rndf2streetnetwork(
	rndf::RNDF;
	verbosity::Int=0,
	convert_ll2utm::Bool=true,
	CYCLE_CONNECTION_THRESHOLD::Float64=6.25 # [m]
	)
	# convert the given rndf to a street network


	verbosity < 1 || println("RNDF 2 STREET NETWORK")
	starttime = time()

	sn = StreetNetwork()
	G = sn.graph

	const EDGE_DIST_THRESHOLD = 50.0

	node_map = Dict{WaypointID, Int}()
	lane_ids = Dict{LaneID, Vector{Int}}()

	# build the road network graph
	verbosity < 1 || @printf("BUILDING GRAPH [%.2f]\n", time()-starttime)
	for segment in values(rndf.segments)
		for lane in values(segment.lanes)
			@assert(lane.id.segment == segment.id)
			ids = sort!(collect(keys(lane.waypoints))::Vector{Int})
			for (waypoint_index,lla) in lane.waypoints
				lat, lon = lla.lat, lla.lon
				if convert_ll2utm
					utm = convert(UTM, LatLonAlt(deg2rad(lat), deg2rad(lon), NaN))
					eas, nor = utm.e, utm.n
				else
					eas, nor = lat, lon
				end
				pos = VecE3(eas,nor,0.0)

				node = StreetNode(WaypointID(segment.id, lane.id.lane, waypoint_index), pos,
					              NaN, NaN, NaN, NaN, NaN, -999, -999, NaN, NaN)

				push!(sn.nodes, node)
				node_index = add_vertex!(G)
				@assert(node_index == length(sn.nodes))

				node_map[WaypointID(segment.id, lane.id.lane, waypoint_index)] = node_index
			end

			# starting with a root node, find the next closest node to either end of the chain
			# to build the path
			root_bot = 1 # [index in ids]
			root_top = 1 # [index in ids]
			ids_yet_to_take = Set(collect(2:length(ids)))
			new_order = Int[1]
			while !isempty(ids_yet_to_take)
				closest_node_to_bot = -1 # index in ids
				best_dist_bot = Inf
				closest_node_to_top = -1 # index in ids
				best_dist_top = Inf

				node_index_T = node_map[WaypointID(segment.id, lane.id.lane, ids[root_top])]
				node_index_B = node_map[WaypointID(segment.id, lane.id.lane, ids[root_bot])]
				nodeT = sn.nodes[node_index_T]
				nodeB = sn.nodes[node_index_B]
				Tn, Te = nodeT.pos.x, nodeT.pos.y
				Bn, Be = nodeB.pos.x, nodeB.pos.y

				for id in ids_yet_to_take
					node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[id])]
					node = sn.nodes[node_index]

					index_n, index_e = node.pos.x, node.pos.y
					dnT, deT = Tn-index_n, Te-index_e
					dnB, deB = Bn-index_n, Be-index_e
					distT = sqrt(dnT*dnT + deT*deT)
					distB = sqrt(dnB*dnB + deB*deB)
					if distT < best_dist_top
						best_dist_top, closest_node_to_top = distT, id
					end
					if distB < best_dist_bot
						best_dist_bot, closest_node_to_bot = distB, id
					end
				end

				@assert(min(best_dist_top, best_dist_bot) < EDGE_DIST_THRESHOLD)
				if best_dist_bot < best_dist_top
					root_bot = closest_node_to_bot
					delete!(ids_yet_to_take, root_bot)
					unshift!(new_order, closest_node_to_bot)
				else
					root_top = closest_node_to_top
					delete!(ids_yet_to_take, root_top)
					push!(new_order, closest_node_to_top)
				end
			end

			# infer direction from whether ids are generally increasing or decreasing
			ids_generally_increasing = sum([(ids[new_order[i]] - ids[new_order[i-1]] > 0.0) for i in 2:length(ids)]) > 0.0
			node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[new_order[1]])]

			local i = 2
			while i ≤ length(new_order)
				nextnode_index = node_map[WaypointID(segment.id, lane.id.lane, ids[new_order[i]])]

				ids_generally_increasing ? add_edge!(G, node_index, nextnode_index) : add_edge!(G, nextnode_index, node_index)
				node_index = nextnode_index
				i += 1
			end

			# if this is a cycle - connect it
			# TODO(tim): use direction to infer whether this is correct
			node_index_T = node_map[WaypointID(segment.id, lane.id.lane, ids[root_top])]
			nodeT = sn.nodes[node_index_T]
			node_index_B = node_map[WaypointID(segment.id, lane.id.lane, ids[root_bot])]
			nodeB = sn.nodes[node_index_B]
			if dist(nodeT, nodeB) < CYCLE_CONNECTION_THRESHOLD
				if ids_generally_increasing
					add_edge!(G, node_index_T, node_index_B)
				else
					add_edge!(G, node_index_B, node_index_T)
				end
			end

			@assert(length(new_order) == length(ids))
			lane_ids[lane.id] = ids[new_order]
		end
	end

	# add exit edges
	verbosity < 1 || @printf("ADDING EXIT EDGES [%.2f]\n", time()-starttime)
	for segment in values(rndf.segments)
		for lane in values(segment.lanes)
			for (exitnode, entry) in lane.exits
				exit_waypoint = WaypointID(segment.id, lane.id.lane, exitnode)
				if haskey(node_map, exit_waypoint)
					add_edge!(G, node_map[exit_waypoint], node_map[entry])
				end
			end
		end
	end

	# add nodes to tiles
	verbosity < 1 || @printf("ADDING NODES TO TILES [%.2f]\n", time()-starttime)
	for segment in values(rndf.segments)
		for lane in values(segment.lanes)
			# keep the curve in the tile
			ids = lane_ids[lane.id]

			first_node_laneindex = 1 # NOTE(tim): index in lane
			first_node = sn.nodes[node_map[WaypointID(segment.id, lane.id.lane, ids[first_node_laneindex])]]
			index_e, index_n = utm2tileindex(first_node.pos.x, first_node.pos.y)
			tile = get_tile!(sn, index_e, index_n)
			n_pts_in_lane = length(lane.waypoints)
			@assert(n_pts_in_lane == length(ids))

			while first_node_laneindex ≤ n_pts_in_lane
				# find the last node in this lane that is still in the tile
				final_node_laneindex = findfirst(index->begin
													node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[index])]
													node = sn.nodes[node_index]
													node_e, node_n = utm2tileindex(node.pos.x, node.pos.y)
													return (index_e != node_e) || (index_n != node_n)
												end,
											(first_node_laneindex+1) : n_pts_in_lane)

				if final_node_laneindex == 0
					final_node_laneindex = n_pts_in_lane
				else
					final_node_laneindex += first_node_laneindex-1
				end

				node_indeces = Array(Int, length(first_node_laneindex : final_node_laneindex))
				nodes = Array(StreetNode, length(node_indeces))
				for (k,laneindex) in enumerate(first_node_laneindex : final_node_laneindex)
					node_index = node_map[WaypointID(segment.id, lane.id.lane, ids[laneindex])]
					node_indeces[k] = node_index
					nodes[k] = sn.nodes[node_index]
				end

				streetsegment = get_segment!(tile, segment.id)

				if has_lane(streetsegment, lane.id.lane)
					# need to stitch lanes together
					streetlane = streetsegment.lanes[lane.id.lane]

					if dist(nodes[end], sn.nodes[streetlane.node_indeces[1]]) < dist(sn.nodes[streetlane.node_indeces[end]], nodes[1])
						streetlane.node_indeces = [node_indeces, streetlane.node_indeces]
					else
						streetlane.node_indeces = [streetlane.node_indeces; node_indeces]
					end
				else
					width = isnan(lane.width) ? DEFAULT_LANE_WIDTH : lane.width*METERS_PER_FOOT

					lanetag = LaneTag(tile.index_e, tile.index_n, lane.id.segment, lane.id.lane)
					streetlane = StreetLane(lanetag, width, lane.boundary_left, lane.boundary_right, node_indeces, Curve(), false, false)
					add_lane!(streetsegment, streetlane)
				end

				first_node_laneindex = final_node_laneindex+1
				if first_node_laneindex ≤ n_pts_in_lane
					node = sn.nodes[node_map[WaypointID(segment.id, lane.id.lane, ids[first_node_laneindex])]]
					index_e, index_n = utm2tileindex(node.pos.x, node.pos.y)
					tile = get_tile!(sn, index_e, index_n)
				end
			end
		end
	end

	# attempt to connect hanging edges
	verbosity < 1 || @printf("CONNECTING HANGING EDGES [%.2f]\n", time()-starttime)

	const THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT = 30.0 # [m]
	const THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE  = deg2rad(20) # [rad]
	const THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 = deg2rad(5) # [rad]

	for node_index in vertices(G)

		node = sn.nodes[node_index]

		if outdegree(G, node_index) == 0 && indegree(G, node_index) > 0
			# NOTE(tim): this is an ending node

			prevnode_index = prev_node_index(sn, node_index)
			prevnode = sn.nodes[prevnode_index]

			θ = atan2(node.pos.y - prevnode.pos.y, node.pos.x - prevnode.pos.x)
			E = VecE2(node.pos.x, node.pos.y)
			vE = Vec.polar(1.0, θ)

			best_node_index = node_index
			best_score = Inf

			tile = get_tile(sn, node)
			for seg in values(tile.segments)
				for lane in values(seg.lanes)
					if (seg.id == node.id.segment && lane.id.lane == node.id.lane) || length(lane.node_indeces) < 2
						continue
					end

					for (i,testnode_index) in enumerate(lane.node_indeces)

						testnode = sn.nodes[testnode_index]

						θ₂ = i > 1 ? atan2(testnode.pos.y - sn.nodes[lane.node_indeces[i-1]].pos.y,
							               testnode.pos.x - sn.nodes[lane.node_indeces[i-1]].pos.x) :
									 atan2(sn.nodes[lane.node_indeces[i+1]].pos.y - testnode.pos.y,
									 	   sn.nodes[lane.node_indeces[i+1]].pos.x - testnode.pos.x)

						vT = Vec.polar(1.0, θ₂)
						T = VecE2(testnode.pos.x, testnode.pos.y)
						A = T - E

						c_p = dot(A,vE)
						if c_p > 0.0
							proj = vE * c_p
							perp_dist = hypot(A - proj)
							line_dist = hypot(proj)

							# NOTE(tim): angle between start and end orientation
							Δθ = abs(_signed_dist_btw_angles(θ, θ₂))

							# NOTE(tim): angle between dangling edge and projection
							θ₃ = atan2(testnode.pos.y - node.pos.y, testnode.pos.x - node.pos.x)
							Δ₃ = abs(_signed_dist_btw_angles(θ,θ₃))

							score = Δθ + 10.0*Δ₃

							if line_dist < THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT &&
								Δθ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE &&
								Δ₃ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 &&
								score < best_score

								best_score = score
								best_node_index = testnode_index
							end
						end
					end
				end
			end

			if node_index != best_node_index
				verbosity > 1 || println("ADDED END NODE HANGING EDGE ", node.pos.x, "  ", node.pos.y)
				add_edge!(G, node_index, best_node_index)
			end
		end
		if indegree(G, node_index) == 0 && outdegree(G, node_index) > 0
			# NOTE(tim): this is a starting node
			nextnode = sn.nodes[next_node_index(sn, node_index)]
			θ = atan2(nextnode.pos.y - node.pos.y, nextnode.pos.x - node.pos.x)
			E = VecE2(node.pos.x, node.pos.y)
			vE = Vec.polar(-1.0, θ)

			best_node_index = node_index
			best_score = Inf

			tile = get_tile(sn, node)
			for seg in values(tile.segments)
				for lane in values(seg.lanes)
					if (seg.id == node.id.segment && lane.id.lane == node.id.lane) || length(lane.node_indeces) < 2
						continue
					end

					for (i,testnode_index) in enumerate(lane.node_indeces)

						testnode = sn.nodes[testnode_index]

						θ₂ = i > 1 ? atan2(testnode.pos.y - sn.nodes[lane.node_indeces[i-1]].pos.y,
										   testnode.pos.x - sn.nodes[lane.node_indeces[i-1]].pos.x) :
									 atan2(sn.nodes[lane.node_indeces[i+1]].pos.y - testnode.pos.y,
									 	   sn.nodes[lane.node_indeces[i+1]].pos.x - testnode.pos.x)



						vT = Vec.polar(1.0, θ₂)
						T = VecE2(testnode.pos.x, testnode.pos.y)
						A = T - E

						c_p = dot(A,vE)
						if c_p > 0.0
							proj = vE * c_p
							perp_dist = hypot(A - proj)
							line_dist = hypot(proj)
							Δθ = abs(_signed_dist_btw_angles(θ,θ₂))

							θ₃ = atan2(node.pos.y - testnode.pos.y, node.pos.x - testnode.pos.x)
							Δ₃ = abs(_signed_dist_btw_angles(θ,θ₃))

							score = Δθ + 10.0*Δ₃

							if line_dist < THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT &&
								Δθ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE &&
								Δ₃ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 &&
								score < best_score

								best_score = score
								best_node_index = testnode_index
							end
						end
					end
				end
			end

			if node_index != best_node_index
				verbosity > 1 || println("ADDED START NODE HANGING EDGE ", node.pos.x, "  ", node.pos.y)
				add_edge!(G, best_node_index, node_index)
			end
		end
	end

	# check that each node from the graph is in a matching lane in its tile
	for node_index in vertices(G)
		node = sn.nodes[node_index]
		tile = get_tile(sn, node)
		if !has_lane(tile, node.id)
			println("MISSING ", node.id)
		end
	end

	# check that each lane is non-empty
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)
				@assert(!isempty(lane.node_indeces))
			end
		end
	end

	# compute curves for segments
	# TODO(tim): maybe do this across tile boundaries for the entire lane at a time
	verbosity < 1 || @printf("COMPUTE CURVES [%.2f]\n", time()-starttime)
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for key in keys(seg.lanes)

				lane = seg.lanes[key]

				node_indeces = lane.node_indeces
				node_index_prev = prev_node_index(sn, node_indeces[1])
				node_index_next = next_node_index(sn, node_indeces[end])
				has_prev = node_index_prev != node_indeces[1]
				has_next = node_index_next != node_indeces[end]

				n_nodes = length(node_indeces) + has_prev + has_next
				if n_nodes < 2
					println("DELETING LANE WITH $n_nodes nodes")
					delete!(seg.lanes, key)
					continue
				end

				pts = Array(Float64, 2, n_nodes)
				total = 0
				if has_prev
					total += 1
					node_prev = sn.nodes[node_index_prev]
					pts[1,total] = node_prev.pos.x
					pts[2,total] = node_prev.pos.y
				end
				for node_index in node_indeces
					total += 1
					node = sn.nodes[node_index]
					pts[1,total] = node.pos.x
					pts[2,total] = node.pos.y
				end
				if has_next
					total += 1
					node_next = sn.nodes[node_index_next]
					pts[1,total] = node_next.pos.x
					pts[2,total] = node_next.pos.y
				end

				if total > 1
					lane.curve = fit_curve(pts, lane.id.lane, DESIRED_DISTANCE_BETWEEN_CURVE_SAMPLES)
					if has_prev
						# adjust s so dist to first node is 0.0
						c = lane.curve
						extind = Curves.closest_point_extind_to_curve(c, pts[1,2], pts[2,2])
						pt = Curves.curve_at(c, extind)
						lane.curve = Curve(lane.id.lane, c.s - pt.s, c.x, c.y, c.t, c.k, c.k_d)
					end
					lane.has_leading_node = has_prev
					lane.has_trailing_node = has_next
				else
					error("LANE WITH ONE POINT!")
				end
			end
		end
	end

	# fix the extind and d_along for each waypoint
	verbosity < 1 || @printf("COMPUTE NODE EXTIND AND D_ALONG [%.2f]\n", time()-starttime)
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)
				guess = 1.0
				for node_index in lane.node_indeces
					node = sn.nodes[node_index]
					@assert(length(lane.curve) > 1)
					extind = closest_point_extind_to_curve_guess(lane.curve, node.pos.x, node.pos.y, guess)
					d_along = curve_at(lane.curve, extind).s

					@assert(!isnan(d_along))

					sn.nodes[node_index] = StreetNode(node.id,
					                                  node.pos,
					                                  extind,
					                                  d_along,
					                                  node.d_merge,
					                                  node.d_split,
					                                  node.d_end,
					                                  node.n_lanes_left,
					                                  node.n_lanes_right,
					                                  node.marker_dist_left,
					                                  node.marker_dist_right
					                                  )
				end
			end
		end
	end

	# compute remaining values for each waypoint
	verbosity < 1 || @printf("COMPUTE NODE D_ VALUES [%.2f]\n", time()-starttime)
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)
				for node_index in lane.node_indeces

					node = sn.nodes[node_index]

					d_end   = _distance_to_lane_end(sn, seg, lane.id.lane, node.extind)
					d_split = _distance_to_lane_split(sn, seg, lane.id.lane, node.extind)
					d_merge = _distance_to_lane_merge(sn, seg, lane.id.lane, node.extind)
					n_lanes_left, marker_dist_left, n_lanes_right, marker_dist_right = _calc_num_lanes_on_side(sn, tile, node_index)

					@assert(n_lanes_left ≥ 0)
					@assert(n_lanes_left ≥ 0)
					@assert(marker_dist_left > 0.0)
					@assert(marker_dist_right > 0.0)

					sn.nodes[node_index] = StreetNode(node.id,
					                                  node.pos,
					                                  node.extind,
					                                  node.d_along,
					                                  d_merge,
					                                  d_split,
					                                  d_end,
					                                  n_lanes_left,
					                                  n_lanes_right,
					                                  marker_dist_left,
					                                  marker_dist_right
					                                  )
				end
			end
		end
	end

	verbosity < 1 || @printf("DONE [%.2f]\n", time()-starttime)
	sn
end

function _calc_num_lanes_on_side(sn::StreetNetwork, center_tile::NetworkTile, node_index::Int)

	const THRESHOLD_PROJECTION_DELTA_ALONG_LANE = 0.5 # [m]
	const THRESHOLD_PROJECTION_DELTA_PERP_LANE = 20.0 # [m]
	const THRESHOLD_FLOW_ANGLE_DIFFERENCE = deg2rad(20) # [rad] how parallel neighboring lane needs to be to be considered a lane

	const LANE_SEP_MIN = 2.5 # min distance lanes should be apart [m]
	const LANE_SEP_MAX = 5.0 # max distance lanes should be apart [m]
	const N_DIST_DISCRETIZATIONS = 6

	const DIST_DISCRETIZATION = linspace(LANE_SEP_MIN/2, LANE_SEP_MAX/2, N_DIST_DISCRETIZATIONS)

	node = sn.nodes[node_index]
	seg_id = convert(Int, node.id.segment)
	lane_id = convert(Int, node.id.lane)
	current_lanetag = LaneTag(center_tile.index_e, center_tile.index_n, seg_id, lane_id)

	x, y = node.pos.x, node.pos.y
	θ    = curve_at(get_lane(center_tile, node.id).curve, node.extind).θ
	p_orig = VecE2(x,y)

	# project out left a series of discretizations
	# project out right a series of discretizations

	# LEFT
	unit_left = Vec.polar(1.0, θ+π/2)
	dist_add = 0.0
	finished = false
	active_lanetag = current_lanetag

	n_lanes_left = 0
	marker_dist_left = DEFAULT_LANE_WIDTH/2

	lanes_seen = Set{LaneTag}()
	push!(lanes_seen, active_lanetag)
	push!(lanes_seen, next_lanetag(sn, active_lanetag))
	push!(lanes_seen, prev_lanetag(sn, active_lanetag))

	while !finished
		finished = true
		for dist in DIST_DISCRETIZATION
			pt = p_orig + (dist+dist_add)*unit_left
			proj = project_point_to_streetmap(pt.x, pt.y, sn)
			if proj.successful
				lanetag = proj.lane.id
				if !in(lanetag, lanes_seen) # found a new one!
					perp_dist = hypot(x - proj.footpoint.x, y - proj.footpoint.y)
					if LANE_SEP_MIN < perp_dist-dist_add < LANE_SEP_MAX
						n_lanes_left += 1
						if n_lanes_left == 1
							marker_dist_left = perp_dist/2
						end
						finished = false

						active_lanetag = lanetag
						push!(lanes_seen, active_lanetag)
						push!(lanes_seen, next_lanetag(sn, active_lanetag))
						push!(lanes_seen, prev_lanetag(sn, active_lanetag))

						dist_add += perp_dist
					end
					break
				end
			end
		end
	end

	# RIGHT
	unit_right = Vec.polar(1.0, θ-π/2)
	dist_add = 0.0
	finished = false
	active_lanetag = current_lanetag

	n_lanes_right = 0
	marker_dist_right = DEFAULT_LANE_WIDTH/2

	lanes_seen = Set{LaneTag}()
	push!(lanes_seen, active_lanetag)
	push!(lanes_seen, next_lanetag(sn, active_lanetag))
	push!(lanes_seen, prev_lanetag(sn, active_lanetag))

	while !finished
		finished = true
		for dist in DIST_DISCRETIZATION
			pt = p_orig + (dist+dist_add)*unit_right
			proj = project_point_to_streetmap(pt.x, pt.y, sn)
			if proj.successful
				lanetag = proj.lane.id
				if !in(lanetag, lanes_seen) # found a new one!
					perp_dist = hypot(x-proj.footpoint.x, y-proj.footpoint.y)
					if LANE_SEP_MIN < perp_dist-dist_add < LANE_SEP_MAX

						n_lanes_right += 1
						if n_lanes_right == 1
							marker_dist_right = perp_dist/2
						end
						finished = false

						active_lanetag = lanetag
						push!(lanes_seen, active_lanetag)
						push!(lanes_seen, next_lanetag(sn, active_lanetag))
						push!(lanes_seen, prev_lanetag(sn, active_lanetag))

						dist_add += perp_dist
					end
					break
				end
			end
		end
	end

	(n_lanes_left, marker_dist_left, n_lanes_right, marker_dist_right)
end

function curves2streetnetwork(curves::CurveSet; verbosity::Int=0)
	rndf = curves2rndf(curves)
	rndf2streetnetwork(rndf, verbosity=verbosity, convert_ll2utm = false)
end
function curves2rndf(curves::CurveSet; node_separation::Real=5.0)

	seg = RNDF_Segment(1)

	rndf = RNDF()
	RoadNetwork.add_segment!(rndf, seg)

	nlanes = length(curves)
	for i = 1 : nlanes
		lane = RNDF_Lane(LaneID(1,i))
		seg.lanes[i] = lane

		curve = curves[i]
		lane.waypoints[1] = RoadNetwork.LatLonAlt(curve.x[1], curve.y[1])
		s_last = curve.s[1]
		waypoint_count = 1

		for j = 2 : length(curve)::Int
			if (curve.s[j] - s_last > node_separation) || j == length(curve)
				waypoint_count += 1
				lane.waypoints[waypoint_count] = RoadNetwork.LatLonAlt(curve.x[j], curve.y[j])
				s_last = curve.s[j]
			end
		end
	end

	rndf
end

dist(A::StreetNode, B::StreetNode) = hypot(A.pos.x - B.pos.x, A.pos.y - B.pos.y)

function create_straight_nlane_curves(
	n::Int;
    lane_spacing::Real=3.0, # distance between lanes [m]
    section_length::Real=3000.0, # [m]
    origin::Tuple{Real,Real,Real}=Tuple(0.0,-lane_spacing*(n-0.5),0.0), # (x,y,θ) origin of lane 1
    point_spacing::Real=2.0 # distance between points [m]
    )

    x,y,θ = origin

    npts = int(section_length / point_spacing) + 1
    point_spacing = section_length / (npts-1)

    cθ = cos(θ)
    sθ = sin(θ)
    cϕ = cos(θ+π/2)
    sϕ = sin(θ+π/2)

    curves = Array(Curve, n)
    for i = 1 : n

        w = (i-1)*lane_spacing
        x₀ = x + w*cϕ
        y₀ = y + w*sϕ

        s_arr = Array(Float64, npts)
        x_arr = Array(Float64, npts)
        y_arr = Array(Float64, npts)
        t_arr = fill(θ, npts)
        k_arr = zeros(Float64, npts)
        kd_arr = zeros(Float64, npts)

        s_arr[1] = 0.0
        x_arr[1] = x₀
        y_arr[1] = y₀

        for j = 2 : npts
            l = section_length * (j-1)/(npts-1)
            s_arr[j] = l
            x_arr[j] = x₀ + l*cθ
            y_arr[j] = y₀ + l*sθ
        end

        curves[i] = Curve(i, s_arr, x_arr, y_arr, t_arr, k_arr, kd_arr)
    end
    curves
end
function generate_straight_nlane_streetmap(
	nlanes::Integer;
	lane_spacing::Real=3.0, # distance between lanes [m]
    section_length::Real=5000.0, # [m]
    origin::Tuple{Real,Real,Real} = Tuple(0.0,-lane_spacing*(nlanes-0.5),0.0), # (x,y,θ) origin of lane 1
    point_spacing::Real=2.0 # distance between points [m]
	)

	curves = create_straight_nlane_curves(nlanes, lane_spacing=lane_spacing,
										  section_length=section_length, origin=origin,
										  point_spacing=point_spacing)
	rndf = curves2rndf(curves)
	rndf2streetnetwork(rndf, convert_ll2utm=false)
end


end # end module
