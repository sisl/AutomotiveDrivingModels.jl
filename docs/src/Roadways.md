# Roadways

## Data Types and Accessing Elements
The data structure to represent roadways can be decomposed as follows:

- **Roadway** The high level type containing all the information. It contains a list of `RoadSegment`.
  - **RoadSegment**: a vector of lanes
    - **Lane**: A driving lane on a roadway. It identified by a `LaneTag`. A lane is defined by a curve which
represents a center line and a width. In addition it has attributed like speed limit. A lane can be connected to other lane in the roadway, the connection are specified in the exits
and entrances fields.

**Lower level types:**

- **Curves**: A curve is a list of `CurvePt`
- **CurvePt**: the lowest level type. It represents a point on a curve by its global position, position along the curve, curvature at this point and derivative of the curvature at this point. Other types like `CurveIndex` or `CurveProjection` are used to identify a curve point along a curve. 

```@docs
      Roadway
      RoadSegment
      move_along
```

## Roadway generation

`AutomotiveDrivingModels.jl` provide high level functions to generate road networks by drawing straight road segment and circular curves. Two predefined road network can be generated easily: multi-lane straight roadway sections and a multi-lane stadium shaped roadway.

```@docs
      gen_straight_curve
      gen_straight_segment
      gen_bezier_curve
      gen_straight_roadway
      gen_stadium_roadway
```


## Lane

The `Lane` data structure represent a driving lane in the roadway. The default lane width is 3m. It contains all the low level geometry information.

```@docs
      Lane
      LaneTag
      SpeedLimit
      LaneBoundary
      LaneConnection
      is_in_exits
      is_in_entrances
      connect!
      is_between_segments_hi
      is_between_segments
      has_segment
      has_lanetag
      next_lane
      prev_lane
      has_next
      has_prev
      next_lane_point
      prev_lane_point
      n_lanes_left
      n_lanes_right
```

## Frenet frame 

The Frenet frame is a lane relative frame to represent a position on the road network.

```@docs
      Frenet
      get_posG
```

## Accessing objects and projections

The main `roadway` object can be indexed by different object to access different elements
such as lane or curve points:
- `LaneTag`: indexing roadway by a lane tag will return the lane associated to the lane tag 
- `RoadIndex`: indexing roadway by a road index will return the curve point associated to this index

```@docs
  RoadIndex
  CurveIndex
  RoadProjection
  proj(posG::VecSE2{T}, lane::Lane, roadway::Roadway;move_along_curves::Bool = true ) where T<: Real
  proj(posG::VecSE2{T}, seg::RoadSegment, roadway::Roadway) where T<: Real
  proj(posG::VecSE2{T}, roadway::Roadway) where T<: Real
  Base.getindex(lane::Lane, ind::CurveIndex, roadway::Roadway)
  Base.getindex(roadway::Roadway, segid::Int)
  Base.getindex(roadway::Roadway, tag::LaneTag)
```

## Low level

```@docs
      Curve
      CurvePt
      CurveProjection
      is_at_curve_end
      get_lerp_time
      index_closest_to_point
      get_curve_index
      lerp(A::VecE2{T}, B::VecE2{T}, C::VecE2{T}, D::VecE2{T}, t::T) where T<:Real
      lerp(A::VecE2{T}, B::VecE2{T}, C::VecE2{T}, t::T) where T<:Real
```

## 1D roadway

```@docs
      StraightRoadway
      mod_position_to_roadway
      get_headway
```

## Read and Write roadways

```@docs
      Base.read(io::IO, ::MIME"text/plain", ::Type{Roadway})
      Base.write(io::IO, ::MIME"text/plain", roadway::Roadway)
```