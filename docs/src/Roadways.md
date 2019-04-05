# Roadways

## Data Types and Accessing Elements
The data structure to represent roadways can be decomposed as follows:

- **Roadway** The high level type containing all the information. It contains a list of `RoadSegment`.
  - **RoadSegment**: a vector of lanes
    - **Lane**: A driving lane on a roadway. It identified by a `LaneTag`. A lane is defined by a curve which
represents a center line and a width. In addition it has attributed like speed limit. A lane can be connected to other lane in the roadway, the connection are specified in the exits
and entrances fields.
      - **Curves**: A curve is a list of `CurvePt`
      - **CurvePt**: the lowest level type. It represents a point on a curve by its global position, position along the curve, curvature at this point and derivative of the curvature at this point. Other types like `CurveIndex` or `CurveProjection` are used to identify a curve point along a curve. 

## Roadway generation

`AutomotiveDrivingModels.jl` provide high level functions to generate road networks by drawing straight road segment and circular curves. Two predefined road network can be generated easily: multi-lane straight roadway sections and a multi-lane stadium shaped roadway.

```@docs
      gen_straight_curve
      gen_straight_segment
      gen_bezier_curve
      gen_straight_roadway
      gen_stadium_roadway
```

## Frenet frame 

The Frenet frame is ...



## Low level API
