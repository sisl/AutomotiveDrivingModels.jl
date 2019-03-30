# Roadways

The data structure to represent roadways can be decomposed as follows:

**Roadways**
The high level type containing all the information.

    - RoadSegment: a vector of lanes
    - Lane: 
    - Curves 
      - A curve is a list of CurvePt 
      - CurvePt: the lowest level type. It represents a point on a curve by its global position, position along the curve, curvature at this point and      derivative of the curvature at this point
      - CurveIndex
      - CurveProjection
  
