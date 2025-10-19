# Points defining a spline. Theta allows holonomic vehicles to rotate along the trajecotry.

# JLG_CHANGES_START
# Navithor has a bug where this is defined as an int32, not a float64
int32 degree 1                            # Range: [1 … infinity) Defines the number of control points that influence
                                          # any given point on the curve. Increasing the degree increases continuity.
                                          # If not defined, the default value is 1.
# JLG_CHANGES_END

float64[] knot_vector                     # Range: [0.0 … 1.0] Sequence of parameter values that determines where and
                                          # how the control points affect the NURBS curve. knot_vector has size of number
                                          # of control points + degree + 1.

vda5050_msgs/ControlPoint[] control_points # List of JSON control_point objects defining the control points of the nurbs,
                                          # which includes the beginning and end point.
