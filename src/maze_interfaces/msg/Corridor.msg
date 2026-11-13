# A convex safe region for the BALL CENTER, axis-aligned in maze_board, m.
# Wall clearance (ball radius + margin) is already taken off, so a controller can use the bounds
# directly as constraints on the ball state's x and y.
float64 x_min
float64 x_max
float64 y_min
float64 y_max
