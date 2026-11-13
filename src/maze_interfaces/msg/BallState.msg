# Contract 2 (docs/interfaces.md): the ball, every camera frame, on /ball/state.

std_msgs/Header header             # stamp = capture time of the camera image this came from (not the
                                   # publish time); frame_id = "maze_board"
float64 x                          # ball center projected on the floor plane (z = 0), m
float64 y                          # m
float64 vx                         # m/s
float64 vy                         # m/s
float64[4] position_covariance     # 2x2 row-major [xx, xy, yx, yy], m^2; all zeros if unknown
bool tracking_ok                   # false = ball not detected recently; x, y, vx, vy are a prediction
