# Contract 4 (docs/interfaces.md): board tilt. Commanded on /board/tilt_cmd (ball control -> arm
# control), measured on /board/tilt_state (arm control -> ball control, system ID).
#
# Defined by the effect on the ball, relative to the board's level home pose:
#   tilt_x > 0 lowers the board's +x edge, so the ball accelerates toward +x:  a_x = 5/7 g sin(tilt_x)
#   tilt_y > 0 lowers the board's +y edge, so the ball accelerates toward +y:  a_y = 5/7 g sin(tilt_y)
# As rotations of maze_board (right-handed): tilt_x = +angle about board +y,
#                                            tilt_y = -angle about board +x.

std_msgs/Header header   # stamp = when the command was computed (cmd) or measured (state);
                         # frame_id = "maze_board"
float64 tilt_x           # rad
float64 tilt_y           # rad
