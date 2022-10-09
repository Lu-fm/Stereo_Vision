#ifndef SEED_SERIAL
#define SEED_SERIAL
#include <seed_serial/order.h>
#include <seed_serial/state.h>
#define T_ 3.0



bool loosen_claw_state=false;
bool shrink_claw_state=false;
bool detect_state=false;

axis_angle_msg origin     ( -2.90,   30.33,   100.44,   -0.50,   40.94,       0.0,   7.0, 1500.0);
axis_angle_msg wp_up_msg  ( 28.24,   65.55,   108.7,     0.49,   66.55,   -5.73,   7.0, 1500.0);
axis_angle_msg wp_down_msg( 28.24,   68.35,   110.39,       0,   67.24,   -5.74,   7.0, 1500.0);
axis_angle_msg detect_msg (-23.26,   67.59,   111.11,       0,   67.01,   -5.74,   7.0, 1500.0);

position_msg origin_pos  (4467, -155,  2398);//pos_choose:1
position_msg wp_up_pos   (4283, 2277,  109 );//pos_choose:2
position_msg wp_down_pos (4302, 2287,  -55 );//pos_choose:3
position_msg detect_pos  (4584, -1946, 54 );//pos_choose:4

#endif