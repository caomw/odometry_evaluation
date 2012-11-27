set grid
set term wxt 0
set title "Velocities-Pitch"
set xlabel "t [s]"
set ylabel "v [rad/s]"
plot "`echo $FILE`" using 28:13 w l title "Ground Truth" , "`echo $FILE`" using 28:26 w l title "Odometry"
