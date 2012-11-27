set grid
set term wxt 0
set title "Velocities-Yaw"
set xlabel "t [s]"
set ylabel "v [rad/s]"
plot "`echo $FILE`" using 28:14 w l title "Ground Truth" , "`echo $FILE`" using 28:27 w l title "Odometry"
