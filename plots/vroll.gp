set grid
set term wxt 0
set title "Velocities-Roll"
set xlabel "t [s]"
set ylabel "v [rad/s]"
plot "`echo $FILE`" using 28:12 w l title "Ground Truth" , "`echo $FILE`" using 28:25 w l title "Odometry"
