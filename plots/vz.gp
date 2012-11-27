set grid
set term wxt 0
set title "Velocities-Z"
set xlabel "t [s]"
set ylabel "v [m/s]"
plot "`echo $FILE`" using 28:11 w l title "Ground Truth" , "`echo $FILE`" using 28:24 w l title "Odometry"
