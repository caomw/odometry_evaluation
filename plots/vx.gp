set grid
set term wxt 0
set title "Velocities-X"
set xlabel "t [s]"
set ylabel "v [m/s]"
plot "`echo $FILE`" using 28:9 w l title "Ground Truth" , "`echo $FILE`" using 28:22 w l title "Odometry"
