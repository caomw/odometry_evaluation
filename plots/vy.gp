set grid
set term wxt 0
set title "Velocities-Y"
set xlabel "t [s]"
set ylabel "v [m/s]"
plot "`echo $FILE`" using 28:10 w l title "Ground Truth" , "`echo $FILE`" using 28:23 w l title "Odometry"
