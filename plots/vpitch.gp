set grid
set term wxt 0
set title "Velocities-Pitch"
set xlabel "t [s]"
set ylabel "v [rad/s]"
set style line 1 lt 1 lc rgb "green" lw 1
set style line 2 lt 1 lc rgb "blue" lw 1
plot "`echo $FILE`" using 28:13 w l ls 1 title "Ground Truth" , "`echo $FILE`" using 28:26 w l ls 2 title "Odometry"
