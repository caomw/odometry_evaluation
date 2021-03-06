set grid
set term wxt 0
set title "Velocities-X"
set xlabel "t [s]"
set ylabel "v [m/s]"
set style line 1 lt 1 lc rgb "green" lw 1
set style line 2 lt 1 lc rgb "blue" lw 1
plot "`echo $FILE`" using 1:8 w l ls 1 title "Ground Truth" , "`echo $FILE`" using 1:20 w l ls 2 title "Odometry"
