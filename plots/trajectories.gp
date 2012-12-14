set grid
set term wxt 
set view equal xyz
set title "Trajectories"
set style line 1 lt 1 lc rgb "green" lw 1
set xlabel "X [m]"
set ylabel "Y [m]"
set zlabel "Z [m]"
splot "`echo $FILE`" using 2:3:4 w l ls 1 title "Ground Truth", "`echo $FILE`" using 15:16:17:28 w l palette title "Odometry"
