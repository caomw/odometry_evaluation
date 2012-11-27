set grid
set term wxt 
set view equal xyz
set title "Trajectories"
splot "`echo $FILE`" using 2:3:4 w l lw 1 title "Ground Truth" , "`echo $FILE`" using 15:16:17:28 w l lw 1 palette title "Odometry"
