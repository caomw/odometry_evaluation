set grid
set term wxt 0
set title "Translational velocitiy error"
set xlabel "t [s]"
set ylabel "error [m/s]"
plot \
       "`echo $FILE`" using 28:($9-$22) w l title "Velocity Error X", \
       "`echo $FILE`" using 28:($10-$23) w l title "Velocity Error Y" , \
       "`echo $FILE`" using 28:($11-$24) w l title "Velocity Error Z"

