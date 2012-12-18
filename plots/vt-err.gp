set grid
set term wxt 0
set title "Translational velocitiy error"
set xlabel "t [s]"
set ylabel "error [m/s]"
plot \
       "`echo $FILE`" using 1:($8-$20) w l title "Velocity Error X", \
       "`echo $FILE`" using 1:($9-$21) w l title "Velocity Error Y" , \
       "`echo $FILE`" using 1:($10-$22) w l title "Velocity Error Z"

