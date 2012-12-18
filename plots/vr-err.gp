set grid
set term wxt 0
set title "Rotational velocitiy error"
set xlabel "t [s]"
set ylabel "error [rad/s]"
plot \
       "`echo $FILE`" using 1:($11-$23) w l title "Velocity Error Roll", \
       "`echo $FILE`" using 1:($12-$24) w l title "Velocity Error Pitch" , \
       "`echo $FILE`" using 1:($13-$25) w l title "Velocity Error Yaw"

