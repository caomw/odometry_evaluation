set grid
set term wxt 0
set title "Rotational velocitiy error"
set xlabel "t [s]"
set ylabel "error [rad/s]"
plot \
       "`echo $FILE`" using ($12-$25) w l title "Velocity Error Roll", \
       "`echo $FILE`" using ($13-$26) w l title "Velocity Error Pitch" , \
       "`echo $FILE`" using ($14-$27) w l title "Velocity Error Yaw"

