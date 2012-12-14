set grid
set term wxt 0
set title "Rotational velocitiy error"
set xlabel "t [s]"
set ylabel "error [%]"
plot \
       "`echo $FILE`" using (abs(($12-$25)/$12*100)) w l title "Velocity Error Roll", \
       "`echo $FILE`" using (abs(($13-$26)/$13*100)) w l title "Velocity Error Pitch" , \
       "`echo $FILE`" using (abs(($14-$27)/$14*100)) w l title "Velocity Error Yaw"

