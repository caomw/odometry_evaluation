set grid
set term wxt 0
set title "Translational velocitiy error"
set xlabel "t [s]"
set ylabel "error [%]"
plot \
       "`echo $FILE`" using 1:(abs(($8-$20)/$8*100)) w l title "Velocity Error X", \
       "`echo $FILE`" using 1:(abs(($9-$21)/$9*100)) w l title "Velocity Error Y" , \
       "`echo $FILE`" using 1:(abs(($10-$22)/$10*100)) w l title "Velocity Error Z"

