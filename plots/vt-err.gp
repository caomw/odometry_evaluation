set grid
set term wxt 0
set title "Velocitiy errors"
plot \
       "`echo $FILE`" using ($9-$22):28 w l title "Velocity Error X", \
       "`echo $FILE`" using ($10-$23):28 w l title "Velocity Error Y" , \
       "`echo $FILE`" using ($11-$24):28 w l title "Velocity Error Z"

