MY_PATH="`dirname \"$0\"`"
FILE=$1 gnuplot -p $MY_PATH/trajectories.gp
FILE=$1 gnuplot -p $MY_PATH/vx.gp
FILE=$1 gnuplot -p $MY_PATH/vy.gp
FILE=$1 gnuplot -p $MY_PATH/vz.gp
FILE=$1 gnuplot -p $MY_PATH/vroll.gp
FILE=$1 gnuplot -p $MY_PATH/vpitch.gp
FILE=$1 gnuplot -p $MY_PATH/vyaw.gp
FILE=$1 gnuplot -p $MY_PATH/vr-err.gp
FILE=$1 gnuplot -p $MY_PATH/vt-err.gp
