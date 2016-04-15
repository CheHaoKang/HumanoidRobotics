#!/usr/bin/gnuplot -persist

set title "trajectory of the left hand"
set xlabel "x"
set ylabel "y"
set zlabel "z"
splot "`rospack find forward_kinematics`/data/result.txt" u 1:2:3 w l lw 3 title "your solution", "`rospack find forward_kinematics`/data/joints.txt" u 6:7:8 title "ground truth" w l lc 3
