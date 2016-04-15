#!/usr/bin/gnuplot -persist

set contour
set key off
set hidden3d
set cntrparam bspline
set size ratio -1
set yrange [400:800]
set xrange [200:600]
set cntrparam levels discrete 0
unset colorbox 
splot "`rospack find signed_distance_function`/data/result.txt" matrix every 5:5 with pm3d t ''
