#!/usr/bin/gnuplot --persist
unset surface
set table "`rospack find signed_distance_function`/data/contour.txt"
set contour
set cntrparam bspline
set cntrparam levels discrete 0
splot "`rospack find signed_distance_function`/data/result.txt" matrix every 5:5 w l
unset table
set size ratio -1
set yrange [400:800]
set xrange [250:600]
set key outside bottom center
plot "`rospack find signed_distance_function`/data/contour.txt" w l lt -1 lw 1.5 t 'walls', "`rospack find signed_distance_function`/data/data.txt" u 1:2 every ::2 w l lt 1 lw 2 t 'robot trajectory'
