#!/usr/bin/gnuplot --persist

set title "ICP" 
set xlabel "X"
set ylabel "Y"
set autoscale

plot "data/closestPointResult.txt" using 1:2 title 'P1 -- closest point' with lines, \
      "data/pointToLineResult.txt" using 1:2 title 'P1 -- point-to-line' with lines, \
      "data/data.txt" using 1:2 title 'Q' with lines, \
	"data/data.txt" using 3:4 title 'P' with lines
