#!/usr/bin/gnuplot --persist

if ("`rospack find irm`" eq '') {
    print "Could not find package irm. Make sure that you call 'source devel/setup.bash' in your Catkin workspace first."
    exit
}

set xrange [-4:7]
set yrange [-0.5:6.5]
set palette defined ( -1 "white", 0 "red", 0.5 "yellow", 1 "green" )
unset colorbox
set size ratio -1
set grid xtics ytics front
set arrow 1 from 0,-0.5 to 0,6.5 nohead lt 1 lc rgb "black" front
set arrow 2 from -4,0 to 7,0 nohead lt 1 lc rgb "black" front
plot "`rospack find irm`/data/rm.txt" u ($1/10-6.5):($2/10-0.5):3 matrix with image title ""
