#!/usr/bin/gnuplot --persist

if ("`rospack find path_planning`" eq '') {
    print "Could not find package path_planning. Make sure that you call 'source devel/setup.bash' in your Catkin workspace first."
    exit
}

set xrange [-0.5:9.5]
set yrange [-0.5:9.5]
set size ratio -1
set key outside
unset colorbox
set palette gray negative
set xtics out nomirror 0,1,9 scale 0.01,0.01
set ytics out nomirror 0,1,9 scale 0.01,0.01
set mxtics 2
set mytics 2
set grid mxtics front linetype 1 linecolor rgbcolor "gray40"
set grid mytics front linetype 1 linecolor rgbcolor "gray40"

set object 1 ellipse center 2,7 size 0.5,0.5 front fillstyle solid fillcolor rgbcolor "cyan"
set object 2 ellipse center 4,9 size 0.5,0.5 front fillstyle solid fillcolor rgbcolor "orange"
set label 3 "start" at 4,8.3 front center textcolor rgbcolor "orange"
set label 4 "goal"  at 2,7.8 front center textcolor rgbcolor "cyan"
numLogs=system("cat `rospack find path_planning`/data/log.txt | wc -l")
do for [n=1:numLogs-1] {
    plot "`rospack find path_planning`/data/map.txt" matrix with image title "", \
    -1 with line linewidth 2 lc 1 title "path", \
    "`rospack find path_planning`/data/log.txt" every ::0::n using (strcol(3) eq "open"  ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 2 title "opened node", \
    "`rospack find path_planning`/data/log.txt" every ::0::n using (strcol(3) eq "close" ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 3 title "closed node"
    pause 0.1
}
plot "`rospack find path_planning`/data/map.txt" matrix with image title "", \
    "`rospack find path_planning`/data/path.txt" u 1:2 with line linewidth 2 lc 1 title "path", \
    "`rospack find path_planning`/data/log.txt" every ::0::n using (strcol(3) eq "open"  ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 2 title "opened node", \
    "`rospack find path_planning`/data/log.txt" every ::0::n using (strcol(3) eq "close" ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 3 title "closed node"

