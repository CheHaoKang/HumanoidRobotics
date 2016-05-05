#!/usr/bin/gnuplot --persist

show_random_node = 0

if ("`rospack find rrt`" eq '') {
    print "Could not find package rrt. Make sure that you call 'source devel/setup.bash' in your Catkin workspace first."
    exit
}

set terminal wxt size 1024,768

startcolor = "blue"
goalcolor  = "dark-green"
pathcolor  = "red"
randomcolor = "dark-violet"
startx = 7
starty = 7
goalx = 11
goaly = 33

set xrange [-0.5:39.5]
set yrange [-0.5:39.5]
set size ratio -1
set key outside
unset colorbox
set palette gray negative
set xtics out nomirror 0,1,39 scale 0.01,0.01
set ytics out nomirror 0,1,39 scale 0.01,0.01
set mxtics 2
set mytics 2
set grid mxtics front linetype 1 linewidth 1 linecolor rgbcolor "gray40"
set grid mytics front linetype 1 linewidth 1 linecolor rgbcolor "gray40"

set object 1 ellipse center startx,starty size 0.5,0.5 front fillstyle solid fillcolor rgbcolor startcolor
set object 2 ellipse center  goalx,goaly  size 0.5,0.5 front fillstyle solid fillcolor rgbcolor goalcolor
set label 3 "start" at startx,(starty - 2) front center textcolor rgbcolor startcolor
set label 4 "goal"  at  goalx,(goaly  - 2)  front center textcolor rgbcolor goalcolor
numLogs=system("cat `rospack find rrt`/data/log.txt | wc -l")
do for [n=1:numLogs-1] {
    if (show_random_node) {
      plot "`rospack find rrt`/data/map.txt" matrix with image title "", \
         "`rospack find rrt`/data/log.txt" every ::0::(n-1) using (strcol(7) eq "forward"  ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb startcolor  title "forward tree", \
         "`rospack find rrt`/data/log.txt" every ::0::(n-1) using (strcol(7) eq "backward" ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb goalcolor   title "backward tree", \
         "`rospack find rrt`/data/log.txt" every ::n::n using 5:6 with points linecolor rgb randomcolor pointtype 7 pointsize 2 title "random node", \
         -1 with line linewidth 4 linecolor rgb "red" title "path"
      pause 0.2
      plot "`rospack find rrt`/data/map.txt" matrix with image title "", \
           "`rospack find rrt`/data/log.txt" every ::0::n using (strcol(7) eq "forward"  ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb startcolor  title "forward tree", \
           "`rospack find rrt`/data/log.txt" every ::0::n using (strcol(7) eq "backward" ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb goalcolor   title "backward tree", \
           "`rospack find rrt`/data/log.txt" every ::n::n using 5:6 with points linecolor rgb randomcolor pointtype 7 pointsize 2 title "random node", \
           -1 with line linewidth 4 linecolor rgb "red" title "path"
      pause 0.2
    } else {
      plot "`rospack find rrt`/data/map.txt" matrix with image title "", \
         "`rospack find rrt`/data/log.txt" every ::0::(n-1) using (strcol(7) eq "forward"  ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb startcolor  title "forward tree", \
         "`rospack find rrt`/data/log.txt" every ::0::(n-1) using (strcol(7) eq "backward" ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb goalcolor   title "backward tree", \
         -1 with line linewidth 4 linecolor rgb "red" title "path"
      pause 0.05
    }
}
if (show_random_node) {
  plot "`rospack find rrt`/data/map.txt" matrix with image title "", \
    "`rospack find rrt`/data/log.txt" every ::0::n using (strcol(7) eq "forward"  ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb startcolor  title "forward tree", \
    "`rospack find rrt`/data/log.txt" every ::0::n using (strcol(7) eq "backward" ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb goalcolor   title "backward tree", \
    -1 with points linecolor rgb randomcolor pointtype 7 pointsize 2 title "random node", \
    "`rospack find rrt`/data/path.txt" with line linewidth 4 linecolor rgb pathcolor title "path"
} else {
  plot "`rospack find rrt`/data/map.txt" matrix with image title "", \
    "`rospack find rrt`/data/log.txt" every ::0::n using (strcol(7) eq "forward"  ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb startcolor  title "forward tree", \
    "`rospack find rrt`/data/log.txt" every ::0::n using (strcol(7) eq "backward" ? $1 : 1/0):2:($3-$1):($4-$2) with vectors nohead linewidth 2 linecolor rgb goalcolor   title "backward tree", \
    "`rospack find rrt`/data/path.txt" with line linewidth 4 linecolor rgb pathcolor title "path"
}
