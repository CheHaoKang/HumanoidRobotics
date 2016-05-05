#!/usr/bin/gnuplot -persist

set terminal wxt size 1024,768

if ("`rospack find footstep_planning`" eq '') {
    print "Could not find package footstep_planning. Make sure that you call 'source devel/setup.bash' in your Catkin workspace first."
    exit
}

set xrange [0:4]
set yrange [0:4]
set size ratio -1
set key off


add_footprint(x,y,phi,foot) = sprintf('call "`rospack find footstep_planning`/scripts/plot_footprint.gp" "%f" "%f" "%f" "%s";',x,y,phi,foot)

#set table '/dev/null'
#CMD = ''
#plot "`rospack find footstep_planning`/data/path.txt" u 1:(CMD = CMD.add_footprint($1,$2,$3,strcol(4)))
#unset table

#eval(CMD)

n=system("wc -l `rospack find footstep_planning`/data/path.txt")

set object 1000 circle at 1.0,0.2 size 0.07 fill solid 0.25 fc rgb 'blue' front
set object 1001 circle at 2.0,2.0 size 0.07 fill solid 0.25 fc rgb 'blue' front
set label 1002 "start" at 0.9,0.2  front right textcolor rgbcolor 'black'
set label 1003 "goal"  at 2.0,2.15 front center textcolor rgbcolor 'black'

do for [i=0:n-1] {
    set table '/dev/null'
    plot "`rospack find footstep_planning`/data/path.txt" every ::i::i u 1:(CMD = add_footprint($1,$2,$3,strcol(4)))
    unset table
    eval(CMD)
    set object (i+1) fillstyle solid
    if (i > 1) {
        set object (i-1) fillstyle solid 0.25
    }
    plot "`rospack find footstep_planning`/data/map.png" binary filetype=png dx=0.01 dy=0.01 with rgbimage title ''
    pause 0.05
}
