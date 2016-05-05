#!/usr/bin/gnuplot --persist

set terminal wxt size 1024,768

if ("`rospack find inverse_kinematics`" eq '') {
    print "Could not find package inverse_kinematics. Make sure that you call 'source devel/setup.bash' in your Catkin workspace first."
    exit
}

set xrange [-0.1:4.0]
set yrange [-1.0:2.0]
set size ratio -1
set parametric
set trange [0:2*pi]

rx1 = 0.1; ry1 = 0.2
rx2 = 0.1; ry2 = 0.2
tx  = 2.5; ty  = 0.8
k   = 0.1

a0 = 1.6
a1 = 1.2
a2 = 0.7
h = 0.1

gw = 0.02        # gripper width
gl1 = 0.7 * h    # gripper length 1
gl2 = 1.3 * h    # gripper length 2

wh = 0.05        # glass handle height
wb = 0.03        # glass bottom width
wt = 0.04        # glass top width
wht = 0.07       # glass height 2
wht2 = 0.18      # glass height 2

set object 30 rectangle from 2.25,0.65 to 2.75,0.75 fill solid border -1 fillcolor rgb "gray25" back
set object 31 rectangle from 2.45,0.00 to 2.55,0.65 fill solid border -1 fillcolor rgb "gray25" back

numLines=system("wc -l `rospack find inverse_kinematics`/data/glass.log")
do for [i=0:numLines-1] {
    set table "/dev/null"
    plot "`rospack find inverse_kinematics`/data/glass.log" every ::i::i using 1:(q0=$1):(q1=$2):(q2=$3)
    x1 =      a0 * cos(q0);        y1 =      a0 * sin(q0)
    x2 = x1 + a1 * cos(q0+q1);     y2 = y1 + a1 * sin(q0+q1)
    x3 = x2 + a2 * cos(q0+q1+q2);  y3 = y2 + a2 * sin(q0+q1+q2)
    x4 = x3 + h  * cos(q0+q1+q2);  y4 = y3 + h  * sin(q0+q1+q2)

    # points for gripper
    gx1 = x3 + gl1 * cos(q0+q1+q2) - gw * sin(q0+q1+q2)
    gy1 = y3 + gl1 * sin(q0+q1+q2) + gw * cos(q0+q1+q2)
    gx2 = x3 + gl1 * cos(q0+q1+q2) + gw * sin(q0+q1+q2)
    gy2 = y3 + gl1 * sin(q0+q1+q2) - gw * cos(q0+q1+q2)

    gx3 = x3 + gl2 * cos(q0+q1+q2) - gw * sin(q0+q1+q2)
    gy3 = y3 + gl2 * sin(q0+q1+q2) + gw * cos(q0+q1+q2)
    gx4 = x3 + gl2 * cos(q0+q1+q2) + gw * sin(q0+q1+q2)
    gy4 = y3 + gl2 * sin(q0+q1+q2) - gw * cos(q0+q1+q2)

    # points for glass
    wx1 = x3 + h      * cos(q0+q1+q2) + wh * sin(q0+q1+q2)
    wy1 = y3 + h      * sin(q0+q1+q2) - wh * cos(q0+q1+q2)

    wx2 = x3 + (h-wb) * cos(q0+q1+q2) + wh  * sin(q0+q1+q2)
    wy2 = y3 + (h-wb) * sin(q0+q1+q2) - wh  * cos(q0+q1+q2)

    wx3 = x3 + (h+wb) * cos(q0+q1+q2) + wh  * sin(q0+q1+q2)
    wy3 = y3 + (h+wb) * sin(q0+q1+q2) - wh  * cos(q0+q1+q2)

    wx4 = x3 + h      * cos(q0+q1+q2) - wh  * sin(q0+q1+q2)
    wy4 = y3 + h      * sin(q0+q1+q2) + wh  * cos(q0+q1+q2)

    wx5 = x3 + (h-wt) * cos(q0+q1+q2) - wht * sin(q0+q1+q2)
    wy5 = y3 + (h-wt) * sin(q0+q1+q2) + wht * cos(q0+q1+q2)

    wx6 = x3 + (h+wt) * cos(q0+q1+q2) - wht * sin(q0+q1+q2)
    wy6 = y3 + (h+wt) * sin(q0+q1+q2) + wht * cos(q0+q1+q2)

    wx7 = x3 + (h-wt) * cos(q0+q1+q2) - wht2 * sin(q0+q1+q2)
    wy7 = y3 + (h-wt) * sin(q0+q1+q2) + wht2 * cos(q0+q1+q2)

    wx8 = x3 + (h+wt) * cos(q0+q1+q2) - wht2 * sin(q0+q1+q2)
    wy8 = y3 + (h+wt) * sin(q0+q1+q2) + wht2 * cos(q0+q1+q2)

    unset table

    set object  1 circle at  0,0  size 0.05 fill solid border -1 fillcolor rgb "gray30" front
    set object  2 circle at x1,y1 size 0.05 fill solid border -1 fillcolor rgb "gray30" front
    set object  3 circle at x2,y2 size 0.05 fill solid border -1 fillcolor rgb "gray30" front
    set object  4 circle at x3,y3 size 0.05 fill solid border -1 fillcolor rgb "gray30" front
    set object  5 circle at x4,y4 size 0.01 fill solid border -1 fillcolor rgb "gray30" front

    set arrow   6 from  0, 0 to x1,y1       nohead lw 4 lc rgb "gray40" back
    set arrow   7 from x1,y1 to x2,y2       nohead lw 4 lc rgb "gray40" back
    set arrow   8 from x2,y2 to x3,y3       nohead lw 4 lc rgb "gray40" back
 
    # gripper
    set arrow   9 from  x3, y3 to gx1,gy1 nohead lw 5 lc rgb "gray20" back
    set arrow  10 from  x3, y3 to gx2,gy2 nohead lw 5 lc rgb "gray20" back
    set arrow  11 from gx1,gy1 to gx3,gy3 nohead lw 3 lc rgb "gray20" back
    set arrow  12 from gx2,gy2 to gx4,gy4 nohead lw 3 lc rgb "gray20" back

    # glass
    set arrow 13 from wx1,wy1 to wx4,wy4 nohead lw 3 lc rgb "black" back
    set arrow 14 from wx2,wy2 to wx3,wy3 nohead lw 3 lc rgb "black" back
    set object 15 polygon from wx4,wy4 to wx5,wy5 to wx7,wy7 to wx8,wy8 to wx6,wy6 to wx4,wy4 fill solid border -1 fillcolor rgb "light-blue" front

    plot tx + (t < 0.5*pi ? rx1 * cos(t) : (rx2 + k*(t-0.5*pi)) * cos(2.0 * pi - t)),ty + (t < 0.5*pi ? ry1 * sin(t) : ry1 + ry2 + ry2 * sin(2.0 * pi - t)) w l lw 2 title "planned trajectory", \
       "`rospack find inverse_kinematics`/data/glass.log" every ::0::i using 4:5 with line lw 2 lc 3 title "executed trajectory"
    pause 0.01

}
