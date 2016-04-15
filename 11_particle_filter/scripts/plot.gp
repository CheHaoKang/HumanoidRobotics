#!/usr/bin/gnuplot

if ("`rospack find particle_filter`" eq '') {
    print "Could not find package particle_filter. Make sure that you call 'source devel/setup.bash' in your Catkin workspace first."
    exit
}

set xrange [-1:11]
set yrange [-35:100]
bin(x, s) = s*int(x/s)
ry=-10
ly=-25
a=0.3
b=5.0

set style fill solid
set xzeroaxis
set ytics 0, 20, 100
set xtics 0, 2, 10

set format y "%.0f%%"

set key spacing 1.5

do for [i=0:19] {
  set terminal png enhanced
  set output sprintf("`rospack find particle_filter`/data/distribution_%02d.png", i)
  set title sprintf("t = %i", i)
  plot "`rospack find particle_filter`/data/result" index i using (bin($1, 0.1)):(100./250.) smooth frequency with boxes title "particle distribution", \
       "`rospack find particle_filter`/data/groundTruth" u 3:(ry) every ::i::i w p pt 19 ps 4 lc 3 lw 3 title "robot true position", \
       "" u 3:(ry):(0.2):(0) every ::i::i with vectors nohead lc 3 lw 3 title "", \
       "`rospack find particle_filter`/data/light_sources" u 1:(ly) with points pt 19 ps 4 lc 9 lw 3 title "lights", \
       "" u ($1+a):(ly+0):( a):( 0) with vectors nohead lc 9 lw 3 title "", \
       "" u ($1+0):(ly+b):( 0):( b) with vectors nohead lc 9 lw 3 title "", \
       "" u ($1-a):(ly+0):(-a):( 0) with vectors nohead lc 9 lw 3 title "", \
       "" u ($1+0):(ly-b):( 0):(-b) with vectors nohead lc 9 lw 3 title "", \
       "" u ($1+0.7*a):(ly+0.7*b):( 0.7*a):( 0.7*b) with vectors nohead lc 9 lw 3 title "", \
       "" u ($1-0.7*a):(ly+0.7*b):(-0.7*a):( 0.7*b) with vectors nohead lc 9 lw 3 title "", \
       "" u ($1+0.7*a):(ly-0.7*b):( 0.7*a):(-0.7*b) with vectors nohead lc 9 lw 3 title "", \
       "" u ($1-0.7*a):(ly-0.7*b):(-0.7*a):(-0.7*b) with vectors nohead lc 9 lw 3 title ""
  set output
}

print "Wrote distribution plots to `rospack find particle_filter`/data/distribution_*.png"



