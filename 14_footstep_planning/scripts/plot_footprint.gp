if ($# != 4) {
    print 'Usage: plot_footprint.gp x y theta foot'
    exit
}

x = $0
y = $1
theta = $2
foot = "$3"

w = 0.08
h = 0.044

if (!exists("counter")) {
    counter = 1;
}

set object counter polygon from \
         x + w * cos(theta) - h * sin(theta), y - w * sin(theta) - h * cos(theta) \
    to   x + w * cos(theta) + h * sin(theta), y - w * sin(theta) + h * cos(theta) \
    to   x - w * cos(theta) + h * sin(theta), y + w * sin(theta) + h * cos(theta) \
    to   x - w * cos(theta) - h * sin(theta), y + w * sin(theta) - h * cos(theta) \
    to   x + w * cos(theta) - h * sin(theta), y - w * sin(theta) - h * cos(theta)

set object counter fc rgb (foot eq 'left' ? 'green' : 'red') lw 1 front

counter = counter + 1
