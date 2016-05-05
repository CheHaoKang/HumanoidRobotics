d:=$(shell seq 0 15 165)
distances:=$(patsubst %,distance_%.pgm,$(d))
composites:=$(patsubst %,composite_%.png,$(d))
FOOT_LENGTH:=0.16
FOOT_WIDTH:=0.09
RESOLUTION:=0.01
SCALE:=5

KERNEL="Rectangle:$(shell echo '($(FOOT_LENGTH) / $(RESOLUTION) + 1) / 2' | bc)x$(shell echo '($(FOOT_WIDTH) / $(RESOLUTION) + 1) / 2' | bc)"

all: $(distances) $(composites)

overlay.png: map.pbm Makefile
	convert map.pbm -fill red -opaque black -transparent white $@

distance_%.pgm: map.pbm overlay.png Makefile
	convert -rotate $(patsubst distance_%.pgm,%,$@) +repage map.pbm -threshold 50% -morphology Erode $(KERNEL) -morphology Distance Euclidean:1,$$(($(SCALE)*100)) -rotate -$(patsubst distance_%.pgm,%,$@) +repage -gravity center -crop 400x400+0+0 +repage -colorspace gray -depth 8 $@

composite_%.png: distance_%.pgm overlay.png
	convert $< overlay.png -composite $@
