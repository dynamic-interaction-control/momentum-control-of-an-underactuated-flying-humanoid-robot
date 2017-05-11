#!/bin/sh

INPUT=$1

# default settings, modify if you want.

START_AT_SECOND=0; # in seconds, if you want to skip the first 30 seconds put 30 here

LENGTH_OF_GIF_VIDEO=9999999; # in seconds, how long the gif animation should be

echo "Generate a palette:"
ffmpeg -y -ss $START_AT_SECOND -t $LENGTH_OF_GIF_VIDEO -i $INPUT -vf fps=10,scale=320:-1:flags=lanczos,palettegen palette.png

echo "Output the GIF using the palette:"
ffmpeg -ss $START_AT_SECOND -t $LENGTH_OF_GIF_VIDEO -i $INPUT -i palette.png -filter_complex "fps=10,scale=320:-1:flags=lanczos[x];[x][1:v]paletteuse" $INPUT.gif
