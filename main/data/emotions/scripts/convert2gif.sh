#!/bin/bash
# create an array with all the filer/dir inside ~/myDir
files=(./*.avi)

mkdir -p ./gif
rm -f ./gif/*

# iterate through array using a counter
for ((i=0; i<${#files[@]}; i++)); do
    #echo "${files[$i]}"
    filename=$(basename "${files[$i]}")
    extension="${filename##*.}"
    filename="${filename%.*}"
    #ffmpeg -y -i ${files[$i]} -r 2 -s 128x72 -vcodec libx264 -b 64K /tmp/tmp.avi; ffmpeg -i /tmp/tmp.avi -pix_fmt rgb24 "./gif/${filename}.gif"
    ffmpeg -i ${files[$i]} -r 5 -s 128x72 -f image2pipe -vcodec ppm - | convert -delay 5 - gif:- | convert -layers Optimize - "./gif/${filename}.gif"
done

