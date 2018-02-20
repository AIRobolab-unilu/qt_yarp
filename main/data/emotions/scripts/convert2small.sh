#!/bin/bash
# create an array with all the files
files=(orig/*.avi)

mkdir -p ./640x360_5fps
rm -f ./640x360_5fps/*.avi

# iterate through array using a counter
for ((i=0; i<${#files[@]}; i++)); do
    echo "${files[$i]}"
    filename=$(basename "${files[$i]}")
    extension="${filename##*.}"
    filename="${filename%.*}"
    ffmpeg -y -i ${files[$i]} -r 4 -s 640x360 -vcodec libx264 -b 64K ./640x360_5fps/${filename}.avi
done

