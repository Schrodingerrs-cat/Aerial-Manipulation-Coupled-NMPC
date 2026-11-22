#!/bin/bash
# pair.sh LEFT.mp4 RIGHT.mp4 "left label" "right label" OUT.mp4 : top (3D) view of two writing videos side by side, 2x speed, clean titles
F=/usr/share/fonts/truetype/liberation2/LiberationSerif-Bold.ttf
ENC="-c:v libx264 -crf 26 -preset slow -pix_fmt yuv420p -an -movflags +faststart"
T() { echo "drawbox=x=0:y=0:w=iw:h=34:color=0x2b2b2b:t=fill,drawtext=fontfile=$F:text='$1':fontcolor=white:fontsize=30:x=10:y=3"; }
ffmpeg -v error -y -i "$1" -i "$2" -filter_complex "[0]crop=1280:720:0:0,scale=960:-2,$(T "$3"),tpad=stop_mode=clone:stop_duration=60[a];[1]crop=1280:720:0:0,scale=960:-2,$(T "$4"),tpad=stop_mode=clone:stop_duration=60[b];[a][b]hstack=shortest=0,trim=duration=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$2"),setpts=PTS/2,drawtext=fontfile=$F:text='2x speed':fontcolor=white:fontsize=30:box=1:boxcolor=black@0.6:boxborderw=8:x=w-tw-20:y=h-th-20" -r 30 $ENC "$5"
ffmpeg -v error -y -sseof -1 -i "$5" -frames:v 1 "${5%.mp4}_poster.png"
