#!/bin/bash
# Slide clips for the 2026-11-23 capstone deck: each panel = 3D view of a recorded run with the renderer's header and phase bar cropped off and one plain label.
# panel IN LABEL SPEED OUT [HEADER_CROP_SOURCE]: IN is a 1280-wide writing video (3D view on top) or a pre-cut 1280x720 panel
F=/usr/share/fonts/truetype/liberation2/LiberationSerif-Bold.ttf
ENC="-c:v libx264 -crf 25 -preset slow -pix_fmt yuv420p -an -movflags +faststart"
lab() { echo "crop=1280:558:0:130,pad=1280:618:0:60:0x2b2b2b,drawtext=fontfile=$F:text='$1':fontcolor=white:fontsize=40:x=16:y=10"; }
spd() { [ "$1" = 2 ] && echo ",setpts=PTS/2,drawtext=fontfile=$F:text='2x speed':fontcolor=white:fontsize=34:box=1:boxcolor=black@0.6:boxborderw=8:x=w-tw-20:y=h-th-20" || echo ""; }
single() { ffmpeg -v error -y -i "$1" -vf "crop=1280:720:0:0,$(lab "$2")$(spd $3)" -r 30 $ENC "$4"; }
pair() { D=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$2")
  ffmpeg -v error -y -i "$1" -i "$2" -filter_complex "[0]crop=1280:720:0:0,$(lab "$3"),tpad=stop_mode=clone:stop_duration=90[a];[1]crop=1280:720:0:0,$(lab "$4"),tpad=stop_mode=clone:stop_duration=90[b];[a][b]hstack,trim=duration=$D,scale=1920:-2$(spd $5)" -r 30 $ENC "$6"; }
M=../../Media_results
single $M/2026-11-21_1646_Write_HELLO_WPI_nominal_lab_N_NDF_R21_L2W/N.mp4 "NMPC, no disturbance" 2 v_write.mp4
single $M/2026-11-22_0859_Write_HELLO_WPI_payload100g_lab_N_L2W/N.mp4 "NMPC, unknown 100 g load" 1 v_load.mp4
pair $M/2026-11-22_0859_Write_HELLO_WPI_motor_slow_lab_NDF_R21/NDF.mp4 $M/2026-11-22_0859_Write_HELLO_WPI_motor_slow_lab_NDF_R21/R21.mp4 "NMPC + disturbance estimate" "Learned controller" 2 v_slow.mp4
S=$M/2026-11-21_1646_Write_HELLO_WPI_wall_soft_lab_NDF_R21_L2W; pair $S/NDF.mp4 $S/L2W.mp4 "NMPC + disturbance estimate" "NMPC + learned correction" 2 v_soft.mp4
B=$M/AM3_BvsN_arm_oscillation_B_loses_control_2026-10-30/compare_free_osc0.06_T0.25_overview_wall.mp4
ffmpeg -v error -y -i $B -filter_complex "[0]crop=1280:720:0:0,$(lab 'Separate drone and arm controllers')[a];[0]crop=1280:720:1280:0,$(lab 'One model of drone + arm')[b];[a][b]hstack,scale=1920:-2" $ENC v_arm.mp4
ffmpeg -v error -y -i $M/Payload_AB_2m_UvsSvsE_attmodel_2026-11-10/U_best.mp4 -i $M/Payload_AB_2m_UvsSvsE_attmodel_2026-11-10/E.mp4 -filter_complex "[0]crop=960:470:0:70,pad=960:520:0:50:0x2b2b2b,drawtext=fontfile=$F:text='Ignores the load':fontcolor=white:fontsize=34:x=14:y=8[a];[1]crop=960:470:0:70,pad=960:520:0:50:0x2b2b2b,drawtext=fontfile=$F:text='Plans with the swing':fontcolor=white:fontsize=34:x=14:y=8[b];[a][b]hstack" $ENC v_swing.mp4
for v in write:24 load:12 slow:12 soft:26 arm:4.2 swing:1.6; do n=${v%%:*}; ffmpeg -v error -y -ss ${v#*:} -i v_$n.mp4 -frames:v 1 p_$n.png; done
