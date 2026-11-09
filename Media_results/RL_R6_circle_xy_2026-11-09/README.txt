model results/am3_rl/R6_turing_s1/model_final.zip
ref circle_xy  start seed 1000  (held-out, >=100)
zero action (floor): RMS error after 2 s = 1297.2 mm, terminated=True
PD + feedforward (scripted): RMS error after 2 s = 17.3 mm, terminated=False
RL policy (end-to-end PPO): RMS error after 2 s = 2.7 mm, terminated=False
ffmpeg: ffmpeg -y -loglevel error -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R6_circle_xy_2026-11-09/frames/0/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R6_circle_xy_2026-11-09/frames/1/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R6_circle_xy_2026-11-09/frames/2/%05d.png -filter_complex [0:v][1:v][2:v]hstack=inputs=3,scale=1920:-2[v] -map [v] -pix_fmt yuv420p -c:v libx264 -crf 20 /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R6_circle_xy_2026-11-09/compare.mp4
