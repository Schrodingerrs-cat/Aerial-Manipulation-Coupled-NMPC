model results/am3_rl/R5_turing_s0/model_final.zip
ref lissa3d  start seed 101  (held-out, >=100)
zero action (floor): RMS error after 2 s = 1220.9 mm, terminated=True
PD + feedforward (scripted): RMS error after 2 s = 469.6 mm, terminated=False
RL policy (end-to-end PPO): RMS error after 2 s = 2.6 mm, terminated=False
ffmpeg: ffmpeg -y -loglevel error -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R5_lissa3d_2026-11-04/frames/0/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R5_lissa3d_2026-11-04/frames/1/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R5_lissa3d_2026-11-04/frames/2/%05d.png -filter_complex [0:v][1:v][2:v]hstack=inputs=3,scale=1920:-2[v] -map [v] -pix_fmt yuv420p -c:v libx264 -crf 20 /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R5_lissa3d_2026-11-04/compare.mp4
