model results/am3_rl/R4_warm_s1/model_1000k.zip
ref lissa3d  start seed 101  (held-out, >=100)
zero action (floor): RMS error after 2 s = 1174.9 mm, terminated=True
PD + feedforward (scripted): RMS error after 2 s = 19.6 mm, terminated=False
RL policy (end-to-end PPO): RMS error after 2 s = 2.3 mm, terminated=False
ffmpeg: ffmpeg -y -loglevel error -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R4_lissa3d_2026-10-31/frames/0/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R4_lissa3d_2026-10-31/frames/1/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R4_lissa3d_2026-10-31/frames/2/%05d.png -filter_complex [0:v][1:v][2:v]hstack=inputs=3[v] -map [v] -pix_fmt yuv420p -c:v libx264 -crf 20 /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R4_lissa3d_2026-10-31/compare.mp4
