model results/am3_rl/R1_cont_B_fine/model_final.zip
ref hover  start seed 100  (held-out, >=100)
zero action (floor): RMS error after 2 s = 1068.8 mm, terminated=True
PD (scripted): RMS error after 2 s = 3.6 mm, terminated=False
RL policy (end-to-end PPO): RMS error after 2 s = 5.8 mm, terminated=False
ffmpeg: ffmpeg -y -loglevel error -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R1_hover_2026-10-31/frames/0/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R1_hover_2026-10-31/frames/1/%05d.png -framerate 25 -i /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R1_hover_2026-10-31/frames/2/%05d.png -filter_complex [0:v][1:v][2:v]hstack=inputs=3[v] -map [v] -pix_fmt yuv420p -c:v libx264 -crf 20 /home/pinaka/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/../../Media_results/RL_R1_hover_2026-10-31/compare.mp4
