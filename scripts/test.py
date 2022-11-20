import GPPlanning

planner = GPPlanning.pyPlanning2D(True, 0.2, 0.001, 0.01, 0.2, 5)
planner.buildMap(1, (0, 0), 50, 50)

delta_t = 0.4
total_time_step = 5
start_pose = (5, 5, 0)
start_vel = (0.0, 0.0, 0.0)

end_pose = (45, 45, 0)
end_vel = (0.0, 0.0, 0.0)

avg_vel = (end_pose[0] - start_pose[0]) / delta_t, \
          (end_pose[1] - start_pose[1]) / delta_t, \
          (end_pose[2] - start_pose[2] / delta_t)
poses = []
vels = []
for i in range(0, 5):
    pose = (start_pose[0] * (total_time_step - i) / total_time_step + end_pose[0] * i / total_time_step,
            start_pose[1] * (total_time_step - i) / total_time_step + end_pose[1] * i / total_time_step,
            start_pose[2] * (total_time_step - i) / total_time_step + end_pose[2] * i / total_time_step)
    poses.append(pose)
    vels.append(avg_vel)

aa = planner.optimize(poses, vels, delta_t)
print(aa)