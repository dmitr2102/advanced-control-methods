# Obstacle MPC

CasADi MPC experiment with two apex blockers on the fixed S-curve track.

The goal is unchanged: reach the final point with the required orientation and
maximum exit speed while respecting tire grip and track boundaries. The new
constraint is obstacle avoidance: each turn has one obstacle placed near the
apex and covering the inside half of the road up to the centerline.

Run interactive visualization:

```powershell
python obstacle_mpc/play_obstacles.py --horizon 15
```

Record a GIF:

```powershell
python obstacle_mpc/record_obstacle_gif.py --horizon 15
```

Current smoke-test result with `horizon=15`, `seed=0`:

```text
won=True
steps=110
exit speed=91.3 km/h
reason=goal
```
