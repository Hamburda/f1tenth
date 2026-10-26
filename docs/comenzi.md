# Comenzi de rulare (pure pursuit)

## Terminal 1: simulatorul
```bash
cd ~/f1tenth && source .venv/bin/activate && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## Terminal 2: nodul pure pursuit

### 1. Mediul (la fiecare terminal nou)
Intră în venv și încarcă ROS + workspace-ul:
```bash
cd ~/f1tenth && source .venv/bin/activate && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

### 2. Build (doar după ce modifici codul, launch file-urile sau waypoint-urile)
```bash
colcon build --packages-select pure_pursuit && source install/setup.bash
```
După ce schimbi `sim.yaml` (harta sau startul), fă build complet, apoi repornește simulatorul:
```bash
colcon build && source install/setup.bash
```

### 3. Pornire, câte un launch file pe hartă
```bash
ros2 launch pure_pursuit spielberg_launch.py      # Spielberg, raceline safe 0.6, ~38.8 s
ros2 launch pure_pursuit levine_launch.py         # levine_blocked, 4 m/s constant
ros2 launch pure_pursuit levine_obs_launch.py     # levine_obs, 2 m/s constant
```

## Harta și startul, în `src/f1tenth_gym_ros/config/sim.yaml`
Launch file-ul trebuie să se potrivească cu harta pornită în simulator. Valorile cu punct zecimal (`-12.0`, nu `-12`).

| launch | map_path | sx | sy | stheta |
|---|---|---|---|---|
| spielberg_launch.py | `'maps/Spielberg'` | 14.76 | 3.35 | -2.851 |
| levine_launch.py | `'maps/levine_blocked'` | -12.0 | 0.0 | 0.0 |
| levine_obs_launch.py | `'maps/levine_obs'` | 9.96 | 2.8 | 1.5708 |

## Reglaj fără launch file
Aceiași parametri, dați direct din comandă (fiecare `-p nume:=valoare`, cu punct zecimal):
```bash
ros2 run pure_pursuit pure_pursuit_node --ros-args -p csv_name:=Spielberg_raceline_safe_0.6.csv -p vmax:=12.0 -p vmin:=4.0 -p a_lat:=7.5 -p a_brake:=3.0 -p a_acc:=4.0 -p k:=0.3 -p L0:=-0.6 -p L_min:=1.5 -p L_max:=4.0
```
