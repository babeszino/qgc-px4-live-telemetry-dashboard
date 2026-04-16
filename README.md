# qgc-px4-live-telemetry-dashboard
A live telemetry dashboard built upon QGC/Px4 for real life drone testing

## PX4 Gazebo SITL Setup (x500)

When running a PX4 SITL, a MAVLink stream must be started manually for the dashboard to receive telemetry. Run the following command in the PX4 shell:

```
mavlink start -x -u 14562 -r 4000000 -m onboard -t <WSL ip> -o 14551
```

Also start a second stream for GCS software (e.g. QGroundControl):

```
mavlink start -x -u 14563 -r 4000000 -t <WSL ip> -o 14550
```

Replace `<WSL ip>` with the WSL network interface IP (e.g. `172.18.190.31`). The dashboard listens on `0.0.0.0:14551`.

To get your WSL IP, run the following in a WSL terminal:

```
hostname -I
```

NOTE: It's recommended to have Windows Firewall turned off (or make custom rules), as it can cause connectivity issues between Windows and WSL.