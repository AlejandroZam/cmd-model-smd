# SpringMassDamper — Model Description

## Dynamics

Second-order linear system driven by a stochastic force disturbance:

```
m·x''(t) + c·x'(t) + k·x(t) = F_noise(t)
```

Two integrators are registered with the OSK kernel:
- `pos` (position, m) — driven by `vel`
- `vel` (velocity, m/s) — driven by `-(c/m)·vel - (k/m)·pos + F_noise/m`

## Noise Channels

| Key               | Applied to            | Typical use          |
|-------------------|-----------------------|----------------------|
| `noise.force`     | Velocity derivative   | External disturbance |
| `noise.measurement` | Position output only | Sensor noise         |

Force noise is sampled once per frame in `eventUpdate()` and held constant across RK4 sub-steps. Measurement noise is applied in `report()` and does not feed back into the integrator.

## Outputs

| Signal      | Units | Description                        |
|-------------|-------|------------------------------------|
| `t`         | s     | Simulation time                    |
| `pos`       | m     | True position                      |
| `vel`       | m/s   | True velocity                      |
| `pos_noisy` | m     | Position + measurement noise       |

## Configuration Reference

```yaml
model:
  mass:             1.0    # kg
  damping:          0.2    # N·s/m
  stiffness:        4.0    # N/m
  initial_position: 1.0    # m
  initial_velocity: 0.0    # m/s
  report_rate_hz:   10.0

noise:
  force:
    distribution: gaussian
    mean: 0.0
    stddev: 0.01           # N
  measurement:
    distribution: gaussian
    mean: 0.0
    stddev: 0.001          # m
```
