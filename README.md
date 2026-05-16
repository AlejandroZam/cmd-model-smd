# cmd-model-smd

Second-order spring-mass-damper model for the CMD simulation framework.

**Equation of motion:** `m·x'' + c·x' + k·x = F_noise`

## Usage

Add to a CMD scenario YAML:
```yaml
- name: my_smd
  type: SpringMassDamper
  config: smd_params.yaml
  enabled: true
  order: 0
```

Config parameters are documented in `Config/default_params.yaml`.

## Layout

```
include/   spring_mass_damper.h
src/       spring_mass_damper.cpp
Config/    default_params.yaml
doc/       model_description.md
```

## Dependencies

Requires the `osk` package from [cmd-simulations](https://github.com/AlejandroZam/cmd-simulations).
