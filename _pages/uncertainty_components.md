---
layout: single
classes: wide
title: "Uncertainty Components of Automotive Lidar"
permalink: /uncertainty-components/
author_profile: false
# gallery:
#   - url: /assets/media/nir_velodyne_closeup.png
#     image_path: /assets/media/nir_velodyne_closeup.png
#     alt: "NIR image of the Velodyne VLP-32C beam pattern on a 0.5 x 0.5 m 80% reflective, Lambertian target"
#   - url: /assets/media/nir_ouster_closeup.png
#     image_path: /assets/media/nir_ouster_closeup.png
#     alt: "NIR image of the Ouster OS-1 64-channel beam pattern on a 0.5 x 0.5 m 80% reflective, Lambertian target"
#   - url: /assets/media/nir_ruby_closeup.png
#     image_path: /assets/media/nir_ruby_closeup.png
#     alt: "NIR image of the Robosense Ruby 128-channel beam pattern on a 0.5 x 0.5 m 80% reflective, Lambertian target"
---

<!-- {% include gallery %} -->

## Lidar Range Equation

$$
P_r = P_t * \rho * cos(\Theta) * \frac{\pi D^2}{4R} * \eta_{sys} * \eta_{atm}
$$

- $P_r$: Received optical power [W]
- $P_t$: Transmitted optical power [W]
- $\rho$: Target reflectivity
- $cos(\Theta)$: Incidence angle
- $D$: Aperature diameter [m]
- $R$: Target range [m]
- $\eta_{sys}$: Quantum efficency
- $\eta_{atm}$: Atmospheric efficency









<!-- ![150 Velodyne VLP-32C scans overlaid illustrating greater range uncertainty at the left and right edges of the target]({{ '/assets/media/velodyne_25m_edge_effects.png' | relative_url }}) -->


