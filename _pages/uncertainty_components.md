---
layout: splash
classes: wide
mathjax: true
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


<!-- ADD INTRO SECTION HERE -->


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



{% include figure image_path="/assets/media/basic_lidar_graph.png" alt="diagram of a lidar transmitter and receiver with laser power penetrating and reflecting from snowflakes and rain" caption="Figure 1: Illustrated lidar transmit and receive chain interacting with atmospheric particles. The transmitted energy penetrates the atmospheric particles to reflect off the target into the receive chain. Mie scattering scatters energy in all directions, including into the receive chain [1]." %}







<!-- ![150 Velodyne VLP-32C scans overlaid illustrating greater range uncertainty at the left and right edges of the target]({{ '/assets/media/velodyne_25m_edge_effects.png' | relative_url }}) -->



## References 

[1] J. Zhan, Y. Duan, J. Ding, X. Hu, X. Huang and J. Ma, "Towards Visibility Estimation and Noise-Distribution-Based Defogging for LiDAR in Autonomous Driving," 2024 IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 18443-18449, doi: 10.1109/ICRA57147.2024.10610699.

