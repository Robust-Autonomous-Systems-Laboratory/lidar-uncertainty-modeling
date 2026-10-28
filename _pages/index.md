---
layout: splash
classes: wide
mathjax: true
title: "Automotive Lidar Uncertainty Modeling"
permalink: /
author_profile: false
header:
  overlay_color: "#000"
  overlay_filter: "0.5"
  overlay_image: /assets/media/experiment_with_fall_colors.jpg
  # actions:
  #   - label: "Download"
  #     url: "https://github.com/mmistakes/minimal-mistakes/"
  caption: ""
excerpt: "A guide to uncertainty sources in time of flight, automotive lidar sensors in varying conditions"

---

## Introduction

Automotive time-of-flight (ToF) lidar sensors are critical for autonomous vehicle and mobile robot perception systems. Lidar measures the range to objects in the enviornment, but a sensor is only as good as it is certain. Manufacturers typically report measurement uncertainty at approximately 2 - 5 cm. However, lidar is affected by adverse weather, especially wet particulate such as fog and snow, which typically manifests as clutter noise, or false-positive returns caused by the fog or snow particles. We aim to evaluate the lidar range uncertainty due to adverse weather, and report on the overall effect weather may have on the precision of the system.


[How do ToF lidars work?]({{ '/tof-lidar/' | relative_url }}){: .btn .btn--primary}
[Sources of lidar range uncertainty]({{ '/uncertainty-components/' | relative_url }}){: .btn .btn--info}


## Lidar Range Experiment

We design an experiment with the [DIN SAE 91471](https://www.dinmedia.de/en/technical-rule/din-sae-spec-91471/366011551) cross-domain test as a baseline, to measure lidar range uncertainty at a series of 10% and 80% Lambertian targets at 15 m, 30 m, 45 m, 60 m, 75 m, and 100 m distances. Our experiment is designed for practioners to evaulate uncertainty of a lidar system, and is intended for ad-hoc deployment in a temporary test site, such as a parking lot. 

To evaluate a lidar system in varying conditions, we first measure the uncertainty of the test itself through repeated trials over multiple, clear weather days. Range uncertainty from trials in adverse conditions are evaluated against the baseline uncertainty through a one-way analysis of variance (ANOVA) test to determine the uncertainty contribution of specific adverse weather, such as snow, sleet, rain, direct sunlight, fog, and wildfire smoke. 


[Experiment Details]({{ '/experiment/' | relative_url }}){: .btn .btn--primary}
[Processing Pipeline]({{ '/processing/' | relative_url }}){: .btn .btn--info}
[Data + Results]({{ '/data/' | relative_url }}){: .btn .btn--success}