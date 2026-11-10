---
layout: splash
classes: wide
mathjax: true
title: "Time-of-Flight Lidar Operation"
permalink: /tof-lidar/
author_profile: false

---

<br>

In general, time-of-flight lidar works by pulsing an infrared laser, where the light pulses reflect from interactions in the enviornment. The round trip travel time is measured from when the light is pulsed to when it is reflected back into a receiver. Since the speed of light is constant, the one-way travel time is used to solve for distance from the sensor to reflected objects via the relationship $distance = \frac{c \Delta t}{2}$, where the speed of light, $c \approx 3.0\times10^8$ [m/s].

In the sections below, we take a closer look at how each major part of a ToF lidar operates, from optical hardware to signal processing.


## Transmit and Receive Chains

In most automotive lidar, the transmit and receive chains are monostatically aligned, which means that the transmitter and receiver use the same shared optical aperature. Monostatic lidar remain a popular choice in automotive lidar due to smaller physical size and lower cost [1].
 
### Laser Source (Transmitter)

Automotive lidar's lasers operate in the near-infrared (NIR) spectrum, but their exact wavelength is a design choice that varies. Common wavelengths include 865 nm, 905 nm, and 1550 nm. A given laser's power is dictated by its wavelength due to [maximum permissible exposure (MPE)](https://en.wikipedia.org/wiki/Laser_safety#Maximum_permissible_exposure) levels for eye safety. 

Lasers at 1550 nm are allowed a higher MPE due to absoprtion by parts of the eye before light reaches the retina, meaning 1550 nm lidar can operate at higher power than 905 nm, and consequently have a longer detection range.

### Detector (Receiver)

<div style="max-width: 60%; margin: 0 auto;">
{% include figure image_path="/assets/media/vlp16_teardown_diagram.png" alt="The optoelectronics within a Velodyne VLP-16 lidar" caption="Figure 1: CAPTION HERE" class="align-center" %}
</div>

## Matched Filter



## Peak Detection


## Timing Circuits



## Range Calculation

$d = \frac{c \Delta t}{2}$


## 3D Point Cloud

Software ranges 

## References

[1] P. F. McManamon, Lidar Technologies and Systems. Bellingham, WA, USA: SPIE Press, 2019. 