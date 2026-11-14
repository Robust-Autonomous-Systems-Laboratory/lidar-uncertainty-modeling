---
layout: splash
classes: wide
mathjax: true
title: "Time-of-Flight Lidar Operation"
permalink: /tof-lidar/
author_profile: false

---
<br>

# How does time-of-flight lidar work?

In general, time-of-flight lidar works by pulsing an infrared laser, where the light pulses reflect from interactions in the enviornment. The round trip travel time is measured from when the light is pulsed to when it is reflected back into a receiver. Since the speed of light is constant, the one-way travel time is used to solve for distance from the sensor to reflected objects via the relationship $distance = \frac{c \Delta t}{2}$, where the speed of light, $c \approx 3.0\times10^8$ [m/s].

In the sections below, we take a closer look at how each major part of a ToF lidar operates, from optical hardware to signal processing.


## Transmit and Receive Chains

In most automotive lidar, the transmit and receive chains are monostatically aligned, which means that the transmitter and receiver use the same shared optical aperature. Monostatic lidar remain a popular choice in automotive lidar due to smaller physical size and lower cost [1].

Consumer ToF lidar for automotive and robotics applications commonly feature either a mechanical spinning architecture with stacked layers of lasers, or a semi solid-state scanning architecture in a forward scanning direction.

<div style="max-width: 60%; margin: 0 auto;">
{% include figure image_path="/assets/media/lidar_architecture_graphic.png" alt="A high level diagram showing a spinning lidar and its laser channels, and a semi-solid state lidar with its scan pattern." caption="Figure 1: Typical automotive lidar architectures and their laser patterns. The left figure highlights a spinning lidar with laser layers to form a vertical FoV, representative of a Velodyne VLP-16. The right figure highlights a semi solid state lidar with a horizontal and vertical FoV, representative of a Cepton Vista P60 [2]." class="align-center" %}
</div>
 
### Laser Source (Transmitter)

Automotive lidar's lasers operate in the near-infrared (NIR) or short-wave infrared (SWIR) spectrums. Common wavelengths include 865 nm (NIR), 905 nm (NIR), and 1550 nm (SWIR). NIR wavelengths are common because cheaper silicon works as a detector material. SWIR detectors require more exotic and expensive detectors, such as indium gallium aresenide (InGaAs) detectors [1].  A given laser's power is dictated by its wavelength due to [maximum permissible exposure (MPE)](https://en.wikipedia.org/wiki/Laser_safety#Maximum_permissible_exposure) levels for eye safety. 

Water in the human eye absorbs wavelengths above ~1500 nm and below ~400 nm, which allows for greater MPE levels in these wavelengths [1]. Consequently, SWIR lidar operating near 1550 nm operate at higher powers, ranging between 3-8 orders of magnitude higher than that of visible light [1]. Greater lidar power used in SWIR lidar results in an increased maximum deteactable range in clear conditions. 

### Detector (Receiver)

On the receiver side, automotive ToF lidar leverages avalance photodiodes (APDs) which convert light into electrical current. Reflected laser energy is detected by the APDs, and stronger optical returns correlate to stronger electrical current. The optical signal is processed to isolate object detection from background noise, and is discussed more in the following sections.

Many automotive lidars use silcon-based linear mode avalance photodiodes, or LMAPDs. Linear mode indicates that the current output of the device is proprotional to the intensity of the reflected optical signal. Geiger-mode avalanche photodiodes (GMAPDs), also known as single-photon avalanche diodes (SPADs), are also used in some lidar, such as Ouster lidars, and can detect single photons and output a trigger, indicating a return event.

In Figure 2, a Velodyne VLP-16 was opened for careful inspection of the internal architecture. Note that one side of the optical aparatus contains 16 stacks of lasers, and the opposite side contains 16 stacks of linear mode avalanche photodiodes. Each individual circuit corresponds to one of the VLP-16's 16 laser channels, aligned to provide a vertical field of view of roughly equal spacing, covering $\pm 15 ^\circ$.

<div style="max-width: 60%; margin: 0 auto;">
{% include figure image_path="/assets/media/vlp16_teardown_diagram.png" alt="The optoelectronics within a Velodyne VLP-16 lidar" caption="Figure 2: A disassembled view of the internal optoelectronics of the VLP-16, highlighting separate stacks of laser sources and receivers." class="align-center" %}
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

[2] H. Holzhüter, J. Bödewadt, S. Bayesteh, A. Aschinger, and H. Blume, “Technical Concepts of automotive lidar sensors: A review,” Optical Engineering, vol. 62, no. 03, pp. 1–34, Jan. 2023. doi:10.1117/1.oe.62.3.031213 

