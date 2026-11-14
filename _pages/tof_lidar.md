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

In general, time-of-flight (ToF) lidar works by pulsing an infrared laser, and the light pulses reflect from interactions in the environment. The system measures the round-trip travel time from when the light is pulsed to when it is reflected back into a receiver. Since the speed of light is constant, the one-way travel time is used to solve for distance from the sensor to reflected objects via the relationship $distance = \frac{c \Delta t}{2}$, where the speed of light $c \approx 3.0\times10^8$ [m/s].

In the sections below, we take a closer look at how each major part of a ToF lidar operates, from optical hardware to signal processing. Note that not all automotive lidar operate exactly as described, but we present a general overview of a typical ToF lidar system.


## Transmit and Receive Chains

In most automotive lidar, the transmit and receive chains are monostatically aligned, meaning the transmitter and receiver use the same shared optical aperture. Monostatic lidar remains a popular choice in automotive lidar because of its smaller size and lower cost [1].

Consumer ToF lidar for automotive and robotics applications commonly features either a mechanical spinning architecture with stacked laser layers or a semi-solid-state scanning architecture in a forward scanning direction.

<div style="max-width: 90%; margin: 0 auto;">
{% include figure image_path="/assets/media/lidar_architecture_graphic.png" alt=" A high-level diagram showing a spinning lidar and its laser channels, and a semi-solid-state lidar with its scan pattern." caption=" Figure 1: Typical automotive lidar architectures and their laser patterns. The left figure highlights a spinning lidar with laser layers to form a vertical FoV, representative of a Velodyne VLP-16. The right figure highlights a semi solid state lidar with a horizontal and vertical FoV, representative of a Cepton Vista P60 [2]." class="align-center" %}
</div>
 
### Laser Source (Transmitter)

Automotive lidar's lasers operate in the near-infrared (NIR) or short-wave infrared (SWIR) spectrums. Common wavelengths include 865 nm (NIR), 905 nm (NIR), and 1550 nm (SWIR). NIR wavelengths are common because cheaper silicon can serve as the detector material. SWIR detectors require more exotic and expensive detectors, such as indium gallium arsenide (InGaAs) detectors [1].  A laser's power is dictated by its wavelength because of [maximum permissible exposure (MPE)](https://en.wikipedia.org/wiki/Laser_safety#Maximum_permissible_exposure) levels for eye safety. 

Water in the human eye absorbs wavelengths above ~1500 nm and below ~400 nm, allowing higher MPE levels in these wavelengths [1]. Consequently, SWIR lidar operating near 1550 nm can operate at powers 3 to 8 orders of magnitude higher than visible light [1]. Higher lidar power in the SWIR increases the maximum detectable range in clear conditions. 

### Detector (Receiver)

On the receiver side, automotive ToF lidar leverages avalanche photodiodes (APDs), which convert light into electrical current. The APDs detect reflected laser energy, and stronger optical returns produce stronger electrical current. The optical signal is processed to separate object detection from background noise, as discussed in the following sections.

Many automotive lidars use silicon-based linear mode avalanche photodiodes, or LMAPDs. Linear mode means the device's output current is proportional to the intensity of the reflected optical signal [1]. Geiger-mode avalanche photodiodes (GMAPDs), also known as single-photon avalanche diodes (SPADs), are used in some lidar, such as Ouster lidar, and can detect single photons and output a trigger, indicating a return event [1].

A transimpedance amplifier (TIA) converts the APD output current into a proportional voltage signal. For LMAPDs, a high-frequency analog-to-digital converter (ADC) digitizes the voltage waveform. For GMAPDs or SPADs, the detected triggered events are used in a time-to-digital converter (TDC) to digitize the time difference between a transmitted pulse and the detected return, avoiding ADCs and skipping the matched-filtering process.

In Figure 2, a Velodyne VLP-16 was opened for careful inspection of the internal architecture. One side of the optical apparatus contains 16 stacks of lasers, and the opposite side contains 16 stacks of linear-mode avalanche photodiodes. Each individual circuit corresponds to one of the VLP-16's 16 laser channels, aligned to provide a vertical field of view of roughly equal spacing, covering $\pm 15 ^\circ$.

<div style="max-width: 90%; margin: 0 auto;">
{% include figure image_path="/assets/media/vlp16_teardown_diagram.png" alt=" The optoelectronics within a Velodyne VLP-16 lidar" caption="Figure 2: A disassembled view of the internal optoelectronics of the VLP-16, highlighting separate stacks of laser sources and receivers." class="align-center" %}
</div>

## Matched Filter + Thresholding

For automotive lidar using linear-mode avalanche photodiodes, the matched filter aims to detect the known laser pulse in the noisy, reflected signal from the detector. By maximizing the signal-to-noise ratio (SNR), the matched filter identifies true reflections from objects in the environment if their return power exceeds a tuned threshold.

The matched filter convolves a conjugated, time-reversed copy of the laser pulse with the received, noisy detected laser pulse. Figure 3 below illustrates the matched filter's behavior [3]. Because the pulse is Gaussian, it is not obvious in the example that the waveform is time-reversed.

<div style="max-width: 100%; margin: 0 auto;">
{% include figure image_path="/assets/media/matched_filter_diagram.png" alt="Three time series graphs, the top showing a Gaussian pulse, the middle showing the returned Gaussian pulse with white noise, and the bottom showing the cross correlation of the noisy pulse with the original pulse to highlight detected range." caption=" Figure 3: Example of how a matched filter operates - In the upper subplot, a Gaussian representing a laser pulse is shown. The middle subplot shows the detected signal with additive white Gaussian noise against the original signal. In the lower subplot, the resulting cross-correlation of the original signal with the noisy signal is shown, illustrating how a lidar system uses the peak of the resulting waveform to identify the range of the detected object [3]." class="align-center" %}
</div>


Matched filtering produces a clear time-domain location of detected objects, where the round-trip travel time is determined by the *peak* of the laser pulse exceeding the tuned detection threshold. Tuning the detection threshold is important to avoid false detections while still detecting weak, true detections, such as very far targets or targets with low reflectivity.

## Range Data

The lidar's onboard processors use the round-trip travel time of detected objects through the time-of-flight formula, where one-way distance is calculated from the one-way travel time (round-trip travel time divided by 2) and the speed of signal propagation through a given medium, in our case, the speed of light.

$$d = \frac{c \Delta t}{2}$$

The result is a one-way detection distance reported for a given object. In a simple laser rangefinder, the job is complete. But for lidar, the distance must be converted into a usable format that represents where, relative to the sensor, that range detection occurred.

By tracking the horizontal angle ($\theta$) and the vertical angle ($\phi$) each laser pulse was fired from, the 3D position of each detected point can be represented via spherical-to-Cartesian coordinate conversion, resulting in a list of $(X,Y,Z,I)$ coordinates, where $X,Y,Z$ represent the coordinates in 3D space, and $I$ is the point's intensity. These coordinates form the basis of the point cloud we expect from commercial lidar systems.



## References

[1] P. F. McManamon, Lidar Technologies and Systems. Bellingham, WA, USA: SPIE Press, 2019. 

[2] H. Holzhüter, J. Bödewadt, S. Bayesteh, A. Aschinger, and H. Blume, “Technical Concepts of automotive lidar sensors: A review,” Optical Engineering, vol. 62, no. 03, pp. 1–34, Jan. 2023. doi:10.1117/1.oe.62.3.031213 

[3] R. D. Richmond and S. C. Cain, Direct-Detection Ladar Systems. Bellingham, Wash: SPIE Press, 2010. 

