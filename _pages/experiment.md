---
layout: splash
classes: wide
mathjax: true
title: "Experiment Configuration"
permalink: /experiment/
author_profile: false
toc: false
toc_label: "Experiment Components"
toc_icon: "list"
toc_sticky: true
---
<br>

<!-- INTRO SECTION HERE -->
Our lidar range experiment uses a lot of custom-designed, low-cost equipment. Here, we describe each component of the experiment, including assembly and setup instructions for practitioners to conduct their own experiments to benchmark lidar systems. We also describe our test field layout, ground-truth measurement systems and their respective uncertainties, and the lidars under test.


## Lidar Targets

In this experiment, we use rigid, planar targets to evaluate range uncertainty. Planar targets provide a large area that result in multiple point returns at a specified range. Furthermore, we include multiple target reflectivities to evaluate the range performance of two representative reflectivities.

### NIST Traceable Lambertian Targets

To establish a baseline performance level, we use multiple Lambertian targets placed across the test field. Lambertian surfaces scatter light uniformly in all directions, unlike specular reflectance, which reflects light in a single direction. This makes Lambertian targets ideal for lidar detection at any angle.

Specifically, we use twelve [Labsphere Permaflect® Targets](https://www.labsphere.com/product/permaflect-targets-rigid-lidar-commercial-and-custom/) at 10% and 80% reflectivities. Ten targets are 0.5 m x 0.5 m, and two are 1 m x 1 m for longer distances. Half of the targets of each size are 10% reflective, and the other half are 80% reflective, allowing us to evaluate range uncertainty at two representative reflectivities.

<div style="max-width: 60%; margin: 0 auto;">
{% include figure image_path="/assets/media/1m_targets.jpg" alt=" Two lidar targets are positioned in an empty parking lot" caption=" Figure 1: Two 1m Labsphere targets at 10% and 80% spectral reflectance" class="align-center" %}
</div>

The Labsphere Permaflect® targets used in our experiment are calibrated at a single point on the target at wavelengths from 350 to 2400 nm in 10 nm increments, with a reported spectral reflectance of $\pm 2$ % of the advertised reflectance at 905 nm, a common wavelength used by automotive lidar sensors. 

Our targets do not include mounting hardware; therefore, we designed a series of 3D-printed brackets to mount the targets to the post. For more information, please refer to the Target Stands section.

### Low-Cost Painted Analogues

We recognize that the cost of NIST-traceable targets is unattainable for many testing groups. Multiple research papers describe homemade, painted lidar targets that approximate the desired spectral reflectance using various paint types [1, 2].

We aim to use this experiment to evaluate the range uncertainty of custom, painted target analogues relative to Labsphere targets and to report multiple paints that represent varying levels of spectral reflectance.

As we characterize the painted analogues, information on each paint, including spectral reflectance and uncertainty analysis, will be provided at the link below.

<!-- not functional until page is created, need data -->
[Painted Analogue Reflectivity Data]({{ '/painted-analogue/' | relative_url }}){: .btn .btn--primary}  

## Target Stands

Custom-built target stands were designed to mount each lidar target and remain easy to assemble and transport at the test site. We offer two target stand styles, one for 0.5 x 0.5 m targets and another for 1 m x 1 m targets. Each target is constructed from materials purchased at a home improvement store and designed to be easily built by other practitioners. We provide assembly instructions and a bill of materials for each target stand at the links below.

Because our lidar targets do not include mounting hardware, we designed 3D-printed brackets that attach to each stand's signpost to hold the targets without requiring any modifications to the target. The .stl files for each printed bracket are also provided at the links below.

<div style="max-width: 100%; margin: 0 auto;">
{% include figure image_path="/assets/media/fully_assembled_0.5m_target.png" alt="A lidar target mounted to a signpost, and held upright by a wooden base" caption="Figure 2: Fully assembled rendering of the 0.5 m lidar target, featuring a 3 ft X-shaped base and a single, 6 ft tall signpost" %}
</div>


<!-- Target Stand Instructions -->
<div style="display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 1.5rem;">
  <a href="{{ '/assets/files/0.5m_target_stand_instructions.pdf' | relative_url }}" class="btn btn--primary" download>
    <i class="fas fa-download"></i> 0.5 m Target Stand Instructions
  </a>
  <a href="{{ '/assets/files/1m_target_stand_instructions.pdf' | relative_url }}" class="btn btn--primary" download>
    <i class="fas fa-download"></i> 1 m Target Stand Instructions
  </a>
</div>

<!-- Bracket .stl Files -->
<div style="display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 1.5rem;">
  <a href="{{ '/assets/files/corner_target_bracket.stl' | relative_url }}" class="btn btn--info" download>
    <i class="fas fa-download"></i> Corner Bracket .stl
  </a>
  <a href="{{ '/assets/files/middle_target_bracket.stl' | relative_url }}" class="btn btn--info" download>
    <i class="fas fa-download"></i> Middle Bracket .stl
  </a>
</div>


## Test Site

Our experiment is designed to evaluate range uncertainty at discrete distances of 15 m, 30 m, 45 m, 60 m, 75 m, and 100 m from the lidar sensor under test, so range uncertainty as a function of distance can be evaluated simultaneously. This feature is required for consistent winter weather evaluation, since snowfall can change quickly and simultaneous testing ensures that data is captured under the same conditions.

Therefore, the experiment requires a flat or near-flat area approximately 100 meters long and 30 meters wide (328 ft x 98 ft). Parking lots and sports fields are ideal for this requirement due to their large, open, flat spaces. We chose to set up and characterize our experiment in a large auxiliary parking lot (Lot 24) on Michigan Technological University's campus because it has low usage by others on campus and is regularly snowplowed in winter. This allows us easy access and setup of the targets in winter, instead of having to navigate an unplowed sports field.

A permanent test site installation, where target stands are placed in the ground, may be more suitable depending on the climate and testing goals. We opt against this option due to very limited space on our University's campus that we were permitted to use as a permanent, year-round installation and the lack of access in winter (i.e., not snowplowed). We find that our portable target experiment is more suitable for practical deployments of this experiment, as it can be set up and taken down in a simple parking lot.

<div style="max-width: 100%; margin: 0 auto;">
{% include figure image_path="/assets/media/experiment_with_fall_colors_cropped.jpg" alt="A view down-range of the experiment test site, with all 12 lidar targets in frame" caption="Figure 3: A view of the experiment site in Lot 24, with 12 targets placed at discrete ranges from the lidar under test." class="align-center" %}
</div>

## Target Arrangement

We arrange the twelve targets in an "inverted-V" shape, with two lines of targets on the left and right sides of the lidar, each line converging toward the vanishing point, with the 100 m targets directly in line with the lidar.

This arrangement provides a clear line of sight between the lidar and each target. Targets are gently turned to face the lidar so that the center of each target is at the specified range increment.

<div style="max-width: 100%; margin: 0 auto;">
{% include figure image_path="/assets/media/target_layout_plot.png" alt="A top-down plotted figure that illustrates the inverted-V shape of the targets placed in the test range" caption="Figure 4: Target placement in the 12-target lidar range experiment. 0.5 m targets are placed at 15, 30, 45, 60, and 75 m increments, and 1 m targets are placed at 100 m. Targets alternate reflectivity on the left and right-hand sides. The lidar under test is identified in red on the opening of the V-shape." class="align-center" %}
</div>


## Target Placement

Placing each target in the test field is a delicate procedure. It is imperative that the distance from the tripod center to the center of the target face match the specified range exactly for accurate and repeatable range evaluation. Previous outdoor field experiments used survey-grade lidar as a ground-truth reference, but this method is prohibitively expensive for practitioners to repeat the experiment [3].

Our goal is to use a low-cost tool with the lowest uncertainty for point-to-point measurements. We find the best tool for this is a commercial laser rangefinder with $\pm$ 1.0 mm accuracy. We also evaluated a GPS setup and a tape measure, and report on each tool's repeatability below:

### Commercial Laser Rangefinder

A laser rangefinder uses the same ToF principle as lidar but maintains accuracy levels that are centimeters lower than those of automotive lidar. Maximum range, accuracy, and alignment methods are important factors to consider when evaluating laser rangefinders. 

We chose the [Leica Disto X6](https://shop.leica-geosystems.com/measurement-tools/disto/leica-disto-x6/buy?_gl=1*nn36gs*_up*MQ..*_gs*MQ..&gclid=CjwKCAiAqabXBhBUEiwAw_SQpQfUo7lCyk8wIj1EqdgMqMVa1FgZ21mtMt1KVexw7Hmi3a8YtpxloxoC0aIQAvD_BwE&gbraid=0AAAAADnuiFjR9kesCWe0HaT6D-8CcITRZ) laser rangefinder, which features a maximum range of 250 m, $\pm$ 1.0 mm accuracy, and 0.1 mm resolution. It uses a laser positioning dot alongside a camera and touchscreen with a crosshair to help align the tool at the measurement location, allowing easy alignment with targets at 100 m. We use this tool for all current and future lidar experiments.

### GPS with Real-time Kinematic Positioning (RTK)

An alternative measurement system was also considered that does not rely on an optical component, using a Global Positioning System (GPS) sensor with [real-time kinematic positioning (RTK)](https://en.wikipedia.org/wiki/Real-time_kinematic_positioning). RTK allows GPS sensors to maintain centimeter-level accuracy (1-3 cm) via an internet connection to a Continuously Operating Reference Station (CORS). Accuracy improves the closer the GPS receiver is to the CORS tower, and our parking lot test site at Michigan Technological University is approximately 400 m from a [Michigan Department of Transportation (MDOT) CORS tower](https://mdotcors.michigan.gov/), allowing our GPS to maintain 0.5-1.0 cm accuracy at the test location.

We use an [Emlid Reach RS](https://emlid.com/docs/?_gl=1*1ffx8ib*_gcl_au*MTI1MzIwMDkxNS4xNzkzNzMxMDc2*_ga*MTM2MDY1ODc0OC4xNzkzNzMxMDc2*_ga_2L30ZMM7Q4*czE3OTM3MzEwNzUkbzEkZzAkdDE3OTM3MzEwNzckajU4JGwwJGgw) GPS designed for portable land surveying with RTK capability, and we designed a 3D-printed support mount that allowed it to sit on top of the targets during placement. A small program was developed to record the coordinates (in meters) from the GPS. It first set the location of the tripod at the base of the test field, then reported the range between the tripod and the target center live to aid in placing the target, and was repeated for the other targets.

### Tape Measure

The simplest measurement device for target placement is a tape measure reel. However, point-to-point measurement is extremely difficult with a tape measure because the tape sags under its own weight, resulting in measurements that exceed the target distance.

To avoid this issue, we placed the tape measure on the test site ground and aligned it with the center of the tripod and the front of the target face, using an indirect point-to-point measurement in an earlier iteration of this experiment [4]. Uneven ground surfaces and the lack of a direct point-to-point measurement contributed to repeated uncertain measurements, indicating that the tape measure is a very limited target alignment tool in this experiment. Furthermore, using the tape measure in adverse weather (rain, snow, etc.) makes it very difficult for the human operator to read the markings and to avoid inadvertent stretching of the tape measure when it is wet.

## Lidar Under Test

We evaluate four primary automotive lidars in our experiment, representing varying lidar wavelengths and design parameters. The experiment includes the Velodyne VLP-32C, Ouster OS-1, Robosense Ruby, and Luminar Iris, with the following advertised specifications:

<div style="max-width: 100%; margin: 0 auto;">
| Lidar | Wavelength | Max Range | Precision | Accuracy | Channels |
| ----- | ---------- | --------- | --------- | -------- | -------- |
| Velodyne VLP-32C | 905 nm | 200 m | - | $\pm$ 3 cm | 32 |
| Ouster OS-1 | 865 nm | 200 m | 2 cm | $\pm$ 3 cm | 64 |
| Robosense Ruby | 905 nm | 200 m | - | $\pm$ 5 cm | 128 |
| Luminar Iris | 1550 nm | > 275 m | - | $\pm$ 2 cm | - |
</div>


<div style="max-width: 70%; margin: 0 auto;">
{% include figure image_path="/assets/media/lidars_under_test.png" alt="Four automotive lidar sensors used in this test are placed next to each other" caption="Figure 5: Primary lidars evaluated in this experiment, from left to right: Velodyne VLP-32C, Ouster OS-1, Robosense Ruby, and the Luminar Iris" class="align-center" %}
</div>

Data was recorded for each lidar using the following modes:

<u>Velodyne VLP-32C</u> - Strongest Return and Last Return
<u>Ouster OS-1</u> - 1024x10, 1024x20, and 2048x10 resolutions
<u>Robosense Ruby</u> - Strongest Return
<u>Luminar Iris</u> - Default FoV, no programmed foveation

## Computer Setup

A testing campaign of this nature requires a computer with sufficient storage and I/O connections to connect to and log all lidar data while remaining weather-resistant for winter testing. As part of our lab's [WADS dataset](https://digitalcommons.mtu.edu/wads/) collection, we've designed a custom Mini-ITX server with 6 2-TB SSDs configured in RAID and mounted it within a weatherproof Pelican case [5]. This server also serves as the primary data collection computer for our lidar target experiment, featuring 8 Ethernet ports for sensor connections and remaining weather-tight for adverse winter testing.

<div style="max-width: 60%; margin: 0 auto;">
{% include figure image_path="/assets/media/adr.jpg" alt="exploded view of a custom server within a weatherproof pelican case" caption="Figure 6: The Autonomy Data Recorder (ADR), a custom server with 12TB storage and ample ethernet I/O to connect and log all autonomy sensor data." %}
</div>


## References

[1] R. Bernier, X. Cao, G. Roy, and G. Tremblay, “Statistical models for the LIDAR technology: False alarms, receiver operating characteristic curves, and Swerling models,” Optical Engineering, vol. 61, no. 06, Jun. 2022. doi:10.1117/1.oe.61.6.063105 

[2] Z. D. Jeffries, “Evaluation of lidar uncertainty and applications towards slam in off-road environments,” dissertation, Michigan Technological University, Houghton, MI, 2023 

[3] Z. D. Jeffries, J. P. Bos, P. F. McManamon, et al. "Toward open benchmark tests for automotive lidars, year 1: static range error, accuracy, and precision," Optical Engineering 62(3), 031211 (5 Jan 2023) https://doi.org/10.1117/1.OE.62.3.031211

[4] I. Q. Mattson and J. P. Bos, "Assessing the Effect of Inclement Winter Weather on Automotive Lidars: Experimental Design," 2026 IEEE International Workshop on Metrology for Automotive (MetroAutomotive), Brescia, Italy, 2026, pp. 150-155, doi: 10.1109/MetroAutomotive69354.2026.11644655

[5] J. P. Bos, D. J. Chopp, A. Kurup, and N. Spike, “Autonomy at the end of the Earth: An inclement weather autonomous driving data set,” Autonomous Systems: Sensors, Processing, and Security for Vehicles and Infrastructure 2020, p. 6, May 2020. doi:10.1117/12.2558989
