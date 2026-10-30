---
layout: splash
classes: wide
mathjax: true
title: "Experiment Configuration"
permalink: /experiment/
author_profile: false
toc: true
toc_label: "Experiment Components"
toc_icon: "list"
toc_sticky: true
---

<!-- ADD INTRO SECTION HERE -->

* TOC
{:toc}

## Lidar Targets

In this experiment, we use rigid, planar targets to evaluate range uncertainty. Planar targets are provide a large area that result in multiple point returns at a specified range. Furthermore, we include multiple target reflectivities to evaluate range performance of two representative reflectivities.

### NIST Traceable Lambertian Targets

To provide a baseline level of performance, we leverage multiple Lambertian targets placed across the test field. Lambertian surfaces scatter light uniformly in all directions, opposted to specular reflectance which reflects light in one specific direction, such that Lambertian targets are ideal for lidar detection at any angle.

Specifically, we use twelve [Labsphere Permaflect® Targets](https://www.labsphere.com/product/permaflect-targets-rigid-lidar-commercial-and-custom/) at 10% and 80% reflectivities. Ten targets are 0.5 m x 0.5 m in size and two are 1 m x 1 m in size, used for farther distances. Half of the targets of all sizes are 10% reflective, and other half are 80% reflective so we can evaluate range uncertainty at two representative reflectivities.

{% include figure image_path="/assets/media/1m_targets.jpg" alt="Two lidar targets are positioned in an empty parking lot" caption="Figure 1: Two, 1m Labsphere targets at 10% and 80% spectral reflectance" class="align-center half" %}

The Labsphere Permaflect® targets used in our experiment are calibrated at a single point on the target at wavelengths from 350 to 2400 nm at 10 nm increments, with a reported spectral reflectance of $\pm 2$ % the advertised reflectance at 905 nm, a common wavelength used by automotive lidar sensors. 

Our targets do not feature any mounting hardware, therefore, we designed a series of 3D printed brackets to mount the targets to the post. For more information, please refer to the Target Stands section.

### Low-Cost Painted Analogues

We recognize the cost of NIST traceable targets is unattainable for many testing groups. Multiple research papers refer to homemade, painted lidar targets that approximate the desired spectral reflectance with varying paint types. [1, 2]

We aim to leverage this experiment to evaluate the range uncertainty of custom, painted target analogues to that of the Labsphere targets, and report multiple paints that represent varying levels of spectral reflectance.

As we characterize the painted analogues, information of each paint, spectral reflectance, and uncertainty analysis will be provided at the link below.

<!-- not functional until page is created, need data -->
[Painted Analogue Reflectivity Data]({{ '/painted-analogue/' | relative_url }}){: .btn .btn--primary}  

## Target Stands

Custom built target stands were designed to mount each lidar target, and remain easy to assemble and transport at the test site.

<!-- ADD PICTURE OF CURRENT TARGET STAND HERE -->



<a href="{{ '/assets/files/' | relative_url }}" class="btn btn--primary btn--large" download>
  <i class="fas fa-download"></i> Download 0.5 m Target Stand Instructions
</a>

<a href="{{ '/assets/files/' | relative_url }}" class="btn btn--primary btn--large" download>
  <i class="fas fa-download"></i> Download 1 m Target Stand Instructions
</a>


<a href="{{ '/assets/files/corner_target_bracket.stl' | relative_url }}" class="btn btn--primary btn--large" download>
  <i class="fas fa-download"></i> Download Corner Bracket .stl
</a>

<a href="{{ '/assets/files/middle_target_bracket.stl' | relative_url }}" class="btn btn--primary btn--large" download>
  <i class="fas fa-download"></i> Download Middle Bracket .stl
</a>



## Test Site



## Target Placement



## Lidar Under Test

{% include figure image_path="/assets/media/lidars_under_test.png" alt="Four automotive lidar sensors used in this test are placed next to each other" caption="Figure 2: Primary lidars evaluated in this experiment, from left to right: Velodyne VLP-32C, Ouster OS-1, Robosense Ruby, and the Luminar Iris" class="align-center half" %}


## Computer Setup



## Logistics Notes



## References

[1] R. Bernier, X. Cao, G. Roy, and G. Tremblay, “Statistical models for the LIDAR technology: False alarms, receiver operating characteristic curves, and Swerling models,” Optical Engineering, vol. 61, no. 06, Jun. 2022. doi:10.1117/1.oe.61.6.063105 

[2] Z. D. Jeffries, “Evaluation of lidar uncertainty and applications towards slam in off-road environments,” dissertation, Michigan Technological University, Houghton, MI, 2023 