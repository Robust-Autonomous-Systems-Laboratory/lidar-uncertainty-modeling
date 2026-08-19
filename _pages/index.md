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
  overlay_image: /assets/media/ruby_krc_summer.jpeg
  # actions:
  #   - label: "Download"
  #     url: "https://github.com/mmistakes/minimal-mistakes/"
  caption: ""
excerpt: "A guide to uncertainty sources in time of flight, automotive lidar sensors in varying conditions"
# intro: 
#   - excerpt: 'Nullam suscipit et nam, tellus velit pellentesque at malesuada, enim eaque. Quis nulla, netus tempor in diam gravida tincidunt, *proin faucibus* voluptate felis id sollicitudin. Centered with `type="center"`'
# feature_row:
#   - image_path: assets/images/unsplash-gallery-image-1-th.jpg
#     alt: "placeholder image 1"
#     title: "Placeholder 1"
#     excerpt: "This is some sample content that goes here with **Markdown** formatting."
#   - image_path: /assets/images/unsplash-gallery-image-2-th.jpg
#     image_caption: "Image courtesy of [Unsplash](https://unsplash.com/)"
#     alt: "placeholder image 2"
#     title: "Placeholder 2"
#     excerpt: "This is some sample content that goes here with **Markdown** formatting."
#     url: "#test-link"
#     btn_label: "Read More"
#     btn_class: "btn--primary"
#   - image_path: /assets/images/unsplash-gallery-image-3-th.jpg
#     title: "Placeholder 3"
#     excerpt: "This is some sample content that goes here with **Markdown** formatting."
# feature_row2:
#   - image_path: /assets/images/unsplash-gallery-image-2-th.jpg
#     alt: "placeholder image 2"
#     title: "Placeholder Image Left Aligned"
#     excerpt: 'This is some sample content that goes here with **Markdown** formatting. Left aligned with `type="left"`'
#     url: "#test-link"
#     btn_label: "Read More"
#     btn_class: "btn--primary"
# feature_row3:
#   - image_path: /assets/images/unsplash-gallery-image-2-th.jpg
#     alt: "placeholder image 2"
#     title: "Placeholder Image Right Aligned"
#     excerpt: 'This is some sample content that goes here with **Markdown** formatting. Right aligned with `type="right"`'
#     url: "#test-link"
#     btn_label: "Read More"
#     btn_class: "btn--primary"
# feature_row4:
#   - image_path: /assets/images/unsplash-gallery-image-2-th.jpg
#     alt: "placeholder image 2"
#     title: "Placeholder Image Center Aligned"
#     excerpt: 'This is some sample content that goes here with **Markdown** formatting. Centered with `type="center"`'
#     url: "#test-link"
#     btn_label: "Read More"
#     btn_class: "btn--primary"

---

## Introduction

Welcome to my homepage! This is where you can write a brief pitch, introduce your project, or welcome visitors to your site. 

Minimal Mistakes handles typography cleanly, so standard Markdown text will look polished right out of the box.

---

## Lidar Range Equation

$$
𝑃_𝑟=𝑃_𝑡∗\rho∗cos⁡(\Theta)∗(\pi 𝐷^2)/4𝑅∗\eta_𝑠𝑦𝑠∗\eta_𝑎𝑡𝑚
$$

- $P_r$: Received optical power [W]
- $P_t$: Transmitted optical power [W]
- $\rho$: Target reflectivity
- $cos(\Theta)$: Incidence angle
- $D$: Aperature diameter [m]
- $R$: Target range [m]
- $\eta_sys$: Quantum efficency
- $\eta_atm$: Atmospheric efficency

