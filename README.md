# Optical tracking using certus

## Overview
Optical tracking consists of the following stages.

Define the coordinate systems:
1. Digitisation of landmarks (femur, tibia, patella,...)
2. Definition of bone coordinate system
3. Definition of tracker coordinate system
4. Calculation of bone-to-tracker transform

Process tracked data:
1. Import tracked data
2. Define dynamic tracker positions
3. Calculate dynamic bone positions

## Using the output
[`image.tex`](https://github.com/jjholt/certus/blob/main/image.tex) provides examples of how the output data can be plotted in LaTeX.

Alternatively, the output is serialised within [`main.rs`](https://github.com/jjholt/certus/blob/main/src/main.rs#L78-L80), but structure is defined within [`output.rs`](https://github.com/jjholt/certus/blob/main/src/output.rs).
