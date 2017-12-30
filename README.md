About
------

[1] describes the effect of individual differences in elective movement speed and goal-orientedness on group structuring, 
movement dynamics and foraging. The manuscript examines the ramifications of individuals differences both within and between
groups. Further, by showing that group composition was key to individual performance within it, our work highlights the
potential for social selection as a driver of behavioural differentiation between individuals.

Here, we provide code for the individual based simulations (IBS) that accompany data from the manuscript. The code is split
in three branches:

* master: examines the role of individuals differences in speed and goal-orientedness in a foraging context

* schooling: examines the role of individuals differences in speed and goal-orientedness in group structuring and movement
dynamics

* assay: places individuals differing in speed in a traditional "sociability assay" context to examine the role of speed
in proximity to neighbours in the assay context (traditional measure of "sociability")

The simulation was written in C++ and uses OpenCV for visualisation.

Note: The code provided here is not a perfect overlap with what is described in the manuscript. This code is a subtle variation on what is used in the manuscript. All major results should still hold though.

Use
---

The above code is written using C++11 and tested on MacOS Sierra 10.12.5. The visualisation is executed using OpenCV 3.2.0. In case of troubles installing OpenCV / linking it to the C++ project, visualisation can be turned off by silencing OpenCV headers and the graphics function in main.cpp. The rest of the code should still work on naked C++.

Cite
----

If you use Tracktor, please cite it using this zenodo DOI: [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.840644.svg)](https://doi.org/10.5281/zenodo.840644). Thank You for using and citing individual_differences.

Reference
---------

1. Jolle W. Jolles, Neeltje J. Boogert, Vivek H. Sridhar, Iain D. Couzin, and Andrea Manica, Consistent individual
differences drive collective behaviour and group functioning of schooling fish, _Curr Biol (accepted)_ (2017).
