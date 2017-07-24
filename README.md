About
=====

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

Note: The code provided here does not overlap perfectly with what is used in the manuscript. There are aspects of the code
that are used in the manuscript, that aren't described here (a zero bounded distribution to chose individual speeds from).
Also, not all code presented here was used in the manuscript (the assay branch).

Reference
==========

1. Jolle W. Jolles, Neeltje J. Boogert, Vivek H. Sridhar, Iain D. Couzin, and Andrea Manica, Consistent individual
differences drive collective behaviour and group functioning of schooling fish, _Curr Biol (accepted)_ (2017).
