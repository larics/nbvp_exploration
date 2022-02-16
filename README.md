# nbvp_exploration
This repository contains the code for a shadowcasting-based next-best-view planner, presented in the paper:\
**A Shadowcasting-Based Next-Best_View Planner for Autonomous 3D Exploration**\
Ana Batinovic, Antun Ivanovic, Tamara Petrovic and Stjepan Bogdan.

If you use this code in your research, please cite our [journal paper](https://ieeexplore.ieee.org/document/9695290):

```
@ARTICLE{Batinovic-RAL-2022,
  author={Batinovic, Ana and Ivanovic, Antun and Petrovic, Tamara and Bogdan, Stjepan},
  journal={IEEE Robotics and Automation Letters}, 
  title={A Shadowcasting-Based Next-Best-View Planner for Autonomous 3D Exploration}, 
  year={2022},
  volume={7},
  number={2},
  pages={2969-2976},
  doi={10.1109/LRA.2022.3146586}}
```
The repository is based on the proposed [nbvplanner](https://github.com/ethz-asl/nbvplanner).

Proposed shadowcasting-based next-best-view planner is capable of autonomously exploring a previously unknown bounded area and creating an OctoMap of the environment. The results showed an improved behaviour in terms of both computation and total exploration time compared to state-of-the-art strategies. The proposed information gain calculation and path evaluation ensures target evaluation in a short computation time, while a novel dead end recovery algorithm speeds up the exploration process. This 3D exploration planner has been successfully tested and analysed in simulation scenarios and compared with state-of-the-are strategies.

Video recordings of shadowcasting-based next-best-view exploration can be found at [YouTube](https://www.youtube.com/playlist?list=PLC0C6uwoEQ8ZDhny1VdmFXLeTQOSBibQl).

This README gives a brief overview. For more information, please refer to the [wiki](https://github.com/larics/nbvp_exploration/wiki), where all further instructions on installation, visualization of exploration progress, as well as demo scenarios can be found.


## Contact

You can contact [Ana Batinovic](mailto:ana.batinovic@fer.hr) for any question or remark.
