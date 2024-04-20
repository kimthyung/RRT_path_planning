# RRT based path planning

RRT based path planning applied at  Yonsei University

RRT(Rapidly-exploring Random Tree) algorithm is designed to efficiently search paths in nonconvex high-dimensional spaces. RRT based path planning expands the tree to a randomly-sampled point in the configuration space while satisfying given constraints.

This project aims to generate paths using the Rapidly-exploring Random Tree (RRT) algorithm. Starting with testing on a virtual map, the project has also successfully been tested on paths extracted from within Yonsei University premises.



# Path Generation and Post Processing

### Binary Coded Map & Random Tree in Blue and Generated Path to Ending Point in Red Marker



<p align="center"> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/b445e577-c36f-493d-9631-b966e3ff23eb" width="400" height="300"/> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/abd9fe54-531c-4fdb-948f-108c5a6cb804" width="400" height="300"/> 



### Generated Path After First & Second Smoothing Process

<p align="center"> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/6b99d080-216f-41ac-b764-3584f4afc706" width="400" height="300"/> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/21a47b74-46e2-4432-893f-83c8af6b6832" width="400" height="300"/>



### Verification with Paths in Yonsei University 

<p align="center"><img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/8a28a672-6bf5-4ad3-891a-173c7f05bdd0" width="400" height="300"/> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/63350d18-d87a-4fd5-b7fd-f09998fda68b" width="400" height="300"/>


### Generated Path in Yonsei University
<p align="center">
<img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/1b08dfc6-286d-44ba-83d7-9b3ac6a8138c" width="400" height="300"/> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/20d1ab90-42dd-43d0-abdc-1be41e5f269f" width="400" height="300"/>


# Test Map Making


<p align="center">  
   <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/43a103a1-77e7-4815-abde-1779d166292e" align="center" width="32%">
<img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/847a4ef7-9b25-4684-8b82-48f3db684cc0" align="center" width="32%">

 <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/c3743b1e-9710-4b90-b18a-8b97b317acdf" align="center" width="32%">  
  <figcaption align="center">Map Generation Automation with MATLAB Command
  </figcaption></p>



