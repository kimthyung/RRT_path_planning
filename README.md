# RRT based path planning

RRT based path planning applied at  Yonsei University

RRT(Rapidly-exploring Random Tree) algorithm is designed to efficiently search paths in nonconvex high-dimensional spaces. RRT based path planning expands the tree to a randomly-sampled point in the configuration space while satisfying given constraints.

This project aims to generate paths using the Rapidly-exploring Random Tree (RRT) algorithm. Starting with testing on a virtual map, the project has also successfully been tested on paths extracted from within Yonsei University premises.



# Path Generation and Post Processing

### Binary Coded Virtual Map for Testing
### Random Tree in Blue and Generated Path to Ending Point in Red
Starting Point with Green Marker, Ending Point with Red Marker


<p align="center"> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/b445e577-c36f-493d-9631-b966e3ff23eb" width="400" height="300"/> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/abd9fe54-531c-4fdb-948f-108c5a6cb804" width="400" height="300"/> <figcaption> 주석 내용 </figcaption>











### Path After First & Second Smoothing Process

<p align="center"> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/6b99d080-216f-41ac-b764-3584f4afc706" width="400" height="300"/> <img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/21a47b74-46e2-4432-893f-83c8af6b6832" width="400" height="300"/> <figcaption align="center">3개 이미지 띄우기</figcaption></p>



### Final Path 


<img src="https://github.com/kimthyung/RRT_path_planning/assets/98934172/8162f19c-9531-44ca-839d-45f5e782fbdf" width="400" height="300"/>



<div align="center">
</div>

<p align="center">  <img src="이미지경로" align="center" width="32%">  <img src="이미지경로" align="center" width="32%"> <figcaption align="center">3개 이미지 띄우기</figcaption></p>

