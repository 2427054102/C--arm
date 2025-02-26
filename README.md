# C--arm
The Manipulator Sensing and Control System Based on the Depth Camera

In structured light triangulation, a structured light projector and a camera are used to measure the three-dimensional information of an object. The structured light projector is usually a laser or a projector, which projects a specially encoded light pattern onto the object to be measured. By utilizing the known geometric relationship between the optical axis of the camera and the structured light plane of the projector, as well as the internal parameters of the camera, the measured depth is calculated according to similar triangles. 
![image](https://github.com/user-attachments/assets/ef1ebd6f-5176-4e48-898d-ed3a761fdf73)
Collect the RGB images of the working area through the depth camera, and segment the weld seam in the image as the target. Obtain the internal and external parameters of the camera, the pose of the manipulator, and the pose between the two manipulator bases, and reconstruct the three-dimensional coordinates of the weld seam in the coordinate system of the phased array manipulator base.

Plan the detection space field according to the three-dimensional target reconstruction, and control the manipulator to automatically scan the surface of the weld seam based on the detection space field. During the scanning process, perceive the relative pose between the target weld seam and the Mark, and correct the pose of the manipulator to ensure that the phased array probe contacts the detection space field for scanning.

Use the ultrasonic phased array to roughly scan the weld seam to obtain the internal sound field diffusion situation, and reconstruct the angular C-scan image of the weld seam in combination with the pose of the manipulator. Interact with the suspected defect areas in the weld seam based on artificial experience.

Use the Target Tracking algorithm to track the suspected areas. Construct a loss function for the characteristics of the suspected areas based on the defect feature clarity of the target area, the degree of the edge relative to the sector scan, and the deviation degree of the Mark relative to the detection space field. Control the manipulator to explore the optimal pose within the constrained space to observe the characteristic changes of the suspected areas.

Obtain the Mark and the weld seam through target segmentation, and calculate the positional relationship between them. Map the B-scan image to the inside of the weld seam according to the prior structure of the weld seam, and determine the nature of the weld seam. 
![image](https://github.com/user-attachments/assets/bf3b8634-b57e-4b2a-956f-d7673945d476)

