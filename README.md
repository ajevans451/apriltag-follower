# AprilTag Follower
## AJ Evans and Jack Levitsky

The goal was to direct a NEATO to navigate around the Olin oval.

To achieve this goal, we utilized the AprilTag visual fiducial system to capitalize on the AprilTags that were already present in preset positions around the Olin oval.

The AprilTags that were set up were sorted according to their ID numbers, meaning a valid way to navigate the oval would be for the NEATO to seek the next ID in the sequence and navigate toward it; however, we chose to have the NEATO seek the farthest AprilTag that it detected when it detected multiple tags in one frame. This allowed initial testing to include AprilTags that were not ordered sequentially and forced us to dive further into handling the tag pose data.

When reading images, the shape of the matrix in the image_msg was not compatible with the AprilTag detection function; we could not encode the incoming image in RGB, so we had to encode it in grayscale or mono encoding for the detector to be able to read the AprilTags in the photo.

When calibrating the camera mounted on the NEATO, the ROS camera calibration packages had difficulties installing and could not detect the packages it was dependent on, leaving it unable to be built. To get the camera intrinsics, we instead exported images and used the Camera Calibrator tool on MATLAB.

If granted additional time, we would increase the resolution of the images we were capturing to attempt to detect Apriltags at an increased distance.
Additionally, adding camera support and stabilization would also be beneficial in increasing the range at which the NEATO could detect the AprilTags.

First and foremost, we have gained confidence from this project working with AprilTags and computer vision that will undoubtedly be helpful at some point in future robotics projects. Our other main takeaway from the project is how useful it can be to get a new set of eyes on a problem. When working to fix the same issue for a long time it can be easy to get lost in a certain way of trying to solve it. Getting someone else to take a look saved us from falling into that rut and ending the project without a successful demo.
