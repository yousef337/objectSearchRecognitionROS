# objectSearchRecognitionROS

*Under development*

This package is intended to provide the functionality of searching for a specific object inside a room using ROS. Currently, the package only runs on ROS Melodic.

The package consists of several services that help to achieve the required task.

## Services
### Room Rasterization
The main task of this service is to rasterize a given room data, given in geojson format, into a programmatically proccesable data structure, mainly list.
Because a room may not be a rectangle, special values on the lists are allocated to support every room structure. A value of 0 indicate that this coordinate is
outside the room, while one for a value that is inside the room

#### Input:
      name: the name of the room, this is used to get the room data from the geojson file
      blocksize: represents the size that a single list element should represent from the map
#### Output:
      deltaBlock[]: returns the dimenstion of the list, user for reshaping since ROS does not have a standard data structure of 2D lists.
      boundires[]: returns the boundries of the room on the following order x left, y top, x right, y bottom.
      rasterizedRoom[]: the room data as a list.

Images:
![in](https://user-images.githubusercontent.com/56966315/206865400-23d262c5-f2b3-4cb1-b509-c5cdc1f8fbf0.png)
![Full](https://user-images.githubusercontent.com/56966315/206865401-1d18bd8e-ab43-4246-975d-2c50f4903f38.png)

      
 ### Point Sampling (LocationSampling)
 The main task of this service is to sample locations that ought to cover most of the room, in which these coordinates will be used by the
 robot to search for the object.
 search.
 
#### Input:
      int32[] deltaBlock: returns the dimenstion of the list, user for reshaping since ROS does not have a standard data structure of 2D lists.
      int32[] rasterizedRoom: the room data as a list.
      float32[] boundries:  returns the boundries of the room on the following order x left, y top, x right, y bottom.
      float32 blockSize: represents the size that a single list element should represent from the map
      float32 visionScope: this value represent the range of vision that the robot can sense.
#### Output:
      sampledRoom[]: returns the list that represent the room, including setting a value of 2 when a location is selected.
      selectedLocations[]: returns a list of all selected points on the basis of the original map. 
      
Images:
  ![RandomSampler](https://user-images.githubusercontent.com/56966315/206865415-ff7f1fad-536b-405a-9ccf-041a86dcee5a.png)

### Search
The main algorithm that perform the search. It consist of a smach state machine as illustrated on the following image. The algorithm instruct the robot to go to the next point on the sampled map, then the robot perform a head tour to examin its field of vision, then it go to the next point on the sampled locations.

![SMACH_DIAGRAM](https://user-images.githubusercontent.com/56966315/212655939-cae06885-d69c-411a-aa3d-2bb40b0efc05.png)


**IMPORTANT: The object detection service used in main is not followed here. Instead, an ObjectDetection service is provided**



#### Input:
      string name: The name of the item to look for
      string room: The room to look in
      float32 blockSize: represents the size that a single list element should represent from the map
      float32 visionScope: this value represent the range of vision that the robot can sense.

### Output:
      bool found: True if the required item is found

