---  
title: "Part 6 Object Detection Node"  
---

# Part 6 Object Detection Node

Copy **all** the code below into your `object_detection.py` file, and **make sure you read the annotations**!

```py title="object_detection.py"
--8<-- "code_templates/object_detection.py"
```

1. Nothing new here, moving on...
   
2. We're importing the OpenCV library for Python (remember the Python API [that we talked about earlier](../part6.md#opencv)), which is called `cv2`, and *also* that ROS-to-OpenCV bridge interface that we talked about earlier too: `cv_bridge`.

    From `cv_bridge` we're importing the `CvBridge` and `CvBridgeError` classes from the `cv_bridge` library specifically.

3. We need to subscribe to an image topic in order to obtain the data being published to it. You should've already identified the *type of interface* that is published to the `/camera/image_raw` topic, so we import that interface type here (from the `sensor_msgs` package) so that we can build a subscriber to the topic later.

4. We're also importing the Python `Path` class from [the `pathlib` module](https://docs.python.org/3/library/pathlib.html). A very handy tool for doing file operations.

5. Initialising our `#!py ObjectDetection()` Class (should be very familiar to you by now):
    1. Giving our node a name.
    1. Creating a subscriber to the `/camera/image_raw` topic, providing the *interface type* used by the topic (`sensor_msgs/msg/Image` - as imported above), and pointing it to a callback function (`camera_callback`, in this case), to define the processes that should be performed every time a message is obtained on this topic (in this case, the messages will be our camera images)

6. We're creating a flag to indicate whether the node has obtained an image yet or not. For this exercise, we only want to obtain a single image, so we will set the `waiting_for_image` flag to `False` once an image has been obtained and processed, to avoid capturing any more. 

    This flag will then be used to shut down the node when it's done its job.

7. Here, we're defining a callback function for our `self.camera_sub` subscriber...

8. Here, we create an instance of the `CvBridge` class that we imported earlier, and which we'll use later on to convert ROS image data into a format that OpenCV can understand.

9. We're using the CvBridge interface to take our ROS image data and convert it to a format that OpenCV will be able to understand.  In this case we are specifying conversion (or *"encoding"*) to an 8-bit BGR (Blue-Green-Red) image format: `"bgr8"`.
        
    We contain this within a `#!py try`-`#!py except` block though, which is the [recommended procedure when doing this](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython).  Here we *try* to convert an image using the desired encoding, and if a `CvBridgeError` is raised then we print a **warning** message to the terminal.  Should this happen, this particular execution of the camera callback function will stop.

10. Then we check the `waiting_for_image` flag to see if this is the first image that has been received by the node.  If so, then:

    1. Obtain the height and width of the image (in pixels), as well as the number of colour channels.
    1. Print a log message containing the image dimensions.
    1. Pass the image data to the `show_image()` function (defined below). We also pass a descriptive name for the image to this function too (`img_name`).
    
11. This class method presents the image to us in a pop-up window and also calls another method which saves the image to file for us.    
    
12. Display the actual image in a pop-up window:

    1. The image data is passed into the function via the `img` argument,
    1. We need to give the pop-up window a name, so in this case we are using the `img_name` argument that is passed into this class method.
    
13. The `show_image()` class method has a `save_img` argument, which is set to `True` by default, so that this `#!py if` condition is triggered, and *another* class method is called to save the image to file.    

14. We're supplying a value of `0` here, which tells this function to keep this window open **indefinitely** and wait until it is closed manually before allowing our `show_image()` class method to complete.
    
    If we had supplied a value here (say: `1`) then the function would simply wait for 1 millisecond and then close the pop-up window down. In our case however, we want some time to actually look at the image and then close the window down ourselves, manually. 
    
    Once the window has been closed, the execution of our code is able to continue...    

15. We then set the `waiting_for_image` flag to `False` so that we only ever perform these processing steps once (we only want to capture a single image).  This will then trigger the main `#!py while` loop to stop (see below), thus causing the overall execution of the node to stop too.

16. `cv2.destroyAllWindows()` ensures that any OpenCV image pop-up windows that may still be active or in memory are destroyed before the class method exits (and the node shuts down).     

17. This class method handles the saving of the image to a file using `cv2` tools and `pathlib`.
    
18. Here, we define a filesystem location to save images to. 
    
    We want this to exist in a folder called "`myrosdata/object_detection`" in the home directory, so we can use Pathlib's `#!py Path.home().joinpath(...)` to define it (a handy way to access the User's home directory, without needing to know the Users name).
    
    Then, we use the Pathlib `#!py Path.mkdir()` method to create this directory if it doesn't exist already.    
    
19. A full file path is constructed for the image here (using the `Path.joinpath()` method), based on:
        
    1. The `base_image_path` that we defined above 
    1. An image name that is passed into this class method via the `img_name` argument.

20. This saves the image to a `.jpg` file.  We're supplying the `full_image_path` that was created above, and also the actual image data (`self.cv_img`) so that the function knows what image we want to save.

21. We're printing a log message to the terminal to inform us of:

    1. Where the image has been saved to
    1. How big the image is (in terms of its pixel dimensions)
    1. How big the image *file* is (in bytes).

22. We're using `spin_once()` inside a `#!py while` loop here so that we can keep an eye on the value of the `wait_for_image` flag, and stop spinning (i.e. break out of the `#!py while` loop) once it turns `#!py False`.
    
<p align="center">
  <a href="../../part6#ex2_ret">&#8592; Back to Part 6 - Exercise 2</a>
</p>