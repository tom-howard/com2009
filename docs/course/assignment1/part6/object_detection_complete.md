---
title: "Part 6 Object Detection Node (Complete)"  
---

# Part 6 Object Detection Node (Complete)

Here's a full example of the `object_detection.py` node that you should have developed during [Part 6 Exercise 2](../part6.md#ex2).  Also included here is an illustration of how to use the `cv2.circle()` method to create a marker on an image illustrating the centroid of the detected feature, as discussed [here](../part6.md#image-moments).

```py title="object_detection_complete.py"
--8<-- "code_templates/object_detection_complete.py"
```

1. Everything here should be familiar to you from earlier in this exercise, except for this section...

2. Here, we obtain the moments of our colour blob by providing the boolean representation of it (i.e. the `img_mask`) to the `cv2.moments()` function.

3. Then, we are determining *where* the central point of this colour blob is located by calculating the `cy` and `cz` coordinates of it.  This provides us with pixel coordinates relative to the top left-hand corner of the image.

4. Finally, this function allows us to draw a circle on our image at the centroid location so that we can visualise it.  Into this function we pass:

    1. The image that we want the circle to be drawn on.  In this case: `filtered_img`.
    1. The *location* that we want the circle to be placed, specifying the horizontal and vertical pixel coordinates respectively: `(int(cy), int(cz))`.
    1. How *big* we want the circle to be: here we specify a radius of 10 pixels.
    1. The *colour* of the circle, specifying this using a Blue-Green-Red colour space: `(0, 0, 255)` (i.e.: pure red in this case)
    1. Finally, the thickness of the line that will be used to draw the circle, in pixels.

<p align="center">
  <a href="../../part6#ex2b_ret">&#8592; Back to Part 6 - Exercise 2</a>
</p>