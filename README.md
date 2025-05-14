# Depth Calculator(PX2) through ROS2
This repository provides a pipeline to estimate the absolute distance from a single RGB image by using:
- MiDaS for monocular depth estimation
- YOLOv7 for real-time object detection

## predict relative distance phase
the relevant distance of an object is calculated by the median estimated distance of all pixels inside the bounded
box.
![Image](https://github.com/user-attachments/assets/1e33c8b5-7fd3-47d6-ab27-3dbab618515e)

## convert predicted depth to absolute distance phase
through least squares fitting repository, be able to convert predicted depth values into real-world absolute distances.
![Image](https://github.com/user-attachments/assets/091e6707-24b9-4342-ba62-cfdce6342772)


# How to use
- put the training image into the data folder
- run inference.cpp to estimate relative distance
- record these data to csv file and it is save to data folder automatically.


## Method: Calculating Real-World Distance from Image Edge to Object Center

This method estimates the real-world horizontal distance between the left edge of an image and the center of a detected object, using bounding box data and a known pixel-to-centimeter ratio.
- The input is a 2D image 
- An object has been detected with a bounding box: (x, y, w, h) 
- by letting real-world distance correspond to 1 pixel in the horizontal axis, you get pixel_to_cm_ratio (in cm/pixel). This may be computed from prior calibration or depth-based fitting.
```
double pixel_ratio = 0.171837;       // Real-world units per pixel
double calculateRealCoordinate(double x, double w, int image_width) {
    // Compute pixel offset from image center
    double x_center = x + w / 2.0;
    double x_coordinate = x_center // + static_cast<double>(x_ori);
    // Convert to real-world distance using pixel-to-meter (or cm) ratio
    double x_real_coord = x_coordinate * pixel_ratio;
    return x_real_coord;
}
```



# Referenes
- [Absolute distance prediction based on deep learning object detection and monocular depth estimation models](https://arxiv.org/abs/2111.01715)
