//
// Created by jjgomez on 1/02/23.
//

#include <opencv2/opencv.hpp>

using namespace std;


void ComputeXGradient(const cv::Mat& image, cv::Mat& gradient_x) {
    // Define the kernel for the x-gradient.
    cv::Mat kernel_x = (cv::Mat_<float>(3, 3) << -1, 0, 1,
            -2, 0, 2,
            -1, 0, 1);

    // Apply the kernel to the image.
    cv::filter2D(image, gradient_x, -1, kernel_x);
}

int main() {
    // Load the image from disk.
    cv::Mat image;
    image = cv::imread("Lab0/Data/task3.png");

    // Display the image.
    cv::imshow("Input image", image);

    // Wait until the user press any key
    cv::waitKey(0);
    
    // We define the gradient matrix as an image of
    // floats with the same size of the image
    cv::Mat gradient_x(image.size(), CV_32FC1);

    // Compute the x-gradient of the image.
    ComputeXGradient(image, gradient_x);

    // Convert the gradient back to 8 bit image (for drawing purposes).
    cv::Mat converted_gradient_x;
    convertScaleAbs(gradient_x, converted_gradient_x);

    // Draw the image in a new window.
    cv::imshow("Computed gradient", converted_gradient_x);

    // Wait until the user press any key
    cv::waitKey(0);

    return 0;
}