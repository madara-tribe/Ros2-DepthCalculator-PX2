
#include <iostream>
int x_ori = 17; // (+17px)
double pixel_ratio = 0.171837;       // Real-world units per pixel
double calculateRealCoordinate(double x, double w, int image_width) {
    // Compute pixel offset from image center
    double x_center = x + w / 2.0;
    double x_coordinate = x_center + static_cast<double>(x_ori);

    // Convert to real-world distance using pixel-to-meter (or cm) ratio
    double x_real_coord = x_coordinate * pixel_ratio;

    return x_real_coord;
}

std::vector<std::string> ListImages(const std::string& path) {
    std::vector<std::string> images;
    DIR* dir = opendir(path.c_str());
    if (!dir) {
        std::cerr << "Error: Cannot open directory " << path << std::endl;
        return images;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename == "." || filename == "..") continue;

        std::string ext = filename.substr(filename.find_last_of('.') + 1);
        if (ext == "jpg") {
            images.push_back(path + "/" + filename);
        }
    }
    closedir(dir);
    std::sort(images.begin(), images.end()); // Sort alphabetically
    return images;
}

float calculateBboxSize(double x, double y, double w, double h) {
    double width = w - x;
    double height = h - y;
    if (width <= 0 || height <= 0) {
        return 0.0f;  // invalid box
    }
    return static_cast<float>(width * height);
}

void saveToCSV(const std::string& csvPath, const std::vector<std::vector<std::string>> &data) {
    std::ofstream file(csvPath, std::ios::out | std::ios::app);  // Open file in append mode
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open CSV file " << csvPath << std::endl;
        return;
    }
    // Write headers if the file is newly created
    file << "outputPath,Hight,Width,x_center,y_center,median,bbox_size,xc_real\n";

    for (const auto &row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i != row.size() - 1) file << ",";  // Add commas between values
        }
        file << "\n";  // New line for each row
    }

    file.close();
}

// Function to delete the existing CSV file
void deleteCSV(const std::string &csvPath) {
    if (std::remove(csvPath.c_str()) == 0) {
        std::cout << "Deleted existing CSV file: " << csvPath << std::endl;
    } else {
        std::cout << "No existing CSV file to delete or failed to delete: " << csvPath << std::endl;
    }
}
