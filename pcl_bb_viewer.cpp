
#include <iostream>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

/*
 * A class to read data from a csv file containing bounding boxes info in separate lines.
 */
class CSVReader
{
	std::string fileName;
	std::string delimeter;

public:
	CSVReader(std::string filename, std::string delm = ",") : fileName(filename), delimeter(delm)
	{
	}

	// Function to fetch data from a CSV File
	std::vector<std::vector<std::string>> get_bb_info();
};

/*
 * Parses through csv file and extract BB info.
 */
std::vector<std::vector<std::string>> CSVReader::get_bb_info()
{
	std::ifstream file(fileName);
	std::vector<std::vector<std::string>> dataList;
	std::string line = "";

	// Iterate through each line and split the content using delimeter
	while (getline(file, line))
	{
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
		dataList.push_back(vec);
	}

	// Close the File
	file.close();

	return dataList;
}

/*
 * Main function
 */
int main(int argc, char **argv)
{
	// arg check
	if (argc != 2 && argc != 3)
	{
		std::cout << "Usage: pcl_bb_view <Path_to_pcd_file>" << std::endl;
		std::cout << " OR    pcl_bb_view <Path_to_pcd_file> <[Optional] Path_to_bb_csv_file>" << std::endl;
		return (0);
	}

	// load pcd file
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		std::cout << "ERROR: pcd file is empty!" << std::endl;
		return (0);
	}

	// Viewer setup
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

	//load csv file contatining bounding box info
	if (argc == 3)
	{
		// Get data
		CSVReader reader(argv[2]);
		std::vector<std::vector<std::string>> dataList = reader.get_bb_info();

		if (!dataList.empty())
		{
			// // DEBUG: Print the content of row by row on screen
			// for (std::vector<std::string> vec : dataList)
			// {
			// 	for (std::string data : vec)
			// 	{
			// 		std::cout << data << " , ";
			// 	}
			// 	std::cout << std::endl;
			// }

			// Add all bounding boxes
			int count = 0;
			for (std::vector<std::string> vec : dataList)
			{
				try
				{
					// Extract data
					long x = stof(vec[0]);
					long y = stof(vec[1]);
					long z = stof(vec[2]);
					long w = stof(vec[3]);
					long h = stof(vec[4]);
					long l = stof(vec[5]);
					long t = stof(vec[6]);

					// Variable setup
					Eigen::Vector3f position(x, y, z);
					Eigen::Quaternionf orientation;
					orientation = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(t, Eigen::Vector3f::UnitZ()); // Euler 2 Quat

					// Axis Aligned Bounding Box setup
					viewer->addCube(position, orientation, w, l, h, "wire_" + std::to_string(count));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "wire_" + std::to_string(count));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "wire_" + std::to_string(count)); // white wire frame

					// Oriented Bounding Box setup
					viewer->addCube(position, orientation, w, l, h, "solid_" + std::to_string(count));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "solid_" + std::to_string(count));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.0, "solid_" + std::to_string(count));
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "solid_" + std::to_string(count)); // transluscent yellow box

					// increment counter
					count++;
				}
				catch (const std::invalid_argument &ia)
				{
					// NOTE: Ideally a print out should be added
				}
			}
		}
		else
		{
			std::cout << "ERROR: csv file is empty!" << std::endl;
			return (0);
		}
	}
	else
	{
		// Features extraction and variables setup
		pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud);
		feature_extractor.compute(); // extract moment of inertia and other details from the pcd

		std::vector<float> moment_of_inertia;
		std::vector<float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		pcl::PointXYZ min_point_OBB;
		pcl::PointXYZ max_point_OBB;
		pcl::PointXYZ position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);
		feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues(major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter(mass_center);

		// Axis Aligned Bounding Box setup
		viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "AABB");

		// Oriented Bounding Box setup
		Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
		Eigen::Quaternionf quat(rotational_matrix_OBB);
		viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "OBB");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.0, "OBB");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "OBB");

		// Axis definition setup
		pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
		pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
		pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
		pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
		viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
		viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
		viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
	}

	// Vizualize
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return (0);
}