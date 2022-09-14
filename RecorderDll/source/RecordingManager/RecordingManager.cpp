#include "RecordingManager.h"
#include "../KinectV1/KinectV1.h"
#include "../KinectV2/KinectV2.h"
#include <chrono>
#include <fstream>
#include <filesystem>
#include <regex>
#include <ranges>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <format>
#include <pcl/common/transforms.h>

RecordingManager::RecordingManager()
{
	findRecorders();

	_colors = new Colors[2];

	// OpóŸnienie, aby kinecty na pewno by³y gotowe (kinect V2 omija³ pierwsz¹ chmurê).
	// Oczywiœcie dobrze by³oby to rozwi¹zaæ w jakiœ cywilizowany sposób.
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	SetDirectory("./vrfilms");
}

RecordingManager::~RecordingManager()
{
	if (_recording)
		StopRecording();

	if (_recorder1 != nullptr)
	{
		_recorder1->Stop();
		_recorder1 = nullptr;
	}

	if (_recorder2 != nullptr)
	{
		_recorder2->Stop();
		_recorder2 = nullptr;
	}

	delete[] _colors;
}

void
RecordingManager::findRecorders()
{
	try {
		_recorder1 = std::make_shared<KinectV1>();
	}
	catch (DeviceNotFoundException& e) {
		LOG("Kinect V1 not found: " << e.what());
		_recorder1 = nullptr;
	}

	try {
		_recorder2 = std::make_shared<KinectV2>();
	}
	catch (DeviceNotFoundException& e) {
		LOG("Kinect V2 not found: " << e.what());
		_recorder2 = nullptr;
	}

}

int
RecordingManager::GetRecordersNumber()
{
	if (_recorder1 == nullptr && _recorder2 == nullptr)
		return 0;
	
	if (_recorder1 == nullptr || _recorder2 == nullptr)
		return 1;

	return 2;
}

/// <summary>
/// Color bitmap for each kinect.
/// </summary>
/// <returns>Array of Colors</returns>
Colors*
RecordingManager::GetColorBitmaps()
{
	_colors[0] = _recorder1 ? _recorder1->GetColorPixels() : Colors();
	_colors[1] = _recorder2 ? _recorder2->GetColorPixels() : Colors();

	return _colors;
}

void
RecordingManager::StartRecording()
{
	if (GetRecordersNumber() == 0)
		return;

	_recording = true;
	std::chrono::milliseconds interval(750);
	
	_recordingThread = std::thread([this, interval]()
	{
		LOG("RecordingManager::StartRecording() - _recordingThread")
		int frameNumber = 1;
		std::vector<std::string> filenames1;
		std::vector<std::string> filenames2;

		CreateDirectory(_mainDirectory.c_str(), nullptr); // Tworzymy najpierw folder g³owny jeœli nie istnieje.
		path = _mainDirectory + "/" + std::to_string(time(nullptr)) + "/";
		std::string path_copy = path;
		CreateDirectory(path.c_str(), nullptr);

		std::thread t1;
		std::thread t2;

		while(_recording)
		{
			auto start = std::chrono::high_resolution_clock::now(); 
			auto target = start + interval;

			if (_recorder1 != nullptr)
			{
				t1 = std::thread([this, frameNumber, &filenames1, path_copy]() {
					auto t1_start = std::chrono::high_resolution_clock::now(); // Potrzeben tylko do logu.

					std::string filename = "kinectv1-" + std::to_string(frameNumber) + ".pcd";
					_recorder1->RecordOneFrame(path + filename);

					filenames1.push_back(filename);

					auto kinectV1FrameTime = std::chrono::high_resolution_clock::now() - t1_start; // Potrzeben tylko do logu.
					LOG( "v1FrameTime: " << (kinectV1FrameTime.count() / 1000000) << " milliseconds" )
				});
			}


			if (_recorder2 != nullptr)
			{
				t2 = std::thread([this, frameNumber, &filenames2, path_copy]() {
					auto t2_start2 = std::chrono::high_resolution_clock::now(); // Potrzeben tylko do logu.


					std::string filename = "kinectv2-" + std::to_string(frameNumber) + ".pcd";
					_recorder2->RecordOneFrame(path + filename);
					filenames2.push_back(filename);

					auto kinectV2FrameTime = std::chrono::high_resolution_clock::now() - t2_start2; // Potrzebne tylko do logu.
					LOG( "v2FrameTime: " << (kinectV2FrameTime.count() / 1000000) << " milliseconds" )
				});
			}
			
			if (t1.joinable())
				t1.join();

			if (t2.joinable())
				t2.join();

			frameNumber++; // Musi byæ po joinach.

			std::this_thread::sleep_until(target);

			auto timeForFrame = std::chrono::high_resolution_clock::now() - start;
			LOG( "timeForFrame: " << (timeForFrame.count() / 1000000) << " milliseconds" )
		}
		
		saveSettingsVrfilmFile(path, "kinectv1-settings.vrfilm", interval.count(), filenames1);
		saveSettingsVrfilmFile(path, "kinectv2-settings.vrfilm", interval.count(), filenames2);

	});

	// OpóŸnienie, aby na pewno wszystko by³o ok.
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

void
RecordingManager::SetDirectory(const char* str)
{
	_mainDirectory = std::string(str);
}

void
RecordingManager::saveSettingsVrfilmFile(
	std::string directory, 
	std::string settingsVrfilmFilename,
	int interval,
	std::vector<std::string> pcdFilenames)
{
	std::ofstream outfile;
	outfile.open(directory + settingsVrfilmFilename, std::ios::out | std::ios::trunc );
	outfile << "pcd\n";
	outfile << interval << "\n"; // Odleg³oœæ w czasie miêdzy kolejnymi klatkami (interval).
	outfile << std::to_string(pcdFilenames.size()) << '\n';
	for (std::string filename : pcdFilenames) 
		outfile << filename << '\n';
}

void
RecordingManager::StopRecording()
{
	_recording = false;
	
	if (_recordingThread.joinable())
		_recordingThread.join();

	std::thread t(&RecordingManager::Merge, this);
}

void
RecordingManager::Merge()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ndCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	std::vector<std::filesystem::path> p1;
	std::vector<std::filesystem::path> p2;
	std::vector<std::filesystem::path> p4;

	std::filesystem::path path_to_files(path);

	const std::regex r1("kinectv1.*\\.pcd$");
	const std::regex r2("kinectv2.*\\.pcd$");
	const std::regex r3(".*\\.png", std::regex_constants::icase);
	std::vector<std::filesystem::path> p3;
	for (auto const& dir_entry : std::filesystem::directory_iterator(path))
	{
		if (std::regex_match(dir_entry.path().filename().string(), r1))
			p1.push_back(dir_entry.path().filename());
		else if (std::regex_match(dir_entry.path().filename().string(), r2))
			p2.push_back(dir_entry.path().filename());
		else if (std::regex_match(dir_entry.path().filename().string(), r3))
			p3.push_back(dir_entry.path().filename());
		else
			p4.push_back(dir_entry.path().filename());
	}

	for (auto& entry : p3)
	{
		std::filesystem::path path_settings = path_to_files.string() + "\\" + entry.string();
		std::filesystem::path path_settings_out = std::filesystem::relative("out").string() + "\\" + entry.string();
		std::filesystem::copy_file(path_settings, path_settings_out, std::filesystem::copy_options::update_existing);
	}
	std::stringstream ss;
	if (std::ifstream file(path_to_files.string() + "\\" + p4[0].string()); file.is_open())
	{
		int i = 0;
		std::string line;
		while (std::getline(file, line))
		{
			if (i <= 2)
				ss << line << std::endl;
			else
			{
				auto suffix = line | std::ranges::views::split('-') | std::ranges::views::transform([](auto&& str) {return std::string_view(&*str.begin(), std::ranges::distance(str)); });
				std::string suf = "";
				for (auto&& word : suffix)
					suf = word;
				auto name = suf | std::ranges::views::split('.') | std::ranges::views::transform([](auto&& str) {return std::string_view(&*str.begin(), std::ranges::distance(str)); });
				std::string filen = "";
				for (auto&& word : name) {
					filen = word;
					break;
				}
				ss << std::format("frame-{}.pcd", filen) << std::endl;
			}
			i++;
		}
		file.close();
	}

	std::ofstream outfile(std::filesystem::relative("out").string() + "\\settings.vrfilm", std::fstream::out);
	outfile << ss.rdbuf();
	outfile.close();

	assert(p1.size() == p2.size());

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ndCloud_transformed(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr resCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;


	Eigen::Matrix4f transformation_matrix;

	for (int i = 0; i < p1.size(); i++)
	{
		std::string filename = p1[i].stem().string();
		auto suffix = filename | std::ranges::views::split('-') | std::ranges::views::transform([](auto&& str) {return std::string_view(&*str.begin(), std::ranges::distance(str)); });
		std::string suf = "";
		for (auto&& word : suffix)
			suf = word;
		pcl::io::loadPCDFile(path_to_files.string() + "\\" + p1[i].string(), *cloud);
		pcl::io::loadPCDFile(path_to_files.string() + "\\" + p2[i].string(), *ndCloud);

		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		transform_2.rotate(Eigen::AngleAxisf(-22 / 7.0, Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(*cloud, *cloud, transform_2);

		for (auto& point : *cloud)
			point.x *= -1;

		if (i == 0) {
			icp.setInputSource(cloud);
			icp.setInputTarget(ndCloud);
			icp.setEuclideanFitnessEpsilon(1e-7);
			icp.setTransformationEpsilon(1e-3);
			icp.setMaximumIterations(1000);
			icp.setMaxCorrespondenceDistance(0.5);
			icp.align(*cloud);
			transformation_matrix = icp.getFinalTransformation();
		}

	    pcl::transformPointCloud(*cloud, *cloud_transformed, transformation_matrix);

	    Eigen::Isometry3f pose(Eigen::Isometry3f(Eigen::Translation3f(
	        ndCloud->sensor_origin_[0],
	        ndCloud->sensor_origin_[1],
	        ndCloud->sensor_origin_[2])) *
	        Eigen::Isometry3f(ndCloud->sensor_orientation_));
	    transformation_matrix = pose.matrix();
	    pcl::transformPointCloud(*ndCloud, *ndCloud_transformed, transformation_matrix);

	    *resCloud += *cloud_transformed;
	    *resCloud += *ndCloud_transformed;

		pcl::io::savePCDFileASCII(std::format("out\\frame-{}.pcd", suf), *resCloud);
		resCloud->clear();
	}
}
