#include "RecordingManager.h"
#include "../KinectV1/KinectV1.h"
#include "../KinectV2/KinectV2.h"
#include <chrono>
#include <fstream>
#include <filesystem>


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
		std::string path = _mainDirectory + "/" + std::to_string(time(nullptr)) + "/";
		CreateDirectory(path.c_str(), nullptr);

		std::thread t1;
		std::thread t2;

		while(_recording)
		{
			auto start = std::chrono::high_resolution_clock::now(); 
			auto target = start + interval;

			if (_recorder1 != nullptr)
			{
				t1 = std::thread([this, frameNumber, &filenames1, path]() {
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
				t2 = std::thread([this, frameNumber, &filenames2, path]() {
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

}
