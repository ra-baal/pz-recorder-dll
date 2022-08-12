#pragma once

#include "IRecordingManager.h"
#include "../ICloudRecorder.h"
#include <array>
#include <thread>

class RecordingManager : IRecordingManager
{
public:
	RecordingManager();
	~RecordingManager();
	// Inherited via IRecordingManager
	// virtual RecorderState GetState() override;
	virtual int GetRecordersNumber() override;
    virtual Colors* GetColorBitmaps() override;
	virtual void StartRecording() override;
	virtual void StopRecording() override;
	virtual void SetDirectory(const char* str) override;

protected:
	void findRecorders();
	void RecordingManager::saveSettingsVrfilmFile(std::string directory, std::string settingsVrfilmFilename, int intervalSeconds, std::vector<std::string> pcdFilenames);

	std::shared_ptr<ICloudRecorder> _recorder1;
	std::shared_ptr<ICloudRecorder> _recorder2;

	Colors* _colors;

	std::thread _recordingThread;
	bool _recording;
	std::string _mainDirectory;


};