#pragma once

#include "IRecordingManager.h"
#include "ICloudRecorder.h"
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

protected:
	std::shared_ptr<ICloudRecorder> _recorder1;
	std::shared_ptr<ICloudRecorder> _recorder2;

	Colors* _colors;

	std::thread _recordingThread;
	bool _recording;

};