#pragma once
#include <iostream>
#include <string>
#include <atomic>
#include <chrono>


// singleton RecordImageManager CLASS
class RecordImageManager {
private:
	RecordImageManager()//コンストラクタを private に置く
	{
		std::cout << "initialize RecordImageManager : " << std::endl;
	};					
	~RecordImageManager() = default;					//デストラクタを private に置く
public:
	RecordImageManager(const RecordImageManager&) = delete;			//コピーコンストラクタを delete 指定
	RecordImageManager& operator=(const RecordImageManager&) = delete;	//コピー代入演算子も delete 指定
	RecordImageManager(RecordImageManager&&) = delete;			//ムーブコンストラクタを delete 指定
	RecordImageManager& operator=(RecordImageManager&&) = delete;		//ムーブ代入演算子も delete 指定

	static RecordImageManager& getInstance() {
		static RecordImageManager inst;			// privateなコンストラクタを呼び出す
		return inst;
	}

	void setRecordFolder(const std::string dirname, int fps) {
	};
	std::string getRecordFolder() const {
		return "Not implemented";
	};

	//-----------------------------------------------------------------------------
	bool isRecorded() const {
		//return _isRecorded;
		return false;
	}

	//-----------------------------------------------------------------------------
	
	bool already_create_directory = false;
	void stopRecord() {};
	void startRecord() {};
	double getElapsedSec() { return 0.0; };
private:

	//void makeDirectory();

	//-----------------------------------------------------------------------------
	//
	bool isStopRecord = false;
	std::atomic<bool> _isRecorded = false;
	std::string record_foldername;
	std::chrono::system_clock::time_point  time_start; // 型は auto で可
	double recorded_time = 0.0;
};
