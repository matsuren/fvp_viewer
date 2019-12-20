//#include "RecordImageManager.hpp"
//
//#include <boost/date_time.hpp>
//#include <boost/filesystem.hpp>
//
//// record raw folder
//namespace fs = boost::filesystem;
//namespace pt = boost::posix_time;
////-----------------------------------------------------------------------------
//void RecordImageManager::setRecordFolder(const std::string dirname, int fps)
//{
//	// フォーマットの指定
//	// facet はストリーム側で自動的に delete される
//	auto facet = new pt::time_facet("%Y%m%d_%H%M%S");
//	std::stringstream ss;
//	ss.imbue(std::locale(std::cout.getloc(), facet));
//
//	// 現在の時刻を取得
//	auto now_time = pt::second_clock::local_time();
//	ss << now_time;
//
//	// 録画用フォルダ名の設定
//	record_foldername = dirname + "/"
//		+ ss.str() + "_" + std::to_string(fps) + "fps";
//
//}
////-----------------------------------------------------------------------------
//void RecordImageManager::makeDirectory()
//{
//	// 録画用フォルダの確認
//	if (!already_create_directory) {
//		std::cout << "\n/ ********\ncreate directory : "
//			<< record_foldername << "\n***********/" << std::endl;
//
//		const fs::path path(record_foldername);
//
//		boost::system::error_code error;
//		const bool result = fs::create_directories(path, error);
//		if (!result || error) {
//			std::cout << "ディレクトリの作成に失敗" << std::endl;
//		}
//		already_create_directory = true;
//	}
//}
////-----------------------------------------------------------------------------
//std::string RecordImageManager::getRecordFolder() const {
//	return record_foldername;
//}
////-----------------------------------------------------------------------------
//void RecordImageManager::startRecord() {
//
//	// if the directory exists, do nothing
//	makeDirectory();
//
//	std::cout << "**** save raw data in the following directory! *******\n";
//	std::cout << record_foldername << std::endl;
//	_isRecorded = true;
//
//	// save start time
//	time_start = std::chrono::system_clock::now();
//}
////-----------------------------------------------------------------------------
//void RecordImageManager::stopRecord() {
//	std::cout << "\n**** stop record! *******\n";
//
//	_isRecorded = false;
//	auto end = std::chrono::system_clock::now();  // 計測終了時間
//	//処理に要した時間を秒に変換
//	double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - time_start).count()/1000.0; 
//	recorded_time += elapsed;
//}
////-----------------------------------------------------------------------------
//double RecordImageManager::getElapsedSec()
//{
//	if (_isRecorded)
//	{
//		auto end = std::chrono::system_clock::now();  // 計測終了時間
//		 //処理に要した時間を秒に変換
//		double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - time_start).count() / 1000.0;
//		return recorded_time + elapsed;
//	}
//	else
//	{
//		return recorded_time;
//	}
//}
////-----------------------------------------------------------------------------
