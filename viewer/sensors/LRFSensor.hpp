#pragma once
#include <urg/Urg_driver.h>
#include <vector>
#include <iostream>
#include <fstream>

struct LRFPoint {
	float x;
	float y;
	LRFPoint()
	{
		x = 0.0f;
		y = 0.0f;
	}
	LRFPoint(float x_, float y_)
	{
		x = x_;
		y = y_;
	}
};

class LRFSensor
{
public:
	LRFSensor(int argc, char *argv[]);

	int grab(void);
	bool retrieve(std::vector<LRFPoint> &LRF_data);

	bool isOpened = false;


	static int loadLRFDataCSV(const std::string lrf_fname, std::vector<LRFPoint> &LRF_data) {
		std::ifstream ifs_lrf(lrf_fname);
		if (!ifs_lrf) {
			std::cout << "error !\n cannot read LRF data file : " << lrf_fname << std::endl;
			return 1;
		}
		// read from csv file
		LRF_data.clear();
		std::string str;
		while (std::getline(ifs_lrf, str)) {
			std::vector<std::string> ret_str = LRFSensor::split(str, ",");
			LRFPoint tmp_pair(std::stof(ret_str[0]), std::stof(ret_str[1]));
			LRF_data.push_back(tmp_pair);
		}
	}

	static std::vector<std::string> split(const std::string& s, const std::string delim)
	{
		std::vector<std::string> result;
		result.clear();

		using string = std::string;
		string::size_type pos = 0;

		while (pos != string::npos)
		{
			string::size_type p = s.find(delim, pos);

			if (p == string::npos)
			{
				result.push_back(s.substr(pos));
				break;
			}
			else {
				result.push_back(s.substr(pos, p - pos));
			}

			pos = p + delim.size();
		}

		// compress
		for (size_t i = 0; i < result.size(); i++) {
			if (result[i] == "" || result[i] == delim) {
				result.erase(result.begin() + i);
				i--;
			}
		}

		return result;
	}
	//-----------------------------------------------------------------------------
	static void getLRFGLdata(const std::vector<LRFPoint> &LRF_data,
		std::vector<float> &vertices, std::vector<unsigned int> &elements, float height)
	{
		const double distance_threshold = 30.0;
		vertices.clear();
		vertices.reserve(3 * 4 * 1200);
		elements.clear();
		elements.reserve(3 * 3 * 1200);

		// origin
		vertices.push_back(0.0f);
		vertices.push_back(0.0f);
		vertices.push_back(0.0f);

		for (size_t i = 1; i < LRF_data.size(); i++)
		{
			double diff_x = LRF_data[i].x - LRF_data[i - 1].x;
			double diff_y = LRF_data[i].y - LRF_data[i - 1].y;
			double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

			int current_num = int(vertices.size() / 3);
			// vertex
			vertices.push_back(LRF_data[i - 1].x);
			vertices.push_back(LRF_data[i - 1].y);
			vertices.push_back(0.0f);

			vertices.push_back(LRF_data[i - 1].x);
			vertices.push_back(LRF_data[i - 1].y);
			vertices.push_back(height);

			vertices.push_back(LRF_data[i].x);
			vertices.push_back(LRF_data[i].y);
			vertices.push_back(height);

			vertices.push_back(LRF_data[i].x);
			vertices.push_back(LRF_data[i].y);
			vertices.push_back(0.0f);

			if (distance < distance_threshold)
			{
				// element
				elements.push_back(current_num);
				elements.push_back(current_num + 1);
				elements.push_back(current_num + 2);

				elements.push_back(current_num);
				elements.push_back(current_num + 2);
				elements.push_back(current_num + 3);
			}

			// floor
			elements.push_back(0);
			elements.push_back(current_num);
			elements.push_back(current_num + 3);

		}

		// floor loop close
		elements.push_back(0);
		int current_num = int(vertices.size() / 3) - 1;
		elements.push_back(current_num);
		elements.push_back(1);
	}

private:
	qrk::Urg_driver urg;
	std::vector<long> data;

	// double radian = urg.index2rad(int(i));
	// index2cos : cos(radian)
	std::vector<double> index2cos;
	// index2sin : sin(radian)
	std::vector<double> index2sin;

};

