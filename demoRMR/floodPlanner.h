#pragma once

#include <memory>
#include <vector>

class FloodPlanner
{
	using Map = std::vector<std::vector<int>>;
public:
	void loadMap(const std::string &filename);

private:
	std::unique_ptr<Map> m_map;
};

