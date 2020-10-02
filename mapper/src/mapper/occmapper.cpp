// Author: Benned Hedegaard
// Last revised 8/27/2020

#include "mapper/occmapper.h"

using namespace std;

// Formula from Wikipedia for now. More quaternion understanding is needed.
double quat_to_yaw(geometry_msgs::Quaternion q)
{
	return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

// Returns log-odds representation of the given probability.
double log_odds(double probability)
{
	return log( probability / (1.0 - probability) );
}

// Constructor. Inputs are grid resolution, width/height of grid in # cells,
// origin of the map, and probabilities: prior, free, occupied.
OccMapper::OccMapper(double res, int threshold, unsigned int width, unsigned int height,
	double obstacle_width, geometry_msgs::Pose origin, double p0,
	double p_free, double p_occ)
{
	RESOLUTION = res;
	if (RESOLUTION == 0.0)
		RESOLUTION = 0.01;
	_map.info.resolution = RESOLUTION;
	_map.info.width = width;
	_map.info.height = height;
	_map.info.origin = origin;
	
	vector<int8_t> data(width*height, 0);
	_map.data = data;
	
	// (0,0) in the map is its bottom-left corner in the 2D plane.
	MIN_X = origin.position.x;
	MAX_X = origin.position.x + width*RESOLUTION;
	MIN_Y = origin.position.y;
	MAX_Y = origin.position.y + width*RESOLUTION;
	
	// How many steps of size resolution are obstacles deep?
	_occ_steps = floor(obstacle_width/RESOLUTION);
	_threshold = threshold;
	
	// Input appropriate p0, p_free, and p_occ.
	l0 = log_odds(p0);
	l_free = log_odds(p_free);
	l_occ = log_odds(p_occ);
	
	return;
}

OccMapper::~OccMapper() {} // Deconstructor

void OccMapper::handleOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	_pose = msg->pose.pose;
	return;
}

void OccMapper::handleLaserscan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	_scan = *msg;
	update(_pose, _scan);
	return;
}

void OccMapper::handleClick(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	cout << "Handling point..." << endl;
	int index = point_to_index(msg->point.x, msg->point.y);
	cout << "Index was " << index << "." << endl;
	_map.data[index] = 100;
	publishMap();
	cout << "New map published." << endl;
	return;
}

void OccMapper::publishMap()
{
	_map.header.stamp = ros::Time::now();
	map_pub.publish(_map);
	return;
}

// Returns if a given cell is occupied.
bool OccMapper::occupied(double x, double y)
{
	if (inMap(x, y)) {
		int index = point_to_index(x, y);
		return (_map.data[index] > _threshold);
	} else {
		return true;
	}
}

// Returns if the given (x,y) point is in the map's range.
bool OccMapper::inMap(double x, double y)
{
	return ((MIN_X < x) && (x < MAX_X) && (MIN_Y < y) && (y < MAX_Y));
}

// Returns the map's column index for the given x coordinate.
int OccMapper::x_to_col(double x)
{
	return floor((x - _map.info.origin.position.x)/RESOLUTION);
}

// Returns the map's row index for the given y coordinate.
int OccMapper::y_to_row(double y)
{
	return floor((y - _map.info.origin.position.y)/RESOLUTION);
}

// Returns the index in the grid of the given (x,y) point.
int OccMapper::point_to_index(double x, double y)
{
	return _map.info.width*y_to_row(y) + x_to_col(x);
}

void OccMapper::update(geometry_msgs::Pose pose, sensor_msgs::LaserScan scan)
{
	if (scan.ranges.size() == 0)
		return;
	
	double heading = quat_to_yaw(pose.orientation);
	
	// Track cells that need to be updated from each laser.
	set<int> occupied_cells;
	set<int> free_cells;
	
	for (int i = 0; i < scan.ranges.size(); i++)
	{
		// Clear sets for each laser; update cells at most once per laser.
		occupied_cells.clear();
		free_cells.clear();
		
		if ((scan.ranges[i] < scan.range_min) || (scan.ranges[i] > scan.range_max))
			continue; // Skip invalid laser returns.
		
		double bearing = scan.angle_min + i*scan.angle_increment; // radians
		double global_bearing = heading + bearing;
		
		int free_steps = floor(scan.ranges[i]/RESOLUTION);
		
		// Step along the laser scan; all cells here are free.
		for (int step = 0; step < free_steps; step++)
		{
			double range = step*RESOLUTION;
			double x = pose.position.x + range*cos(global_bearing);
			double y = pose.position.y + range*sin(global_bearing);
			if (!inMap(x, y))
				continue;
			int cell_index = point_to_index(x, y);
			free_cells.insert(cell_index);
		}
		
		// Step across the occupied range for this laser.
		for (int step = 0; step < _occ_steps; step++)
		{
			double range = scan.ranges[i] + step*RESOLUTION;
			double x = pose.position.x + range*cos(global_bearing);
			double y = pose.position.y + range*sin(global_bearing);
			if (!inMap(x, y))
				continue;
			int cell_index = point_to_index(x, y);
			occupied_cells.insert(cell_index);
		}
		
		// Now update each grid cell accordingly.
		for (set<int>::iterator free = free_cells.begin();
			free != free_cells.end(); ++free)
		{
			double result = _map.data[*free] + l_free - l0;
			if (result < 0.0)
				result = 0;
			if (result > 100.0)
				result = 100;
			_map.data[*free] = (int) result;
		}
		
		for (set<int>::iterator occ = occupied_cells.begin();
			occ != occupied_cells.end(); ++occ)
		{
			double result = _map.data[*occ] + l_occ - l0;
			if (result < 0.0)
				result = 0;
			if (result > 100.0)
				result = 100;
			_map.data[*occ] = (int) result;
		}
	}
	
	publishMap();
	return;
}
