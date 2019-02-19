#include <ros/ros.h>
#include <opt_msgs/TrackArray.h>
#include <cv_bridge/cv_bridge.h>

#define ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME "dummy_track_publisher"

ros::Publisher pub;


void buildTrack(double x, double y, double z, int id, opt_msgs::Track& track)
{
		track.id = id;

		track.x = x;
		track.y = y;
		track.height = z;
		track.distance = 0;
		track.age = 0;
		track.confidence = 1;

		track.visibility = 2;

		track.stable_id = 0;

		track.box_2D.x = 0;
		track.box_2D.y = 0;
		track.box_2D.width = 0;
		track.box_2D.height = 0;

		track.object_name = "";
		track.face_name = "";
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("starting "<<ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	pub = nh.advertise<opt_msgs::TrackArray>("/optar/dummy_track", 10);

	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
		opt_msgs::TrackArray trackArray;

		opt_msgs::Track track;
		buildTrack(0,0,0, 42, track);
		trackArray.tracks.push_back(track);

		buildTrack(0.5,0,0, 420, track);
		trackArray.tracks.push_back(track);

		buildTrack(1,0,0, 421, track);

		trackArray.tracks.push_back(track);
		buildTrack(0,0.33,0, 4210, track);
		trackArray.tracks.push_back(track);
		buildTrack(0,0.66,0, 42100, track);
		trackArray.tracks.push_back(track);

		buildTrack(0,1,0, 422, track);
		trackArray.tracks.push_back(track);

		buildTrack(0,0,1, 423, track);
		trackArray.tracks.push_back(track);

		pub.publish(trackArray);
		r.sleep();
	}
}


