
#include <vector>
#include <string>
#include <map>
#include <list>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>



#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800

class OccupancyGridExplorer {
    protected:
        ros::NodeHandle nh_;
        tf::StampedTransform transform;
        ros::Subscriber end_sub;
        tf::TransformListener listener_;
		ros::Subscriber og_sub_;
		ros::Publisher twist_pub_;
		ros::Publisher target_pub_;
		cv::Point og_center_;
        cv::Mat_<uint8_t> og_, og_2;
        cv::Mat_<cv::Vec3b> og_rgb_;
        cv::Mat_ <uint8_t> sig_map_display;
        cv::Mat_<float> sig_map;
        cv::Mat_<float> temp_map;
        nav_msgs::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        ros::Subscriber signal_sub;
        double alpha;
        bool init = true;
        double robot_radius_;
        double sig;
		bool init2 = true;
        typedef std::multimap<float, cv::Point3i> Heap;
        
        cv::Point P2(const cv::Point3i & P) {return cv::Point(P.x,P.y);}
        // Callback for Occupancy Grids
        void og_callback(const nav_msgs::OccupancyGridConstPtr & msg) {
            
            ROS_INFO("il completed");
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_2 = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution);
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    int8_t v = msg->data[j*msg->info.width + i];
                    switch (v) {
                        case 0: 
                            og_(j,i) = FREE; 
                            og_2(j,i) = FREE; 
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; 
                            og_2(j,i) = OCCUPIED; 
                            break;
                        case -1: 
                        default:
                            og_(j,i) = FREE;
                            og_2(j,i) = UNKNOWN;  
                            break;
                    }
                }
            }
            // dilatation/erosion part
            int erosion_type = cv::MORPH_RECT ;
            int erosion_size = robot_radius_/info_.resolution ;
            cv::Mat element = cv::getStructuringElement(erosion_type,
                    cv::Size(2*erosion_size+1,2*erosion_size+1),
                    cv::Point( erosion_size, erosion_size));
            cv::erode( og_, og_, element );
            cv::cvtColor(og_2, og_rgb_, CV_GRAY2RGB);
            //cv::imshow( "OccGridRGB", og_rgb_ );
            ROS_INFO("ikf completed");
			if (init){
				init = false;
				std_msgs::Bool msg;
				msg.data = true;
				end_callback(msg);
			}
            
            
        }
        
        void end_callback(const std_msgs::Bool & msg) {
			//listener_.waitForTransform(frame_id_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
			//listener_.transformPose(frame_id_,*msg, pose);
			// this gets the current pose in transform
			listener_.lookupTransform("map",base_link_, ros::Time(0), transform);
			
			// TODO: rotate 
			/*
			geometry_msgs::Twist twist;
			
			double r_yaw = tf::getYaw(transform.getRotation());
			twist.linear.x = 0.0;
			twist.angular.z = 2.0;
			
			while (fmod(tf::getYaw(transform.getRotation()) - r_yaw+2*M_PI,(2*M_PI)) < 0.2){
				listener_.lookupTransform("map",base_link_, ros::Time(0), transform);
				twist_pub_.publish(twist);
				ros::Duration(0.1).sleep();
			}
			while (fmod(tf::getYaw(transform.getRotation()) - r_yaw+2*M_PI,(2*M_PI)) > 0.4){
				listener_.lookupTransform("map",base_link_, ros::Time(0), transform);
				twist_pub_.publish(twist);
				ros::Duration(0.1).sleep();
			}
			twist.linear.x = 0.0;
			twist.angular.z = 0.0;
			twist_pub_.publish(twist);*/
            // TODO: list border points and take best
           
            ROS_INFO("befr completed");
            ROS_INFO("done completed");
            double x = transform.getOrigin().x() / info_.resolution;
            double y = transform.getOrigin().y() / info_.resolution;
            
            double min_dist =  0.1/ info_.resolution;
            double sig = 500;
            double best_score=1e30;
            cv::Point best_point;
            for (int i=1; i<og_2.rows -1; i++){
				for(int j=1; j< og_2.cols-1; j++){
						
						if ((og_2(i, j) == FREE && og_(i, j) == FREE) && 
							((og_2(i, j+1) == UNKNOWN && og_(i, j+1) == FREE) || 
							 (og_2(i, j-1) == UNKNOWN && og_(i, j-1) == FREE) || 
							 (og_2(i+1, j) == UNKNOWN && og_(i+1, j) == FREE) || 
							 (og_2(i-1, j) == UNKNOWN && og_(i-1, j) == FREE) )){
							cv::Point P =cv::Point(j,i);
							double dist = hypot(x-(P.x-og_center_.x), y-(P.y-og_center_.y))*info_.resolution;
							double score = (dist-min_dist)*(dist-min_dist);
							og_rgb_(i, j) = cv::Vec3b(255, int(exp(-score)*255), 144);
							if (score<best_score){
								best_point=P-og_center_;
								best_score = score;
							}
						}
						
					}
			}
            
            cv::imshow( "OccGridRGB", og_rgb_ );
            double theta;
            
            // TODO: start planner to best_point
            geometry_msgs::PoseStamped point_msg;
            point_msg.header.stamp = ros::Time::now();
            point_msg.header.frame_id = "map";
            ROS_INFO("done2 completed %d %d",best_point.x,best_point.y);
            point_msg.pose.position.x = double(best_point.x)*info_.resolution;
            point_msg.pose.position.y = double(best_point.y)*info_.resolution;
            theta = atan2(point_msg.pose.position.x-x, point_msg.pose.position.y-y);
            ROS_INFO("done3 completed %f %f",point_msg.pose.position.x,point_msg.pose.position.y);
            point_msg.pose.position.z = 0.0;
            point_msg.pose.orientation.z = theta;
            tf::Quaternion Q = tf::createQuaternionFromRPY(0,0,theta);
            tf::quaternionTFToMsg(Q,point_msg.pose.orientation);
			target_pub_.publish(point_msg);
		}
		
		void signal_callback(const std_msgs::Float32ConstPtr & msg) {
			if(init2){
				init2 = false;
				temp_map = cv::Mat_<float>(info_.height, info_.width, 0.0);
				sig_map = cv::Mat_<float>(info_.height,info_.width, 0.0);
				sig_map_display = cv::Mat_<uint8_t>(int(info_.height), int(info_.width),0xFF);
			}
			sig= msg->data;
			listener_.lookupTransform("map",base_link_, ros::Time(0), transform);
			cv::Point P(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution);
            P += og_center_;
            if (sig_map(P)!=0){
				sig_map(P)=alpha*sig+(1-alpha)*sig_map(P);}
            else{
				sig_map(P)=sig;
				}
			
			int erosion_type = cv::MORPH_RECT ;
            int erosion_size = robot_radius_/info_.resolution ;
			cv::Mat element = cv::getStructuringElement(erosion_type,
                    cv::Size(2*erosion_size+1,2*erosion_size+1),
                    cv::Point( erosion_size, erosion_size));
            cv::dilate( sig_map, temp_map, element );

            double min, max;
            cv::minMaxLoc(temp_map, &min, &max);

            for (int i=1; i<sig_map_display.rows -1; i++){
				for(int j=1; j< sig_map_display.cols-1; j++){
					

					sig_map_display(i, j)= 255-int(temp_map(i,j)/max*255);;
				}
			}
			cv::imshow( "SigDisp", sig_map_display );
			
		}

    public:
        OccupancyGridExplorer() : nh_("~") {
			ros::Duration(1.5).sleep();
            nh_.param("robot_radius",robot_radius_,0.1);
            nh_.param("base_link",base_link_,std::string("/body"));
            nh_.param("alpha", alpha,0.5);
            og_sub_ = nh_.subscribe("occ_grid",1,&OccupancyGridExplorer::og_callback,this);
            end_sub = nh_.subscribe("/End",1,&OccupancyGridExplorer::end_callback,this);
            target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal",1,true);
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twistCommand",1);
            signal_sub= nh_.subscribe("/vrep/signal",1,&OccupancyGridExplorer::signal_callback,this);
            
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"occgrid_explorer");
    OccupancyGridExplorer ogp;
    cv::namedWindow( "OccGridRGB", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "SigDisp", CV_WINDOW_AUTOSIZE );
    while (ros::ok()) {
        ros::spinOnce();
        if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
        }
    }
}

