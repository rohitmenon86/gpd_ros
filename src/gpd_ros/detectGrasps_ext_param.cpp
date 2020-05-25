
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>

// GPD library
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

// this project (headers)
#include <gpd_ros/grasp_messages.h>
#include <gpd_ros/grasp_plotter.h>
#include <gpd_ros/detect_graspsConfig.h> // Auto-generated from cfg/ directory.

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>

// initialize param variables
struct gpd::detect_params myParam;

/*
Eigen::Vector3d direction; 
Eigen::Matrix3Xd camera_position; 
std::vector<double> workspace; 
bool approach_direction;
double thresh_rad;
*/

// get params from param server, values initialized in launch file
std::vector<double> direction;
std::vector<double> camera_position;
std::vector<double> workspace;
bool approach_direction;
double thresh_rad;
std::string point_cloud_topic;
std::string config_file;

std::string frame_; ///< point cloud frame
bool has_cloud = false;
bool cb_run = false; // prevent run in cb before initialization

gpd::util::Cloud *cloudPtr;
gpd::GraspDetector* detector;
GraspPlotter* rviz_plotter_;
std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;

void getPointCloud()
{
    //delete cloud
    delete cloudPtr; //deallocate memory
    cloudPtr = NULL; //point to NULL
    has_cloud = false;
    
    
    ros::Time request_time = ros::Time::now();
    ros::Time point_cloud_time;
    
    // Exception handling, negative time when running simulation
    try {
        point_cloud_time = request_time - ros::Duration(0.1);
    } catch (std::runtime_error& ex) {
        ROS_ERROR("Exception: [%s]", ex.what());
        point_cloud_time = request_time;
        request_time = request_time + ros::Duration(0.1);
    }

    ROS_INFO("Awaiting point cloud.");
    while (point_cloud_time < request_time)
    { 
        sensor_msgs::PointCloud2ConstPtr pc_msg = 
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>("camera/depth/points", ros::Duration(5.0));

        // get the latest point cloud
        if (pc_msg == NULL)
        {
            ROS_INFO("NO point cloud received.");
            return;
        }
        else
        {
            ROS_INFO("YES, point cloud received.");

            typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGB;
            PointCloudRGB::Ptr pc(new PointCloudRGB);
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*pc_msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *pc);

            cloudPtr = new gpd::util::Cloud(pc, 0, myParam.camera_position);
            ROS_INFO_STREAM("Received cloud with " << cloudPtr->getCloudProcessed()->size() << " points.");
            has_cloud = true;
            break;

        }
        point_cloud_time = pc_msg->header.stamp;
        frame_ = pc_msg->header.frame_id;
    }
}

void run()
{
    char input;

    while (ros::ok)
    {
        if(has_cloud){
            std::cout << "Press y to start grasp detection, n to exit and any other key to wait" << std::endl;
            std::cin >> input;

            if (input == 'y')
            {
                ROS_INFO("Starting grasping routine");
                ros::spinOnce();

                if (cloudPtr == NULL)
                {
                    ROS_INFO("No point cloud received within timeout.");
                }
                else
                {
                    ROS_INFO("Preprocessing cloud.");
                    // detect grasps
                    detector->preprocessPointCloud(*cloudPtr, myParam.workspace);
                    ROS_INFO("Detecting grasps.");
                    grasps = detector->detectGrasps(*cloudPtr, myParam);
                    ROS_INFO("Detected!");

                    std::cout << "New point cloud?"
                    "Press y for new cloud, any other key for same point cloud" << std::endl;
                    std::cin >> input;
                    if (input == 'y') {has_cloud = false;}
                }
            }
            else if (input == 'n')
            {
                ROS_INFO("Exiting");
                exit(EXIT_SUCCESS);
            }
            else
            {
                continue;
            }
        } else{
            ROS_INFO("Getting new point_cloud...");
            getPointCloud();
        }
    }
}

//dynamic configCallback
void configCallback(gpd_ros::detect_graspsConfig &config, uint32_t level)
{
    ROS_WARN("CALLBACK CALLED");
    myParam.camera_position.resize(3,1);
    myParam.workspace.resize(6);
    // modify params to type required
    myParam.direction << config.groups.direction.x, 
                         config.groups.direction.y, 
                         config.groups.direction.z;

    myParam.camera_position << config.groups.camera_position.x1, 
                               config.groups.camera_position.y1, 
                               config.groups.camera_position.z1;

    myParam.workspace.assign({config.groups.workspace.size*-1, 
                            config.groups.workspace.size*1, 
                            config.groups.workspace.size*-1, 
                            config.groups.workspace.size*1, 
                            config.groups.workspace.size*-1, 
                            config.groups.workspace.size*1});

    myParam.thresh_rad = config.thresh_rad;
    myParam.approach_direction = config.approach_direction;

    ROS_INFO("New parameters set.");
    
    char input;
    if (has_cloud)
    {
        ROS_INFO("There is already a point cloud. Press y for a NEW point cloud with new param, or any key for SAME point cloud with new param.\n");
        std::cin >> input;

        if (input == 'y') {
            getPointCloud();
        }
    }
}


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "detect_grasp_test");
    ros::NodeHandle nh;
    ros::Publisher grasps_rviz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("plot_grasps", 1);
    
    //! Dynamic reconfigure server.
    dynamic_reconfigure::Server<gpd_ros::detect_graspsConfig> dr_srv_;
    dynamic_reconfigure::Server<gpd_ros::detect_graspsConfig>::CallbackType cb;
    cb = boost::bind(&configCallback, _1, _2);
    dr_srv_.setCallback(cb);
   
    {
        nh.getParam("camera_position", camera_position);
        nh.getParam("workspace", workspace);
        nh.getParam("direction", direction);
        nh.getParam("approach_direction", approach_direction);
        nh.getParam("thresh_rad", thresh_rad);

        nh.param<std::string>("point_cloud_topic", point_cloud_topic, "camera/depth/points");
        nh.param<std::string>("config_file", config_file, "/home/atmaraaj/hrg/gpd/cfg/eigen_params.cfg");

        // Initialize param values, modify params to type required
        
        myParam.workspace.resize(6);
        myParam.camera_position.resize(3,1);

        myParam.direction       << direction[0], direction[1], direction[2];
        myParam.camera_position << camera_position[0], camera_position[1], camera_position[2];
        myParam.workspace           = workspace;
        myParam.thresh_rad          = thresh_rad;
        myParam.approach_direction  = approach_direction; 
    }
    
    detector = new gpd::GraspDetector(config_file);
    //rviz_plotter_ = new GraspPlotter(nh, detector->getHandSearchParameters().hand_geometry_);

    // run detection
    run();

    //rviz_plotter_->drawGrasps(grasps, frame_);

    // Let ROS handle all callbacks.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    return 0;
} // end main()