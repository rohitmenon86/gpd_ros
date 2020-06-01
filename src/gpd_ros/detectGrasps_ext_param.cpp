
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include<tf2_ros/buffer.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

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

// get params from param server, values initialized in launch file
int direction;
std::vector<double> camera_position;
std::vector<double> gpd_workspace_;
bool approach_direction;
double thresh_rad, ws_height;
std::string point_cloud_topic;
std::string config_file;
std::string grasp_frame_id = "camera_depth_optical_frame";
geometry_msgs::Point point1, point2, point3;
visualization_msgs::Marker cube_lines;

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

// point-point3 make the diagonal, point1-left, point2-right
void getFourthPoint_NormalVector(geometry_msgs::Point& point4, double cross_P[], const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const geometry_msgs::Point& point3) {

    // p4 = p1 to p3 diagonal midpoint + (vector of p2 to diagonal midpoint)
    double x_m  = point1.x + (point3.x - point1.x)/2;
    double y_m  = point1.y + (point3.y - point1.y)/2;
    double z_m  = point1.z + (point3.z - point1.z)/2;
    point4.x = x_m + (x_m - point2.x);
    point4.y = y_m + (y_m - point2.y);
    point4.z = z_m + (z_m - point2.z);

    // normal vector = cross product of two vector (3 points)
    double vect_B[3] = {point1.x - point2.x, point1.y - point2.y, point1.z - point2.z}; // p2 to p1
    double vect_A[3] = {point3.x - point2.x, point3.y - point2.y, point3.z - point2.z}; // p2 to p3

    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]; 
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2]; 
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]; 

    // make unit vector of normal
    double magnitude = sqrt(pow(cross_P[0], 2)) + sqrt(pow(cross_P[1], 2)) + sqrt(pow(cross_P[2], 2));
    for(int i = 0; i < 3; i++) {cross_P[i] = cross_P[i] / magnitude;}
}

void transformPosition(const geometry_msgs::Point& pose_in, geometry_msgs::Point& pose_out, std::string source_frame, std::string target_frame)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped source2target;
    ros::Duration timeout(1.0);
    
    try {
        source2target = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), timeout);
    } catch (tf2::LookupException& e) {
        std::cout<<e.what()<<endl<<"Trying again with 2s timeout."<<endl;
        source2target = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2.0) );
    }
    ROS_INFO_STREAM("source2target: "<<source2target<<"\n");
    tf2::doTransform(pose_in,  pose_out, source2target);
}

// receives three points on a plane and height along plane
// point1-point3 make the diagonal
void setGPDGraspWorkspace(const double& height, const geometry_msgs::Point& point1, 
    const geometry_msgs::Point& point2, const geometry_msgs::Point& point3)
{
    double offset = 0.05;
    std::vector< geometry_msgs::Point > workspace_(8);
    double normal_unit[3];
    
    // Vertices of workspace
    workspace_[0] = point1;
    workspace_[0].z += offset;
    
    workspace_[1] = point2;
    workspace_[1].z += offset;
    
    workspace_[2] = point3;    
    workspace_[2].z += offset;
    
    workspace_[3] = point1; //initiate
    getFourthPoint_NormalVector( workspace_[3], normal_unit, point1, point2, point3);
    workspace_[3].z += offset;
    
    workspace_[4].x = workspace_[0].x + height * normal_unit[0];
    workspace_[4].y = workspace_[0].y + height * normal_unit[1];
    workspace_[4].z = workspace_[0].z + height * normal_unit[2];

    workspace_[5].x = workspace_[1].x + height * normal_unit[0];
    workspace_[5].y = workspace_[1].y + height * normal_unit[1];
    workspace_[5].z = workspace_[1].z + height * normal_unit[2];

    workspace_[6].x = workspace_[2].x + height * normal_unit[0];
    workspace_[6].y = workspace_[2].y + height * normal_unit[1];
    workspace_[6].z = workspace_[2].z + height * normal_unit[2];

    workspace_[7].x = workspace_[3].x + height * normal_unit[0];
    workspace_[7].y = workspace_[3].y + height * normal_unit[1];
    workspace_[7].z = workspace_[3].z + height * normal_unit[2];
    
    // Make cube in RViz
    cube_lines.header.stamp     = ros::Time::now();
    cube_lines.points.clear();
    cube_lines.points.push_back(workspace_[0]);
    cube_lines.points.push_back(workspace_[1]);
    cube_lines.points.push_back(workspace_[2]);
    cube_lines.points.push_back(workspace_[3]);
    cube_lines.points.push_back(workspace_[0]);
    cube_lines.points.push_back(workspace_[3]);
    cube_lines.points.push_back(workspace_[7]);
    cube_lines.points.push_back(workspace_[4]);
    cube_lines.points.push_back(workspace_[5]);
    cube_lines.points.push_back(workspace_[6]);
    cube_lines.points.push_back(workspace_[7]);
    cube_lines.points.push_back(workspace_[4]);
    cube_lines.points.push_back(workspace_[0]);
    cube_lines.points.push_back(workspace_[1]);
    cube_lines.points.push_back(workspace_[5]);
    cube_lines.points.push_back(workspace_[6]);
    cube_lines.points.push_back(workspace_[2]);

    std::vector<geometry_msgs::Point> workspace_camera_frame(8);
    std::vector<double> x, y, z;
    x.resize(8);
    y.resize(8);
    z.resize(8);
    
    // transform ws from world frame to camera_optical_depth_frame
    for(short i = 0; i < 8; ++i)
    {
        transformPosition(workspace_[i], workspace_camera_frame[i], "world", grasp_frame_id);
        x[i] = workspace_camera_frame[i].x;
        y[i] = workspace_camera_frame[i].y;
        z[i] = workspace_camera_frame[i].z;
    }
    
    gpd_workspace_.resize(6);
    gpd_workspace_[0] = *std::min_element(x.begin(), x.end());
    gpd_workspace_[1] = *std::max_element(x.begin(), x.end());
    
    gpd_workspace_[2] = *std::min_element(y.begin(), y.end());
    gpd_workspace_[3] = *std::max_element(y.begin(), y.end());
    
    gpd_workspace_[4] = *std::min_element(z.begin(), z.end());
    gpd_workspace_[5] = *std::max_element(z.begin(), z.end());
    
    if (ros::param::has("/detect_grasps/workspace"))
    {
        ros::param::set("/detect_grasps/workspace", gpd_workspace_);
    }

}

void init_param(ros::NodeHandle& nh) {
    nh.getParam("camera_position", camera_position);
    nh.getParam("direction", direction);
    nh.getParam("approach_direction", approach_direction);
    nh.getParam("thresh_rad", thresh_rad);
    nh.getParam("height", ws_height);
    
    std::vector<double> temp;
    nh.getParam("point1", temp);
    nh.setParam("point1_x", temp[0]);
    nh.setParam("point1_y", temp[1]);
    nh.setParam("point1_z", temp[2]);
    point1.x = temp[0];
    point1.y = temp[1];
    point1.z = temp[2];
    temp.clear();

    nh.getParam("point2", temp);
    nh.setParam("point2_x", temp[0]);
    nh.setParam("point2_y", temp[1]);
    nh.setParam("point2_z", temp[2]);
    point2.x = temp[0];
    point2.y = temp[1];
    point2.z = temp[2];
    temp.clear();

    nh.getParam("point3", temp);
    nh.setParam("point3_x", temp[0]);
    nh.setParam("point3_y", temp[1]);
    nh.setParam("point3_z", temp[2]);
    point3.x = temp[0];
    point3.y = temp[1];
    point3.z = temp[2];
    temp.clear();

    nh.param<std::string>("point_cloud_topic", point_cloud_topic, "camera/depth/points");
    nh.param<std::string>("config_file", config_file, "/home/atmaraaj/hrg/gpd/cfg/eigen_params.cfg");

    // Initialize param values, modify params to type required
    myParam.workspace.resize(6);
    myParam.camera_position.resize(3,1);

    setGPDGraspWorkspace(ws_height, point1, point2, point3);
    myParam.workspace           = gpd_workspace_;
    myParam.thresh_rad          = thresh_rad;
    myParam.approach_direction  = approach_direction;
    myParam.camera_position     << camera_position[0], camera_position[1], camera_position[2];

    if      (direction == 0) {myParam.direction << 1, 0, 0;}
    else if (direction == 1) {myParam.direction << 0, 1, 0;}
    else if (direction == 2) {myParam.direction << 0, 0, 1;}
    else {myParam.direction << 1, 0, 0;}
     
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
void configCallback(gpd_ros::detect_graspsConfig &config, uint32_t level, ros::Publisher* ws_pub)
{
    ROS_WARN("CALLBACK CALLED");
    myParam.camera_position.resize(3,1);
    myParam.workspace.resize(6);
    // modify params to type required

    direction = config.direction;
    if      (direction == 0) {myParam.direction << 1, 0, 0;}
    else if (direction == 1) {myParam.direction << 0, 1, 0;}
    else if (direction == 2) {myParam.direction << 0, 0, 1;}
    
    myParam.camera_position << config.groups.camera_position.x1, 
                               config.groups.camera_position.y1, 
                               config.groups.camera_position.z1;

    point1.x = config.groups.workspace.point1_x;
    point1.y = config.groups.workspace.point1_y;
    point1.z = config.groups.workspace.point1_z;
    point2.x = config.groups.workspace.point2_x;
    point2.y = config.groups.workspace.point2_y;
    point2.z = config.groups.workspace.point2_z;
    point3.x = config.groups.workspace.point3_x;
    point3.y = config.groups.workspace.point3_y;
    point3.z = config.groups.workspace.point3_z;
    ws_height = config.groups.workspace.height;

    setGPDGraspWorkspace(ws_height, point1, point2, point3);
    myParam.workspace = gpd_workspace_;
    myParam.thresh_rad = config.thresh_rad;
    myParam.approach_direction = config.approach_direction;

    ROS_INFO("New parameters set.");

    // Publish updated worspace cube lines
    ws_pub->publish(cube_lines);

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
    ros::Publisher grasps_rviz_pub_  = nh.advertise<visualization_msgs::MarkerArray>("plot_grasps", 1);
    ros::Publisher workspacecube_pub = nh.advertise<visualization_msgs::Marker>("workspace_cube", 1);
    
    //! Dynamic reconfigure server.
    dynamic_reconfigure::Server<gpd_ros::detect_graspsConfig> dr_srv_;
    dynamic_reconfigure::Server<gpd_ros::detect_graspsConfig>::CallbackType cb;
    cb = boost::bind(&configCallback, _1, _2, &workspacecube_pub);
    dr_srv_.setCallback(cb);
    
    // Initialize makers line 
    cube_lines.header.frame_id  = "/world";
    cube_lines.ns               = "detect_grasp_test";
    cube_lines.action           = visualization_msgs::Marker::ADD;
    cube_lines.pose.orientation.w = 1.0;
    cube_lines.id               = 0;
    cube_lines.type             = visualization_msgs::Marker::LINE_STRIP;
    cube_lines.scale.x          = 0.1;
    cube_lines.color.r          = 1.0;
    cube_lines.color.a          = 0.6;

    // Assign param values
    init_param(nh);

    workspacecube_pub.publish(cube_lines);

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