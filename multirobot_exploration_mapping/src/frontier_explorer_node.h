#ifndef FRONTIER_EXPLORER_NODE_H
#define FRONTIER_EXPLORER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// Eylem istemcisi için bir tür takma adı tanımlıyoruz.
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @class FrontierExplorerNode
 * @brief Çoklu robotla otonom keşif görevini yöneten sınıf.
 *
 * Bu sınıf, bir haritayı dinler, keşfedilmemiş sınırları tespit eder,
 * bu sınırları kümeleyerek hedef noktalar oluşturur ve bu hedefleri
 * mevcut robotlara atar.
 */
class FrontierExplorerNode
{
public:
    // Kurucu fonksiyon, ROS düğüm tanıtıcısını alır.
    FrontierExplorerNode(ros::NodeHandle &nh);
    
    // Keşif sürecini başlatan ana döngü.
    void runExplorationLoop();

private:
    // Gelen harita verilerini işleyen callback fonksiyonu.
    void handleMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    // Boştaki robotlara yeni hedefler (sınırlar) atayan ana mantık fonksiyonu.
    void dispatchTasksToRobots();

    // Harita üzerinde keşfedilmemiş alanların kenarlarını (sınırları) bulan fonksiyon.
    std::vector<geometry_msgs::Point> identifyExplorationBoundaries();

    // Bulunan sınır noktalarını, birbirlerine olan yakınlıklarına göre gruplayan fonksiyon.
    std::vector<geometry_msgs::Point> groupFrontierPoints(const std::vector<geometry_msgs::Point> &frontiers, double clustering_radius);

    // Belirtilen index'teki robotun anlık konumunu TF (Transform) kütüphanesi ile sorgular.
    geometry_msgs::Point queryRobotPose(int robot_index);

    // Bir robot hedefine ulaştığında veya ulaşamadığında tetiklenen callback fonksiyonu.
    void onGoalCompletion(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result, int robot_index);

    // ROS Düğüm Tanıtıcısı
    ros::NodeHandle nodeHandle_;
    // Harita verilerini dinleyen abone
    ros::Subscriber mapSub_;

    // Toplam robot sayısı
    int robotCount_;
    // Her robot için MoveBase eylem istemcileri
    std::vector<MoveBaseClient *> moveBaseClients_;
    // Robotların meşgul olup olmadığını tutan bayraklar
    std::vector<bool> isRobotActive_;
    // Robotların isim alanları (örn: "tb3_0", "tb3_1")
    std::vector<std::string> robotNamespaces_;
    // Her robota atanmış olan mevcut hedef noktaları
    std::vector<geometry_msgs::Point> activeGoals_;
    // Ulaşılamayan veya başarısız olan hedeflerin listesi
    std::vector<geometry_msgs::Point> failedGoalPoints_;

    // En son alınan harita verisi
    nav_msgs::OccupancyGrid latestMapData_;
    // Haritanın alınıp alınmadığını belirten bayrak
    bool isMapReady_ = false;

    // TF dönüşümleri için buffer ve listener
    tf2_ros::Buffer transformBuffer_;
    tf2_ros::TransformListener transformListener_;
};

#endif // FRONTIER_EXPLORER_NODE_H