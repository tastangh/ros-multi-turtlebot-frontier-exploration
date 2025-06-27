#include "frontier_explorer_node.h"

// Sınıfın kurucu fonksiyonu
FrontierExplorerNode::FrontierExplorerNode(ros::NodeHandle &nh) : nodeHandle_(nh), transformListener_(transformBuffer_)
{
    // Parametre sunucusundan robot sayısını oku, varsayılan değer 4
    nodeHandle_.param("num_robots", robotCount_, 4);

    // Robot isimlerini ve ilgili yapıları boyutlandır
    robotNamespaces_.resize(robotCount_);
    for (int i = 0; i < robotCount_; ++i)
    {
        robotNamespaces_[i] = "tb3_" + std::to_string(i);
    }

    // Harita topic'ine abone ol
    mapSub_ = nodeHandle_.subscribe("/map", 1, &FrontierExplorerNode::handleMapUpdate, this);

    // Robotların durumlarını ve eylem istemcilerini başlat
    isRobotActive_.resize(robotCount_, false);
    moveBaseClients_.resize(robotCount_);
    activeGoals_.resize(robotCount_);

    for (int i = 0; i < robotCount_; ++i)
    {
        std::string serverName = "/" + robotNamespaces_[i] + "/move_base";
        moveBaseClients_[i] = new MoveBaseClient(serverName, true);

        ROS_INFO("Robot %d icin eylem sunucusu bekleniyor: %s", i, serverName.c_str());
        if (!moveBaseClients_[i]->waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("HATA: Robot %d icin eylem sunucusu %s bulunamadi!", i, serverName.c_str());
        }
        else
        {
            ROS_INFO("Robot %d icin eylem sunucusu baglantisi basarili.", i);
        }
    }
}

// Keşif döngüsünü çalıştıran ana fonksiyon
void FrontierExplorerNode::runExplorationLoop()
{
    ros::Rate loopRate(1.0); // Döngü frekansını 1 Hz olarak ayarla
    while (ros::ok())
    {
        dispatchTasksToRobots();
        ros::spinOnce();
        loopRate.sleep();
    }
}

// Harita güncellemesini işleyen fonksiyon
void FrontierExplorerNode::handleMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    latestMapData_ = *msg;
    if (!isMapReady_)
    {
        isMapReady_ = true;
        ROS_INFO_ONCE("Ilk harita verisi alindi. Kesif operasyonu basliyor...");
    }
}

// Bir robotun görev tamamlama durumunu işleyen callback
void FrontierExplorerNode::onGoalCompletion(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result, int robot_index)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Robot %d gorevini basariyla tamamladi.", robot_index);
    }
    else
    {
        ROS_WARN("Robot %d hedefine ulasamadi. Durum: %s", robot_index, state.toString().c_str());
        // Başarısız hedefi kara listeye ekle
        failedGoalPoints_.push_back(activeGoals_[robot_index]);
        ROS_INFO("Basarisiz hedef [x:%.2f, y:%.2f] kara listeye eklendi.", activeGoals_[robot_index].x, activeGoals_[robot_index].y);
    }
    // Robotu tekrar "boşta" olarak işaretle
    isRobotActive_[robot_index] = false;
}

// Robotlara yeni görevler (hedefler) atayan merkezi fonksiyon
void FrontierExplorerNode::dispatchTasksToRobots()
{
    if (!isMapReady_)
    {
        ROS_INFO_THROTTLE(5, "Harita bekleniyor, hedef atama ertelendi.");
        return;
    }

    auto boundaryPoints = identifyExplorationBoundaries();
    if (boundaryPoints.empty())
    {
        bool isAnyRobotActive = false;
        for (const auto &status : isRobotActive_)
        {
            if (status) isAnyRobotActive = true;
        }

        if (!isAnyRobotActive)
        {
            ROS_INFO("Tum robotlar bosta ve yeni sinir noktasi bulunamadi. Kesif tamamlaniyor.");
            ros::shutdown();
        }
        else
        {
            ROS_INFO_THROTTLE(10, "Yeni sinir noktasi yok, aktif robotlarin gorevlerini bitirmesi bekleniyor.");
        }
        return;
    }

    auto frontierCentroids = groupFrontierPoints(boundaryPoints, 1.0);

    for (int i = 0; i < robotCount_; ++i)
    {
        if (!isRobotActive_[i])
        {
            geometry_msgs::Point currentRobotPose = queryRobotPose(i);
            if (!std::isfinite(currentRobotPose.x)) continue;

            double closestDistance = std::numeric_limits<double>::max();
            geometry_msgs::Point optimalTarget;
            bool targetFound = false;

            for (const auto &centroid : frontierCentroids)
            {
                bool isBlacklisted = false;
                for (const auto &failed_pt : failedGoalPoints_)
                {
                    if (std::hypot(centroid.x - failed_pt.x, centroid.y - failed_pt.y) < 1.0)
                    {
                        isBlacklisted = true;
                        break;
                    }
                }
                if (isBlacklisted) continue;

                bool isAlreadyAssigned = false;
                for (int j = 0; j < robotCount_; ++j)
                {
                    if (isRobotActive_[j] && std::hypot(centroid.x - activeGoals_[j].x, centroid.y - activeGoals_[j].y) < 0.5)
                    {
                        isAlreadyAssigned = true;
                        break;
                    }
                }
                if (isAlreadyAssigned) continue;

                double distanceToTarget = std::hypot(currentRobotPose.x - centroid.x, currentRobotPose.y - centroid.y);
                if (distanceToTarget < closestDistance)
                {
                    closestDistance = distanceToTarget;
                    optimalTarget = centroid;
                    targetFound = true;
                }
            }

            if (targetFound)
            {
                move_base_msgs::MoveBaseGoal goalMsg;
                goalMsg.target_pose.header.frame_id = "map";
                goalMsg.target_pose.header.stamp = ros::Time::now();
                goalMsg.target_pose.pose.position = optimalTarget;
                goalMsg.target_pose.pose.orientation.w = 1.0;

                // Modern C++ lambda kullanarak hedefi gönder ve callback'i bağla
                moveBaseClients_[i]->sendGoal(goalMsg, 
                    [this, i](const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
                        this->onGoalCompletion(state, result, i);
                    });

                isRobotActive_[i] = true;
                activeGoals_[i] = optimalTarget;
                ROS_INFO("Robot %d icin yeni gorev atandi: [x: %.2f, y: %.2f]", i, optimalTarget.x, optimalTarget.y);
            }
        }
    }
}

// Bir robotun mevcut konumunu TF sisteminden sorgular
geometry_msgs::Point FrontierExplorerNode::queryRobotPose(int robot_index)
{
    geometry_msgs::TransformStamped transform;
    geometry_msgs::Point position;
    try
    {
        std::string robotFrame = robotNamespaces_[robot_index] + "/base_footprint";
        transform = transformBuffer_.lookupTransform("map", robotFrame, ros::Time(0));
        position.x = transform.transform.translation.x;
        position.y = transform.transform.translation.y;
        position.z = 0;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Robot %d icin konum donusumu alinamadi: %s", robot_index, ex.what());
        position.x = std::numeric_limits<double>::infinity();
        position.y = std::numeric_limits<double>::infinity();
    }
    return position;
}

// Haritadaki bilinmeyen ve bilinen alanlar arasındaki sınırları tespit eder
std::vector<geometry_msgs::Point> FrontierExplorerNode::identifyExplorationBoundaries()
{
    std::vector<geometry_msgs::Point> boundaryPoints;
    if (!isMapReady_) return boundaryPoints;

    const auto &gridData = latestMapData_.data;
    const auto &mapMetadata = latestMapData_.info;
    const unsigned int mapWidth = mapMetadata.width;
    const unsigned int mapHeight = mapMetadata.height;

    // Harita hücreleri üzerinde iterasyon yap (kenarları atlayarak)
    for (unsigned int y = 1; y < mapHeight - 1; ++y)
    {
        for (unsigned int x = 1; x < mapWidth - 1; ++x)
        {
            // Hücre boş (0) değilse atla
            if (gridData[y * mapWidth + x] != 0) continue;

            bool hasUnknownNeighbor = false;
            // 8 komşuyu kontrol et
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 && dy == 0) continue;
                    
                    if (gridData[(y + dy) * mapWidth + (x + dx)] == -1) // Bilinmeyen komşu
                    {
                        hasUnknownNeighbor = true;
                        break;
                    }
                }
                if (hasUnknownNeighbor) break;
            }

            if (hasUnknownNeighbor)
            {
                geometry_msgs::Point p;
                p.x = mapMetadata.origin.position.x + (x + 0.5) * mapMetadata.resolution;
                p.y = mapMetadata.origin.position.y + (y + 0.5) * mapMetadata.resolution;
                p.z = 0;
                boundaryPoints.push_back(p);
            }
        }
    }
    return boundaryPoints;
}

// Sınır noktalarını belirli bir yarıçap içinde kümeleyerek merkezlerini bulur
std::vector<geometry_msgs::Point> FrontierExplorerNode::groupFrontierPoints(const std::vector<geometry_msgs::Point> &frontiers, double clustering_radius)
{
    std::vector<geometry_msgs::Point> clusterCenters;
    if (frontiers.empty()) return clusterCenters;

    std::vector<bool> processed(frontiers.size(), false);

    for (size_t i = 0; i < frontiers.size(); ++i)
    {
        if (processed[i]) continue;

        std::vector<geometry_msgs::Point> currentCluster;
        std::vector<size_t> processingQueue;
        
        processingQueue.push_back(i);
        processed[i] = true;

        size_t head = 0;
        while(head < processingQueue.size())
        {
            size_t currentIndex = processingQueue[head++];
            currentCluster.push_back(frontiers[currentIndex]);

            for (size_t j = 0; j < frontiers.size(); ++j)
            {
                if (processed[j]) continue;

                double dist = std::hypot(frontiers[currentIndex].x - frontiers[j].x, frontiers[currentIndex].y - frontiers[j].y);
                if (dist < clustering_radius)
                {
                    processed[j] = true;
                    processingQueue.push_back(j);
                }
            }
        }

        if (!currentCluster.empty())
        {
            geometry_msgs::Point center;
            for (const auto &p : currentCluster)
            {
                center.x += p.x;
                center.y += p.y;
            }
            center.x /= currentCluster.size();
            center.y /= currentCluster.size();
            clusterCenters.push_back(center);
        }
    }
    return clusterCenters;
}

// Ana program başlangıç noktası
int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_explorer_node");
    ros::NodeHandle nh("~");

    FrontierExplorerNode explorer(nh);
    explorer.runExplorationLoop();

    return 0;
}