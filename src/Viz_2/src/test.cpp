#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher");// Инициализируем узел ROS и имя узла
    ros::NodeHandle n;                    // Создаем дескриптор узла
    ros::Rate loop_rate(100);             // Контроль частоты работы узла, используется вместе с loop.sleep

    tf::TransformBroadcaster broadcaster; // Создаем tf Broadcaster
    
    tf::Transform base2laser;
    tf::Quaternion q;
    q.setRPY(0,0,0);
    base2laser.setRotation(q);              // Устанавливаем координаты вращения
    base2laser.setOrigin(tf::Vector3(1,0,0));// Устанавливаем координаты смещения, лазер находится в позиции (1,0,0) базы

    while (n.ok())
    {
        // Отпускаем преобразование координат двумя способами
        broadcaster.sendTransform(tf::StampedTransform(base2laser,ros::Time::now(),"base_link","laser_link"));
        //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(1, 0.0, 0)),ros::Time::now(),"base_link", "base_laser"));
        loop_rate.sleep();
    }
    return 0;
}
