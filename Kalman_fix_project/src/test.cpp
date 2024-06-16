#include "rclcpp/rclcpp.hpp"
#include "kalman/kalman.hpp"
#include "rclcpp/callback_group.hpp"

#include "atomic"

#include "geometry_msgs/msg/vector3.hpp"
#include <random>

//定义时，为分频模式
#define Frequency

//生成高斯噪音
double generateGaussianNoise(double mean, double std_dev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> distr(mean, std_dev);
    return distr(gen);
}

class Simulator_CV : public rclcpp::Node {
public:
    Simulator_CV(std::string name) : Node(name) {
        Kalman_fillter_CV = new Kalman(0.01);
        Kalman_fillter_CV->Q_set(100);
        Kalman_fillter_CV->R_set(0.001);


        //同一个组别，类型设置为Reentrant
        // callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        //用于发送原始数据
        raw_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("raw", 1);
        //用于发送kalman滤波后的数据
        fillted_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("fillted", 1);

        //用于kalman运行的定时器，和云台输出频率一致
        kalman_timer_ = create_wall_timer(std::chrono::microseconds(100000),
                                          [this] { timer_cb(); }, callback_group_);
        //用于生成原始数据的定时器
        simulator_timer_ = create_wall_timer(std::chrono::microseconds(10000),
                                             [this] { simulator_cb(); }, callback_group_);

        RCLCPP_INFO(this->get_logger(), "节点%s已启动", name.c_str());
    }

private:
    Kalman *Kalman_fillter_CV;

    rclcpp::CallbackGroup::SharedPtr callback_group_;


    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr raw_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fillted_publisher_;


    rclcpp::TimerBase::SharedPtr kalman_timer_;
    rclcpp::TimerBase::SharedPtr simulator_timer_;

    double simulate_pos_x{};


    std::atomic<bool> isunsuccessful = true;

    void timer_cb() {
        RCLCPP_INFO(this->get_logger(), "kalman");
        static int cnt = 0;

        if (!isunsuccessful) {
            Eigen::Matrix<double, 1, 1> meassure;
            meassure << simulate_pos_x;


            if (cnt) {
                Kalman_fillter_CV->predict();
                Eigen::Vector3d CA_fillter_output = Kalman_fillter_CV->update(meassure);
                geometry_msgs::msg::Vector3 fillted;
                fillted.x = CA_fillter_output(0);
                fillted.y = CA_fillter_output(1);
                fillted.z = CA_fillter_output(2);
                fillted_publisher_->publish(fillted);
            } else {

                auto output = Kalman_fillter_CV->predict();
                geometry_msgs::msg::Vector3 fillted;
                fillted.x = output(0);
                fillted.y = output(1);
                fillted.z = 0;
                fillted_publisher_->publish(fillted);
            }
#ifdef Frequency
            (++cnt) %= 10;
#endif
            isunsuccessful = true;
        }
    }


    //Constant velocity Moldel
    void simulator_cb() {
        static double pos_x = 0;
        static double vec_x = 1.0;
        static double T = 0.01;

        static uint16_t error_inject_cnt;

        RCLCPP_INFO(this->get_logger(), "simulator");
        if (pos_x > 2) {
            vec_x = -1.0;
        } else if (pos_x < -2) {
            vec_x = 1.0;
        }

        pos_x = pos_x + vec_x * T;

        simulate_pos_x = pos_x + generateGaussianNoise(0, 0.05);

        if (error_inject_cnt > 150) //故障注入
        {
            simulate_pos_x += 2; //注入1m错误
            error_inject_cnt = 0;
        }
        error_inject_cnt++;

        isunsuccessful = false;

        geometry_msgs::msg::Vector3 raw;
        raw.x = simulate_pos_x;
        raw.y = vec_x;
        raw.z = 0;
        raw_publisher_->publish(raw);
    }
};

//
// class Simulator_CA : public rclcpp::Node {
// public:
//     Simulator_CA(std::string name) : Node(name) {
//         Kalman_fillter_CA = new Kalman(0.01);
//         Kalman_fillter_CA->Q_set(100);
//         Kalman_fillter_CA->R_set(0.001);
//
//
//         //同一个组别，类型设置为Reentrant
//         // callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
//         callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//         //用于发送原始数据
//         raw_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("raw", 1);
//         //用于发送kalman滤波后的数据
//         fillted_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("fillted", 1);
//
//         //用于kalman运行的定时器，和云台输出频率一致
//         kalman_timer_ = create_wall_timer(std::chrono::microseconds(10000),
//                                           [this] { timer_cb(); }, callback_group_);
//         //用于生成原始数据的定时器
//         simulator_timer_ = create_wall_timer(std::chrono::microseconds(10000),
//                                              [this] { simulator_cb(); }, callback_group_);
//
//         RCLCPP_INFO(this->get_logger(), "节点%s已启动", name.c_str());
//     }
//
// private:
//     Kalman *Kalman_fillter_CA;
//
//     rclcpp::CallbackGroup::SharedPtr callback_group_;
//
//
//     rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr raw_publisher_;
//     rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr fillted_publisher_;
//
//
//     rclcpp::TimerBase::SharedPtr kalman_timer_;
//     rclcpp::TimerBase::SharedPtr simulator_timer_;
//
//     double simulate_pos_x{};
//
//
//     std::atomic<bool> isunsuccessful = true;
//
//     void timer_cb() {
//         RCLCPP_INFO(this->get_logger(), "kalman");
//
//
//         if (!isunsuccessful) {
//             Eigen::Matrix<double, 1, 1> meassure;
//             meassure << simulate_pos_x;
//
//             Kalman_fillter_CA->predict();
//
//             Eigen::Vector3d CA_fillter_output = Kalman_fillter_CA->update(meassure);
//
//
//             geometry_msgs::msg::Vector3 fillted;
//             fillted.x = CA_fillter_output(0);
//             fillted.y = CA_fillter_output(1);
//             fillted.z = CA_fillter_output(2);
//
//             fillted_publisher_->publish(fillted);
//             isunsuccessful = true;
//         }
//     }
//
//
//     //测试直线运动
//     void simulator_cb() {
//         static double pos_x = 0;
//         static double vec_x = 0;
//         static double acc_x = 0.5;
//         static double T = 0.01;
//
//         static uint16_t error_inject_cnt;
//
//         RCLCPP_INFO(this->get_logger(), "simulator");
//         if (vec_x > 2) {
//             acc_x = -0.5;
//         } else if (vec_x < -2) {
//             acc_x = 0.5;
//         }
//         vec_x += acc_x * T;
//         pos_x = pos_x + vec_x * T + 0.5 * acc_x * T * T;
//
//
//         simulate_pos_x = pos_x + generateGaussianNoise(0, 0.05);
//
//         if (error_inject_cnt > 150) //故障注入
//         {
//             // simulate_pos_x += 2;//注入1m错误
//             error_inject_cnt = 0;
//         }
//         error_inject_cnt++;
//
//         isunsuccessful = false;
//
//         geometry_msgs::msg::Vector3 raw;
//         raw.x = simulate_pos_x;
//         raw.y = vec_x;
//         raw.z = acc_x;
//         raw_publisher_->publish(raw);
//     }
// };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulator_CV>("test");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
