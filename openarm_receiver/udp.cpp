#include <iostream>
#include <string>
#include <vector>
#include <asio.hpp>
#include <nlohmann/json.hpp>

using asio::ip::udp;
using json = nlohmann::json;

// 定义与 Python 端对应的骨骼姿态结构
struct PoseData {
    std::vector<double> tran;                  // 根位移 [x, y, z]
    std::vector<std::vector<double>> pose24;   // 24个骨骼的四元数 [w, x, y, z]
    int static_index;
    double timestamp;
};

class UdpServer {
public:
    UdpServer(asio::io_context& io_context, short port)
        : socket_(io_context, udp::endpoint(udp::v4(), port)) {
        std::cout << "UDP Server 已启动，监听端口: " << port << std::endl;
        start_receive();
    }

private:
    void start_receive() {
        // 异步等待数据
        socket_.async_receive_from(
            asio::buffer(recv_buffer_), remote_endpoint_,
            [this](std::error_code ec, std::size_t bytes_recvd) {
                if (!ec && bytes_recvd > 0) {
                    process_data(bytes_recvd);
                }
                start_receive(); // 继续监听下一条消息
            });
    }

    void process_data(std::size_t length) {
        try {
            // 1. 将接收到的字节转换为字符串
            std::string msg(recv_buffer_.data(), length);
            
            // 2. 解析 JSON
            json j = json::parse(msg);

            // 3. 映射到 C++ 结构体
            PoseData pd;
            pd.tran = j.at("tran").get<std::vector<double>>();
            pd.pose24 = j.at("pose24").get<std::vector<std::vector<double>>>();
            pd.static_index = j.at("static_index").get<int>();
            pd.timestamp = j.at("timestamp").get<double>();

            // 4. 打印部分数据验证（例如根位移和第一个关节的四元数）
            std::cout << "[" << pd.timestamp << "] 收到来自 " << remote_endpoint_ << " 的数据" << std::endl;
            std::cout << "Root Trans: x=" << pd.tran[0] << " y=" << pd.tran[1] << " z=" << pd.tran[2] << std::endl;
            if(!pd.pose24.empty()) {
                auto q = pd.pose24[13]; // 第一个关节
                std::cout << "Joint[0] Quat: w=" << q[0] << " x=" << q[1] << std::endl;
            }
            std::cout << "---------------------------------------" << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "解析错误: " << e.what() << std::endl;
        }
    }

    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    std::array<char, 65535> recv_buffer_; // UDP 最大包长度
};

int main() {
    try {
        asio::io_context io_context;
        UdpServer server(io_context, 5678);
        io_context.run(); // 运行异步事件循环
    } catch (std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
    }
    return 0;
}