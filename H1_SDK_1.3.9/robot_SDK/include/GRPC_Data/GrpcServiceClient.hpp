#pragma once

#include <string>
#include <grpcpp/grpcpp.h>
#include "ip_service.pb.h"
#include "ip_service.grpc.pb.h"

/**
 * @brief gRPC客户端管理类，负责向服务端发送本机IP地址
 */
class GrpcServiceClient {
public:
    /**
     * @brief 构造函数，初始化gRPC客户端
     * @param server_address 服务端地址（如 "192.168.110.251:50052"）
     */
    explicit GrpcServiceClient(const std::string& server_address);

    /**
     * @brief 发送本机IP地址到服务端
     * @param ip 本机IP地址字符串
     * @return 服务端返回的处理结果
     */
    std::string SendIp(const std::string& ip);
    /** @brief 获取本机IP地址
     *
     * @return 本机IP地址字符串
     */
    std::string get_local_ip();


    /**
     * @brief 获取本机所有非回环IPv4地址（有线和无线）
     * @return 包含所有IP的字符串数组
     */
    std::vector<std::string> get_all_local_ips();
private:
    std::unique_ptr<ipservice::IpService::Stub> stub_;  // gRPC存根对象
};