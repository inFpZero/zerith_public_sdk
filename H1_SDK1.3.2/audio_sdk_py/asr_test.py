import sys
import os
# 把本目录下的 lib 添加到 sys.path，确保能导入编译后的 .so 模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "lib"))
import grpc
import robot_pb2
import robot_pb2_grpc


def run():
    channel = grpc.insecure_channel('localhost:50051')  # 本机可用 'localhost:50051' ，可支持局域网通信，ip:50051
    stub = robot_pb2_grpc.ASRServiceStub(channel)

    # 构造识别配置,来自讯飞服务官网
    config = robot_pb2.RecognitionConfig(
        app_id="替换为你实际使用的app_id",    # 应用 ID 
        api_key="替换为你实际使用的api_key",   # API Key
        api_secret="替换为你实际使用的api_secret",  # API Secret
        device=-1  # 替换为实际设备索引（可选，-1表示默认设备,即机器人本体的环形六麦）
    )
    config.business_args.update({   #具体参数含义参考SDK文档
    "domain": "iat", 
    "language": "zh_cn", 
    "accent": "mandarin",
    "vinfo": "1", 
    "vad_eos": "3000", 
    "pcm": "1", 
    "nunum": "1",
    "dwa": "wpgs"
    })

    # 启动识别并流式获取结果
    responses = stub.StartRecognition(config)
    for res in responses:
        if res.error:
            print("识别出错:", res.error)
        elif res.is_final:
            print("完整识别结果:", res.text)
        else:
            print("流式识别结果:", res.text)

if __name__ == '__main__':
    run()