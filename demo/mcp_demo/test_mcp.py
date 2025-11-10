import asyncio
import os
from typing import Dict, Any
from langchain_mcp_adapters.client import MultiServerMCPClient
from langgraph.prebuilt import create_react_agent
from langchain_openai import ChatOpenAI


# 配置常量 - 从环境变量读取，提供默认值用于开发
BASE_URL = os.getenv("OPENAI_BASE_URL", "https://api.deepseek.com")
API_KEY = os.getenv("OPENAI_API_KEY", "sk-9d807dfca92d45cab7350e7a3180a365")
MODEL_NAME = os.getenv("OPENAI_MODEL", "deepseek-chat")
async def main():
    """主函数 - 运行 MCP 客户端测试"""
    try:
        # 初始化MCP客户端 stdio 方式
        client = MultiServerMCPClient({
            "File-MCP": {
                "command": "python",
                "args": [os.path.abspath("mcp_objects/file_mcp.py")],   # 文件操作MCP的路径
                "transport": "stdio"
            },
            "File-Query-MCP": {
                "command": "python",
                "args": [os.path.abspath("mcp_objects/file_query_mcp.py")],   # 文件查询操作MCP的路径
                "transport": "stdio"
            }
            # 其它MCP
        })
        
        # 获取工具
        tools = await client.get_tools()
        if not tools:
            raise ValueError("未获取到任何工具")
        
        # 初始化一个 ChatOpenAI 实例，用于与大模型交互
        llm = ChatOpenAI(
            base_url=BASE_URL,
            openai_api_key=API_KEY,
            model=MODEL_NAME,
            timeout=60.0,
            max_retries=2
        )

        # 创建agent
        agent = create_react_agent(llm, tools)
        
        while True:
            user_input = input("\n请输入需求（或输入 exit 退出）：\n> ")
            if user_input.strip().lower() == "exit":
                break
            
            # 创建日志文件，实时记录输出
            log_file_path = "mcp_test_output.log"
            with open(log_file_path, "a", encoding="utf-8") as log_file:
                log_file.write(f"\n{'='*60}\n")
                log_file.write(f"用户输入: {user_input}\n")
                log_file.write(f"时间: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                log_file.write(f"{'='*60}\n\n")
                
                async for chunk in agent.astream({"messages": user_input}):
                    print(chunk)
                    chunk_str = str(chunk)
                    if len(chunk_str) > 100:
                        log_file.write(f"长文本 ({len(chunk_str)} 字符):\n{chunk_str}\n")
                    else:
                        log_file.write(f"输出: {chunk_str}\n")
                    log_file.write("-" * 40 + "\n")
                    log_file.flush()  # 确保实时写入
                
                log_file.write(f"\n{'='*60}\n")
                log_file.write("本次会话结束\n")
                log_file.write(f"{'='*60}\n\n")
            
            print(f"\n输出已格式化保存到: {log_file_path}")
                
    except Exception as e:
        print(f"程序初始化失败: {e}")
if __name__ == "__main__":
    asyncio.run(main())
