import os
from typing import Dict, Any
from fastmcp import FastMCP

# 初始化 MCP 服务
app = FastMCP("File MCP")


@app.tool()
def create_file(file_path: str, content: str = "") -> Dict[str, Any]:
    """
    创建文件并写入内容
    
    Args:
        file_path: 文件路径（相对或绝对路径）
        content: 文件内容，默认为空
        
    Returns:
        包含操作状态的字典
    """
    try:
        # 确保目录存在
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        
        # 写入文件
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
            
        return {
            "status": "success",
            "message": f"文件 '{file_path}' 创建成功",
            "file_path": file_path,
            "content_length": len(content)
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def read_file(file_path: str) -> Dict[str, Any]:
    """
    读取文件内容
    
    Args:
        file_path: 文件路径
        
    Returns:
        包含文件内容的字典
    """
    try:
        if not os.path.exists(file_path):
            return {"error": f"文件 '{file_path}' 不存在"}
            
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        return {
            "status": "success",
            "file_path": file_path,
            "content": content,
            "content_length": len(content)
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def append_to_file(file_path: str, content: str) -> Dict[str, Any]:
    """
    向文件追加内容
    
    Args:
        file_path: 文件路径
        content: 要追加的内容
        
    Returns:
        包含操作状态的字典
    """
    try:
        if not os.path.exists(file_path):
            return {"error": f"文件 '{file_path}' 不存在"}
            
        with open(file_path, 'a', encoding='utf-8') as f:
            f.write(content)
            
        return {
            "status": "success",
            "message": f"内容已成功追加到文件 '{file_path}'",
            "file_path": file_path,
            "appended_length": len(content)
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def list_files(directory: str = ".") -> Dict[str, Any]:
    """
    列出目录中的文件和文件夹
    
    Args:
        directory: 目录路径，默认为当前目录
        
    Returns:
        包含文件列表的字典
    """
    try:
        if not os.path.exists(directory):
            return {"error": f"目录 '{directory}' 不存在"}
            
        items = os.listdir(directory)
        files = []
        directories = []
        
        for item in items:
            item_path = os.path.join(directory, item)
            if os.path.isfile(item_path):
                files.append(item)
            else:
                directories.append(item)
                
        return {
            "status": "success",
            "directory": directory,
            "files": sorted(files),
            "directories": sorted(directories),
            "total_files": len(files),
            "total_directories": len(directories)
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def file_info(file_path: str) -> Dict[str, Any]:
    """
    获取文件信息
    
    Args:
        file_path: 文件路径
        
    Returns:
        包含文件信息的字典
    """
    try:
        if not os.path.exists(file_path):
            return {"error": f"文件 '{file_path}' 不存在"}
            
        stat_info = os.stat(file_path)
        
        return {
            "status": "success",
            "file_path": file_path,
            "size": stat_info.st_size,
            "created_time": stat_info.st_ctime,
            "modified_time": stat_info.st_mtime,
            "is_file": os.path.isfile(file_path),
            "is_directory": os.path.isdir(file_path)
        }
    except Exception as e:
        return {"error": str(e)}


if __name__ == "__main__":
    app.run(transport="stdio")
