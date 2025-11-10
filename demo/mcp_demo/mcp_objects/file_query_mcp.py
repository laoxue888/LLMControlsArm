import os
import glob
import fnmatch
from typing import Dict, Any, List
from fastmcp import FastMCP

# 初始化 MCP 服务
app = FastMCP("File Query MCP")


@app.tool()
def search_files_by_name(directory: str = ".", pattern: str = "*", recursive: bool = True) -> Dict[str, Any]:
    """
    根据文件名模式搜索文件
    
    Args:
        directory: 搜索的目录路径，默认为当前目录
        pattern: 文件名模式，支持通配符 (*, ?, [])
        recursive: 是否递归搜索子目录，默认为True
        
    Returns:
        包含匹配文件列表的字典
    """
    try:
        if not os.path.exists(directory):
            return {"error": f"目录 '{directory}' 不存在"}
            
        if not os.path.isdir(directory):
            return {"error": f"'{directory}' 不是目录"}
        
        matches = []
        
        if recursive:
            # 递归搜索所有子目录
            for root, dirs, files in os.walk(directory):
                for file in files:
                    if fnmatch.fnmatch(file, pattern):
                        full_path = os.path.join(root, file)
                        relative_path = os.path.relpath(full_path, directory)
                        matches.append({
                            "name": file,
                            "path": full_path,
                            "relative_path": relative_path,
                            "directory": root,
                            "size": os.path.getsize(full_path) if os.path.isfile(full_path) else 0
                        })
        else:
            # 仅搜索当前目录
            for item in os.listdir(directory):
                item_path = os.path.join(directory, item)
                if os.path.isfile(item_path) and fnmatch.fnmatch(item, pattern):
                    matches.append({
                        "name": item,
                        "path": item_path,
                        "relative_path": item,
                        "directory": directory,
                        "size": os.path.getsize(item_path)
                    })
        
        return {
            "status": "success",
            "directory": directory,
            "pattern": pattern,
            "recursive": recursive,
            "matches": matches,
            "total_matches": len(matches)
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def search_files_by_content(directory: str = ".", search_text: str = "", 
                          file_pattern: str = "*", recursive: bool = True) -> Dict[str, Any]:
    """
    根据文件内容搜索文件
    
    Args:
        directory: 搜索的目录路径，默认为当前目录
        search_text: 要搜索的文本内容
        file_pattern: 文件名模式，支持通配符
        recursive: 是否递归搜索子目录，默认为True
        
    Returns:
        包含匹配文件列表的字典
    """
    try:
        if not os.path.exists(directory):
            return {"error": f"目录 '{directory}' 不存在"}
            
        if not os.path.isdir(directory):
            return {"error": f"'{directory}' 不是目录"}
            
        if not search_text:
            return {"error": "搜索文本不能为空"}
        
        matches = []
        
        def search_in_file(file_path: str) -> List[Dict[str, Any]]:
            """在单个文件中搜索文本"""
            try:
                with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                    if search_text in content:
                        # 计算行号和具体位置
                        lines = content.split('\n')
                        matching_lines = []
                        for i, line in enumerate(lines, 1):
                            if search_text in line:
                                matching_lines.append({
                                    "line_number": i,
                                    "line_content": line.strip(),
                                    "position": line.find(search_text)
                                })
                        
                        return [{
                            "file_path": file_path,
                            "relative_path": os.path.relpath(file_path, directory),
                            "matching_lines": matching_lines,
                            "total_matches": len(matching_lines)
                        }]
            except Exception:
                # 跳过无法读取的文件（二进制文件等）
                pass
            return []
        
        if recursive:
            # 递归搜索所有子目录
            for root, dirs, files in os.walk(directory):
                for file in files:
                    if fnmatch.fnmatch(file, file_pattern):
                        full_path = os.path.join(root, file)
                        matches.extend(search_in_file(full_path))
        else:
            # 仅搜索当前目录
            for item in os.listdir(directory):
                item_path = os.path.join(directory, item)
                if os.path.isfile(item_path) and fnmatch.fnmatch(item, file_pattern):
                    matches.extend(search_in_file(item_path))
        
        return {
            "status": "success",
            "directory": directory,
            "search_text": search_text,
            "file_pattern": file_pattern,
            "recursive": recursive,
            "matches": matches,
            "total_files_matched": len(matches)
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def get_directory_tree(directory: str = ".", max_depth: int = 3) -> Dict[str, Any]:
    """
    获取目录树结构
    
    Args:
        directory: 目录路径，默认为当前目录
        max_depth: 最大递归深度，默认为3
        
    Returns:
        包含目录树结构的字典
    """
    try:
        if not os.path.exists(directory):
            return {"error": f"目录 '{directory}' 不存在"}
            
        if not os.path.isdir(directory):
            return {"error": f"'{directory}' 不是目录"}
        
        def build_tree(current_dir: str, current_depth: int = 0) -> Dict[str, Any]:
            if current_depth > max_depth:
                return {"name": os.path.basename(current_dir), "type": "directory", "truncated": True}
            
            tree = {
                "name": os.path.basename(current_dir),
                "path": current_dir,
                "type": "directory",
                "files": [],
                "directories": []
            }
            
            try:
                items = os.listdir(current_dir)
                for item in items:
                    item_path = os.path.join(current_dir, item)
                    if os.path.isfile(item_path):
                        tree["files"].append({
                            "name": item,
                            "path": item_path,
                            "size": os.path.getsize(item_path),
                            "modified_time": os.path.getmtime(item_path)
                        })
                    elif os.path.isdir(item_path):
                        tree["directories"].append(build_tree(item_path, current_depth + 1))
            except PermissionError:
                tree["permission_denied"] = True
            
            return tree
        
        tree = build_tree(directory)
        
        return {
            "status": "success",
            "directory": directory,
            "max_depth": max_depth,
            "tree": tree
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def find_large_files(directory: str = ".", min_size_mb: float = 10, 
                    recursive: bool = True) -> Dict[str, Any]:
    """
    查找大文件
    
    Args:
        directory: 搜索的目录路径，默认为当前目录
        min_size_mb: 最小文件大小（MB）
        recursive: 是否递归搜索子目录，默认为True
        
    Returns:
        包含大文件列表的字典
    """
    try:
        if not os.path.exists(directory):
            return {"error": f"目录 '{directory}' 不存在"}
            
        if not os.path.isdir(directory):
            return {"error": f"'{directory}' 不是目录"}
        
        min_size_bytes = min_size_mb * 1024 * 1024
        large_files = []
        
        def scan_directory(current_dir: str):
            try:
                items = os.listdir(current_dir)
                for item in items:
                    item_path = os.path.join(current_dir, item)
                    if os.path.isfile(item_path):
                        file_size = os.path.getsize(item_path)
                        if file_size >= min_size_bytes:
                            large_files.append({
                                "name": item,
                                "path": item_path,
                                "size_bytes": file_size,
                                "size_mb": round(file_size / (1024 * 1024), 2),
                                "directory": current_dir
                            })
                    elif os.path.isdir(item_path) and recursive:
                        scan_directory(item_path)
            except PermissionError:
                pass  # 跳过没有权限的目录
        
        scan_directory(directory)
        
        # 按文件大小排序
        large_files.sort(key=lambda x: x["size_bytes"], reverse=True)
        
        return {
            "status": "success",
            "directory": directory,
            "min_size_mb": min_size_mb,
            "recursive": recursive,
            "large_files": large_files,
            "total_large_files": len(large_files),
            "total_size_mb": round(sum(f["size_bytes"] for f in large_files) / (1024 * 1024), 2)
        }
    except Exception as e:
        return {"error": str(e)}


@app.tool()
def get_file_system_info(directory: str = ".") -> Dict[str, Any]:
    """
    获取文件系统信息
    
    Args:
        directory: 目录路径，默认为当前目录
        
    Returns:
        包含文件系统信息的字典
    """
    try:
        if not os.path.exists(directory):
            return {"error": f"目录 '{directory}' 不存在"}
            
        directory = os.path.abspath(directory)
        
        # 获取磁盘使用情况
        statvfs = os.statvfs(directory)
        total_space = statvfs.f_blocks * statvfs.f_frsize
        free_space = statvfs.f_bfree * statvfs.f_frsize
        used_space = total_space - free_space
        
        # 统计文件和目录数量
        file_count = 0
        dir_count = 0
        total_size = 0
        
        for root, dirs, files in os.walk(directory):
            dir_count += len(dirs)
            file_count += len(files)
            for file in files:
                try:
                    file_path = os.path.join(root, file)
                    total_size += os.path.getsize(file_path)
                except (OSError, IOError):
                    pass  # 跳过无法访问的文件
        
        return {
            "status": "success",
            "directory": directory,
            "disk_usage": {
                "total_gb": round(total_space / (1024**3), 2),
                "used_gb": round(used_space / (1024**3), 2),
                "free_gb": round(free_space / (1024**3), 2),
                "usage_percentage": round((used_space / total_space) * 100, 2)
            },
            "file_statistics": {
                "total_files": file_count,
                "total_directories": dir_count,
                "total_size_gb": round(total_size / (1024**3), 2)
            },
            "path_info": {
                "absolute_path": directory,
                "parent_directory": os.path.dirname(directory),
                "exists": True,
                "is_directory": True
            }
        }
    except Exception as e:
        return {"error": str(e)}


if __name__ == "__main__":
    app.run(transport="stdio")
