# coding=utf-8

import sys
from PySide6.QtWidgets import QFileDialog, QListView, QAbstractItemView, QTreeView, QTableView
import re
from nptdms import TdmsWriter, ChannelObject
import numpy as np
from nptdms.types import TimeStamp
import os
import zipfile
import requests


def download_and_extract_zip(url, download_folder, extract_folder=None):
    """
    下载ZIP文件并解压到指定目录

    参数:
        url: ZIP文件的URL
        download_folder: 下载文件保存的目录
        extract_folder: 解压目录(默认为下载目录)
    """
    # 如果未指定解压目录，则使用下载目录
    if extract_folder is None:
        extract_folder = download_folder

    # 确保目录存在
    os.makedirs(download_folder, exist_ok=True)
    os.makedirs(extract_folder, exist_ok=True)

    try:
        # 从URL获取文件名
        zip_filename = os.path.join(download_folder, url.split('/')[-1])

        print(f"正在下载 {url}...")
        # 下载文件
        response = requests.get(url, stream=True)
        response.raise_for_status()  # 检查请求是否成功

        # 写入文件
        with open(zip_filename, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)

        print(f"文件已下载到: {zip_filename}")

        # 解压文件
        print(f"正在解压到 {extract_folder}...")
        with zipfile.ZipFile(zip_filename, 'r') as zip_ref:
            zip_ref.extractall(extract_folder)

        print("解压完成!")

        return True

    except Exception as e:
        print(f"发生错误: {e}")
        return False

def find_dir_path(dir_name):
    """
    作用：查看包含folder_name的模块路径，并返回路径
    """
    # folder_name = 'renamedata'
    folder_name = dir_name
    for path in sys.path:
        # print(path)
        if folder_name in path:
            return path
    return None

def getFoldersPath(m_directory):
    """
    作用：选择多个文件夹的对话框，并获取这些文件夹的路径。
    """
    fileDialog = QFileDialog()
    fileDialog.setDirectory(m_directory)
    fileDialog.setFileMode(QFileDialog.FileMode.Directory)
    fileDialog.setOption(QFileDialog.Option.DontUseNativeDialog, True)
    listView: QListView = fileDialog.findChild(QListView, "listView")
    if listView:
        listView.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)

    treeView:QTreeView = fileDialog.findChild(QTreeView, "treeView")
    if treeView:
        treeView.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
    
    if fileDialog.exec_():
        folders = fileDialog.selectedFiles()
        return folders
    return None

def extract_numbers_from_string(string):
    pattern = r'\d+$'  # 匹配字符串末尾的数字
    match = re.search(pattern, string)
    if match:
        return match.group()
    else:
        return None
    


def get_listview_items(listview: QListView):
        model: QListView = listview.model()
        item_list = []
        for row in range(model.rowCount()):
            index = model.index(row, 0)
            item = model.data(index, 0)
            item_list.append(item)
        return item_list


def find_all_diff_indices(arr, threshold=1):
    """
    Python找出所有数组中后一个减前一个等于1的元素索引，然后封装成函数
    """
    diff_indices = []
    for i in range(1, len(arr)):
        if arr[i] - arr[i-1] == threshold: # 上升沿
            diff_indices.append(i) # 在这里找到对应的索引，从而找到时间点
    return diff_indices

def find_first_last_indices(arr, threshold=1):
    """
    截取数据段
    """
    diff_indices = []
    for i in range(1, len(arr)):
        if arr[i] > threshold: # 大于阈值的第一个数
            diff_indices.append(i)
            break
    for i in range(len(arr)-1, -1, -1):
        if arr[i] > threshold: # 大于阈值的倒数第一个数
            diff_indices.append(i)
            break
    return diff_indices

# 读取txt文件
def read_large_file(file_path):
    with open(file_path, 'r') as file:
        for line in file:
            yield line


def rename_duplicates(my_list):
    """重命名重复元素"""
    counts = {}
    new_list = []

    for item in my_list:
        if item not in counts:
            counts[item] = 1
            new_list.append(item)
        else:
            counts[item] += 1
            new_list.append(f"{item}_{counts[item]}")
    return new_list

