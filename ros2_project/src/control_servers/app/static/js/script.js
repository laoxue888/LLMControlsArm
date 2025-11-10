const API_BASE = 'http://localhost:8000';
let messageLog = document.querySelector('.message-log');

// 添加消息到日志
function addMessage(message, type = 'success') {
    const now = new Date();
    const timeString = now.toLocaleTimeString();
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry ${type}`;
    logEntry.innerHTML = `<span class="log-time">[${timeString}]</span> ${message}`;
    messageLog.appendChild(logEntry);
    messageLog.scrollTop = messageLog.scrollHeight;
}

// 更新状态显示
async function updateStatus() {
    try {
        const response = await fetch(`${API_BASE}/status`);
        const status = await response.json();
        
        // 安全地更新状态显示，检查元素是否存在
        const statusElements = {
            'statusJoint1': `${status.joint1_angle}°`,
            'statusJoint2': `${status.joint2_angle}°`,
            'statusJoint3': `${status.joint3_angle}°`,
            'statusJoint4': `${status.joint4_angle}°`,
            'statusJoint5': `${status.joint5_angle}°`,
            'statusJoint6': `${status.joint6_angle}°`,
            'statusJoint7': `${status.joint7_angle}°`,
            'statusGripper': status.gripper_open ? '打开' : '关闭',
            'statusGripperSize': `${status.gripper_size}%`
        };
        
        for (const [id, value] of Object.entries(statusElements)) {
            const element = document.getElementById(id);
            if (element) {
                element.textContent = value;
            }
        }
        
        // 安全地同步滑块位置
        const sliderElements = {
            'joint1Slider': status.joint1_angle,
            'joint2Slider': status.joint2_angle,
            'joint3Slider': status.joint3_angle,
            'joint4Slider': status.joint4_angle,
            'joint5Slider': status.joint5_angle,
            'joint6Slider': status.joint6_angle,
            'joint7Slider': status.joint7_angle,
            'gripperSizeSlider': status.gripper_size
        };
        
        for (const [id, value] of Object.entries(sliderElements)) {
            const element = document.getElementById(id);
            if (element) {
                element.value = value;
            }
        }
        
        updateSliderDisplays();
        
    } catch (error) {
        addMessage('获取状态失败: ' + error.message, 'error');
    }
}

// 更新滑块显示值
function updateSliderDisplays() {
    const displayElements = {
        'joint1Value': 'joint1Slider',
        'joint2Value': 'joint2Slider',
        'joint3Value': 'joint3Slider',
        'joint4Value': 'joint4Slider',
        'joint5Value': 'joint5Slider',
        'joint6Value': 'joint6Slider',
        'joint7Value': 'joint7Slider',
        'gripperSizeValue': 'gripperSizeSlider'
    };
    
    for (const [displayId, sliderId] of Object.entries(displayElements)) {
        const displayElement = document.getElementById(displayId);
        const sliderElement = document.getElementById(sliderId);
        
        if (displayElement && sliderElement) {
            const suffix = sliderId === 'gripperSizeSlider' ? '%' : '°';
            displayElement.textContent = sliderElement.value + suffix;
        }
    }
}

// 控制机械臂关节
async function controlJoint(joint, value) {
    try {
        const response = await fetch(`${API_BASE}/control`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                joint: joint,
                value: parseFloat(value)
            })
        });
        
        const result = await response.json();
        addMessage(result.message);
        await updateStatus();
        
    } catch (error) {
        addMessage(`控制${joint}关节失败: ` + error.message, 'error');
    }
}


// 控制夹具开度大小
async function controlGripperSize(size) {
    try {
        const response = await fetch(`${API_BASE}/gripper/size`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                gripper_size: parseFloat(size)
            })
        });
        
        const result = await response.json();
        addMessage(result.message);
        await updateStatus();
        
    } catch (error) {
        addMessage('控制夹具开度失败: ' + error.message, 'error');
    }
}


// 重置机械臂
async function resetArm() {
    try {
        const response = await fetch(`${API_BASE}/reset`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
        });
        
        const result = await response.json();
        addMessage(result.message);
        await updateStatus();
        
    } catch (error) {
        addMessage('重置机械臂失败: ' + error.message, 'error');
    }
}

// 事件监听器
document.addEventListener('DOMContentLoaded', function() {
    // 初始化状态
    updateStatus();
    
    // 滑块事件
    const sliders = document.querySelectorAll('.slider');
    sliders.forEach(slider => {
        slider.addEventListener('input', function() { // 实时更新显示值
            updateSliderDisplays();
        });
        
        slider.addEventListener('change', function() { // 仅在释放滑块时发送请求
            const joint = this.id.replace('Slider', '');
            if (joint === 'gripperSize') {
                controlGripperSize(this.value);
            } else {
                controlJoint(joint, this.value);
            }
        });
    });



    // 重置按钮
    document.getElementById('resetArm').addEventListener('click', function() {
        resetArm();
    });

    // 定期更新状态
    setInterval(updateStatus, 2000);
});
