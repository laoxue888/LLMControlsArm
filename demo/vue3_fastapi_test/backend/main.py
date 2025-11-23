from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional

# 创建FastAPI应用实例
app = FastAPI(
    title="Vue3 FastAPI Backend",
    description="一个与Vue3前端配合的FastAPI后端服务",
    version="1.0.0"
)

# 配置CORS中间件，允许前端访问
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5174", "http://127.0.0.1:5174"],  # Vue开发服务器默认端口
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 数据模型定义
class User(BaseModel):
    id: int
    name: str
    email: str
    age: Optional[int] = None

class CreateUserRequest(BaseModel):
    name: str
    email: str
    age: Optional[int] = None

# 模拟数据库
users_db = [
    User(id=1, name="张三", email="zhangsan@example.com", age=25),
    User(id=2, name="李四", email="lisi@example.com", age=30),
    User(id=3, name="王五", email="wangwu@example.com", age=28),
]

# 路由定义
@app.get("/")
async def root():
    return {"message": "欢迎使用Vue3 FastAPI后端服务", "status": "success"}

@app.get("/api/health")
async def health_check():
    return {"status": "healthy", "timestamp": "2025-01-23T08:53:00Z"}

@app.get("/api/users", response_model=List[User])
async def get_users():
    """获取所有用户"""
    return users_db

@app.get("/api/users/{user_id}", response_model=User)
async def get_user(user_id: int):
    """根据ID获取用户"""
    user = next((user for user in users_db if user.id == user_id), None)
    if user is None:
        return {"error": "用户不存在"}
    return user

@app.post("/api/users", response_model=User)
async def create_user(user: CreateUserRequest):
    """创建新用户"""
    new_id = max([u.id for u in users_db]) + 1 if users_db else 1
    new_user = User(
        id=new_id,
        name=user.name,
        email=user.email,
        age=user.age
    )
    users_db.append(new_user)
    return new_user

@app.put("/api/users/{user_id}", response_model=User)
async def update_user(user_id: int, user: CreateUserRequest):
    """更新用户信息"""
    existing_user = next((u for u in users_db if u.id == user_id), None)
    if existing_user is None:
        return {"error": "用户不存在"}
    
    existing_user.name = user.name
    existing_user.email = user.email
    existing_user.age = user.age
    return existing_user

@app.delete("/api/users/{user_id}")
async def delete_user(user_id: int):
    """删除用户"""
    global users_db
    user = next((u for u in users_db if u.id == user_id), None)
    if user is None:
        return {"error": "用户不存在"}
    
    users_db = [u for u in users_db if u.id != user_id]
    return {"message": "用户删除成功", "user_id": user_id}

# 运行应用
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
